#!/usr/bin/env python3
"""
baro_range_monitor.py -- "气压计监视测距" 一致性监视器（机载电脑侧，纯标准库）

目的
----
下视测距给出"合法但错误"的读数时（典型：飞在空中却读得偏小），飞控会把测距
当真，高度估计被带偏，飞机冲高；最坏情况（读数从起飞就持续偏小）飞控以为自己在
2 m、实际飞到 4~5 m，没有任何保护触发。

本监视器用气压计给测距"作证"：气压计短时间内不会跳变，但会慢漂。
两者对不上（持续偏差过大，或 2 s 内相对变化过大）即判定测距不可信，由调用方
采取动作（报警并切 LAND / 切换到气压高度源组 / 只切源组）。

输入（全部来自飞控的 MAVLink 流，20 Hz）
----------------------------------------
  RANGEFINDER.distance      m      测距读数（到地面的距离）
  ATTITUDE.roll/pitch       rad    机体姿态，用于把斜距折算成垂直高度
  SCALED_PRESSURE.press_abs hPa    绝对气压
  解锁状态 / 当前模式

算法（按任务书规定实现，不自行改动）
------------------------------------
  h_r = distance * cos(roll) * cos(pitch)              测距垂直高度
  h_b = 44330 * (1 - (p / p0)^0.1903)                  气压高度（p0 = 解锁前 2 s 平均气压）
  1) h_r、h_b 各过一阶低通，截止频率 fc
  2) 偏置 b 跟踪 (h_b - h_r)，一阶低通，时间常数 tau；**只在"当前判定为一致"时更新**，
     用来吃掉气压计的慢漂；**解锁时 b 置为当时的 (h_b - h_r)**
  3) 残差 e = h_b - h_r - b
  4) 判据 A（偏差）：|e| > E 持续 T 秒
  5) 判据 B（变化率）：过去 2 s 内 (Δh_b - Δh_r) 的绝对值 > R
  6) 只有在"已解锁且 h_b > 0.15 m"之后才启用；测距本身无效（超量程 / 无数据 /
     长时间没有新读数）时不判定（那种情况飞控自己会处理）

依赖：仅 Python 标准库。移植到真机时直接把本文件拷过去即可。

作者：val7 验证任务
"""
from __future__ import annotations

import math
from collections import deque

# 气压高度公式常数（国际标准大气）
BARO_K = 44330.0
BARO_EXP = 0.1903


class BaroRangeMonitor:
    """气压计 / 测距一致性监视器。

    参数
    ----
    fc        : h_r、h_b 低通截止频率 [Hz]
    tau       : 偏置 b 的低通时间常数 [s]
    E         : 判据 A 的残差门限 [m]
    T         : 判据 A 需要持续的时间 [s]
    R         : 判据 B 的 2 s 变化率门限 [m]
    baro_h_min: 启用所需的最低气压高度 [m]（已离地）
    p0_window : p0 取样窗（解锁前多少秒的平均气压）[s]
    rng_min   : 测距读数低于此值视为无效 [m]
    rng_max   : 测距读数高于此值视为无效 [m]
    baro_timeout: 气压消息超过多少秒没有更新视为断流 [s]
    p0_bias_m : p0 被取错造成的等效气压高度偏差 [m]
                （飞机在解锁前被拿起来 d 米 → p0 取自抬起的瞬间 → 之后算出的
                 h_b 比真实高度低 d 米，所以这里取 -d）
    slope    : 【建议一节的单独实验用，默认 False = 任务书原算法】
                True 时偏置带一个线性项：残差改成
                e = (h_b - h_r) - b - s*(t - t_ref)，s 是 (h_b-h_r) 变化率的低通估计，
                用来抵消气压计的线性慢漂。
    tau_s    : 上面斜率估计的时间常数 [s]
    b_hold   : 【建议一节的单独实验用，默认 0 = 任务书原算法】
                判据 B 需要超限连续保持 b_hold 秒才触发（0 = 立即触发）。
    """

    def __init__(self, fc=1.0, tau=30.0, E=0.5, T=1.0, R=0.6,
                 baro_h_min=0.15, p0_window=2.0, rng_min=0.02, rng_max=7.9,
                 baro_timeout=0.5, p0_bias_m=0.0, slope=False, tau_s=30.0, b_hold=0.0):
        self.fc = float(fc)
        self.tau = float(tau)
        self.E = float(E)
        self.T = float(T)
        self.R = float(R)
        self.baro_h_min = float(baro_h_min)
        self.p0_window = float(p0_window)
        self.rng_min = float(rng_min)
        self.rng_max = float(rng_max)
        self.baro_timeout = float(baro_timeout)
        self.p0_bias_m = float(p0_bias_m)
        # --- 可选：斜率补偿（报告"建议"一节的单独实验用，默认关闭，主结果不受影响）---
        self.slope = bool(slope)     # 偏置是否带线性项
        self.tau_s = float(tau_s)   # 斜率估计的时间常数 [s]
        self.b_hold = float(b_hold)  # 判据 B 需要超限持续的时长 [s]（建议实验用，默认 0）
        self.rate_bad_t = 0.0
        self.s = 0.0                # 斜率估计 [m/s]
        self.t_ref = None           # 线性模型参考时刻
        self.d_prev = None          # 上一帧的 (h_b - h_r)

        self._tau_f = 1.0 / (2.0 * math.pi * self.fc)   # 低通时间常数
        self._press_hist = deque()      # (t, p) 气压样本历史，用于算 p0
        self._hist = deque()            # (t, h_b, h_r) 滤波后的历史，用于 2 s 变化率
        self.p0 = None                  # 参考气压 [hPa]
        self.h_b = None                 # 滤波后的气压高度 [m]
        self.h_r = None                 # 滤波后的测距垂直高度 [m]
        self.b = 0.0                    # 偏置 [m]
        self.b_set = False              # b 是否已在解锁时（或首个有效帧）置好
        self.e = 0.0                    # 残差 [m]
        self.armed = False
        self.enabled = False
        self.t_arm = None
        self.t_exceed = 0.0             # |e| > E 已经持续了多久 [s]
        self.trig_A = False             # 判据 A 触发（粘滞）
        self.trig_B = False             # 判据 B 触发（粘滞）
        self.t_trig_A = None
        self.t_trig_B = None
        self.t_trig = None              # 任一判据首次触发时刻
        self.reason = None              # 'A' / 'B' / 'AB'
        self.baro_stale = False
        self.rng_valid = False
        self._last_t = None
        self._last_press_t = None

    # ------------------------------------------------------------------ 工具
    def _alpha(self, dt, tau):
        return dt / (dt + tau) if dt > 0 else 0.0

    def range_valid(self, distance):
        """测距读数本身是否有效（飞控已经判为超量程 / 无数据时不判定）。"""
        if distance is None or not math.isfinite(distance):
            return False
        return self.rng_min <= distance <= self.rng_max

    # ------------------------------------------------------------------ 主循环
    def step(self, t, distance, roll, pitch, press_abs, armed, mode=None):
        """喂一条 20 Hz 的帧（以 RANGEFINDER 消息为时钟），返回状态字典。

        t          : FC 时间（秒，time_boot_ms/1000，单调）
        distance   : 测距读数 [m]（None = 本帧没有新读数）
        roll/pitch : 姿态 [rad]
        press_abs  : 绝对气压 [hPa]（None = 本帧没有新气压）
        armed      : 飞控是否解锁
        mode       : 当前模式（仅记录）
        """
        first = self._last_t is None
        dt = 0.0 if first else max(0.0, t - self._last_t)
        self._last_t = t

        # --- 气压样本：解锁前累积，用于 p0 ---
        if press_abs is not None:
            self._last_press_t = t
            self._press_hist.append((t, press_abs))
            while self._press_hist and t - self._press_hist[0][0] > self.p0_window + 1.0:
                self._press_hist.popleft()

        # --- 解锁瞬间：取 p0、置 b ---
        arm_now = bool(armed) and not self.armed
        if arm_now:
            self.armed = True
            self.t_arm = t
            samples = [p for (tt, p) in self._press_hist if t - tt <= self.p0_window]
            if samples:
                self.p0 = sum(samples) / len(samples)
            self.b = 0.0
            self.b_set = False
        elif (not armed) and self.armed:
            # 重新解锁 / 落地解锁：重置
            self.armed = False
            self.enabled = False
            self.trig_A = self.trig_B = False
            self.t_trig = self.t_trig_A = self.t_trig_B = None
            self.reason = None
            self.t_exceed = 0.0
            self.rate_bad_t = 0.0
            self.p0 = None
            self.h_b = self.h_r = None
            self.b = 0.0
            self.b_set = False
            self.s = 0.0
            self.t_ref = None
            self.d_prev = None

        # --- 气压断流检测 ---
        self.baro_stale = (self._last_press_t is None) or (t - self._last_press_t > self.baro_timeout)

        # --- 原始量 ---
        h_b_raw = None
        if press_abs is not None and self.p0:
            p = press_abs / self.p0
            h_b_raw = BARO_K * (1.0 - p ** BARO_EXP) - self.p0_bias_m
        h_r_raw = None
        if distance is not None and math.isfinite(distance) \
                and (not getattr(self, "skip_invalid", False) or self.range_valid(distance)):
            h_r_raw = distance * math.cos(roll) * math.cos(pitch)

        # --- 解锁瞬间直接置 b = h_b - h_r（任务书要求） ---
        if arm_now:
            if h_b_raw is not None and h_r_raw is not None:
                self.b = h_b_raw - h_r_raw
                self.b_set = True
            self.s = 0.0
            self.t_ref = t
            self.d_prev = None

        # --- 滤波 ---
        if self.h_b is None:
            self.h_b = h_b_raw
            self.h_r = h_r_raw
        else:
            if h_b_raw is not None:
                self.h_b += self._alpha(dt, self._tau_f) * (h_b_raw - self.h_b)
            if h_r_raw is not None:
                self.h_r += self._alpha(dt, self._tau_f) * (h_r_raw - self.h_r)

        if self.h_b is None or self.h_r is None:
            return self.state(mode)

        # --- 测距有效性 ---
        self.rng_valid = self.range_valid(distance)

        # --- 启用条件：已解锁且离地 ---
        self.enabled = bool(self.armed and self.h_b > self.baro_h_min)

        # --- 残差 ---
        self.e = self.h_b - self.h_r - self.b
        pred = self.b
        if self.slope:
            d = self.h_b - self.h_r
            if self.d_prev is not None and dt > 0:
                self.s += self._alpha(dt, self.tau_s) * ((d - self.d_prev) / dt - self.s)
            self.d_prev = d
            if self.t_ref is None:
                self.t_ref = t
            pred = self.b + self.s * (t - self.t_ref)
            self.e = d - pred

        # --- 判据 B：过去 2 s 内 Δh_b - Δh_r ---
        rate_bad = False
        self._hist.append((t, self.h_b, self.h_r))
        while self._hist and t - self._hist[0][0] > 2.0:
            self._hist.popleft()
        if len(self._hist) >= 2:
            t_ref, hb_ref, hr_ref = self._hist[0]
            if t - t_ref >= 1.5:          # 至少 1.5 s 的窗口，避免刚开机时误判
                if abs((self.h_b - hb_ref) - (self.h_r - hr_ref)) > self.R:
                    rate_bad = True

        # --- 未启用 / 测距无效 / 气压断流：不判定 ---
        if not self.enabled or not self.rng_valid or self.baro_stale:
            self.t_exceed = 0.0
            return self.state(mode)

        # --- 首个可用帧：补置 b（解锁时若还没有测距读数） ---
        if not self.b_set:
            self.b = self.h_b - self.h_r
            self.b_set = True
            self.e = 0.0
            self.s = 0.0
            self.t_ref = t
            self.d_prev = None
            return self.state(mode)

        # --- 判据 A：|e| > E 持续 T 秒 ---
        if abs(self.e) > self.E:
            self.t_exceed += dt
        else:
            self.t_exceed = 0.0
        if (not self.trig_A) and self.t_exceed >= self.T:
            self.trig_A = True
            self.t_trig_A = t

        # --- 判据 B 触发 ---
        self.rate_bad_t = self.rate_bad_t + dt if rate_bad else 0.0
        if (not self.trig_B) and rate_bad and self.rate_bad_t >= self.b_hold:
            self.trig_B = True
            self.t_trig_B = t

        # --- 偏置更新：只在"当前判定为一致"时更新 ---
        consistent = (abs(self.e) <= self.E) and (not rate_bad) and (not self.trig_A)
        if consistent:
            if self.slope:
                self.b = pred + self._alpha(dt, self.tau) * ((self.h_b - self.h_r) - pred)
                self.t_ref = t
            else:
                self.b += self._alpha(dt, self.tau) * ((self.h_b - self.h_r) - self.b)

        # --- 触发汇总 ---
        if self.t_trig is None and (self.trig_A or self.trig_B):
            self.t_trig = t
            self.reason = "AB" if (self.trig_A and self.trig_B) else ("A" if self.trig_A else "B")
        return self.state(mode)

    # ------------------------------------------------------------------ 输出
    def state(self, mode=None):
        return {"t": self._last_t, "h_r": self.h_r, "h_b": self.h_b, "b": self.b, "e": self.e,
                "enabled": self.enabled, "armed": self.armed, "trig_A": self.trig_A, "trig_B": self.trig_B,
                "trigger": self.trig_A or self.trig_B, "reason": self.reason, "t_trig": self.t_trig,
                "t_trig_A": self.t_trig_A, "t_trig_B": self.t_trig_B, "t_exceed": self.t_exceed,
                "baro_stale": self.baro_stale, "rng_valid": self.rng_valid, "mode": mode}

    @property
    def triggered(self):
        return self.trig_A or self.trig_B
