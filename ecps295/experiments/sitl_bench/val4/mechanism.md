# 机理草稿（V4 第 2 项，基于 v4_h200n01_a_r1..3 / v4_orig_a_r1 数据帧与源码；表格数据出来后并入 report.md）

## 注入时的实测时序（RFND.Stat：4=Good，2=OutOfRangeLow；时间相对注入首帧）

v4_h200n01_a_r1（FINAL，注入 t≈4.45 s、恢复 t≈5.45 s，均为 AUTO 接通后）：

| 注入后 | RFND.Dist | Stat | EKF 高度 | 真值 | CTUN.DAlt | XKF4.SH(高度检验比) |
|---|---|---|---|---|---|---|
| +0.4..+0.8 | 0.00 | 2 | 1.24→1.64 | 1.36→1.56 | 跟随 EKF | 0.06→0.01 |
| +0.9/+1.0 | 0.02/0.06 | 4 | 1.69/1.74 | 1.61/1.66 | 跟随 | 0.00/0.02 |
| +1.1 | 1.65 | 4 | **0.04** | 1.71 | 0.04 | 0.00 |
| +1.2..+2.9 | 1.73→2.60 | 4 | 0.09→0.94 | 1.76→2.63 | =EKF | 6.8→4.4 |
| （orig 同格）+1.2..+2.9 | 1.75→2.60 | 4 | 0.15→1.74 | 1.78→2.63 | 0.13→1.42 | 0.58→0.04 |

- 故障窗内（读数 0，OutOfRangeLow）：**EKF 没有塌**，估计保持在真值附近（1.24–1.64 vs 1.36–1.56）——EKF 退到气压计。
- 塌缩发生在真值爬过 1.55 m 后：读数 = 真值 − 1.5 ≈ 0.02–0.08 m，刚好大于 `RNGFND1_MIN 0.01` → **状态变回 Good**，被当成有效高度融合，EKF 一步跳到 ≈0.04 m。
- 恢复后（读数 ≈ 真值）：innov ≈ 1.6–1.7 m。FINAL（NSE 0.1、gate 200=2σ）→ SH=6.8 全部拒绝，EKF 只靠 IMU 慢慢漂（0.05 m/0.1 s），控制目标跟随 EKF → 飞机带着 1.7 m 的高度估计误差爬升；5 s 后高度超时强制重置（无 STATUSTEXT），EKF 跳到当前读数 3.6–3.7 m → 越过围栏触发线（2.5+0.3）→ LAND。orig（NSE 0.5、gate 500=5σ）→ SH=0.58 通过，约 1.6 s 收敛回真值，任务完成。

## 源码链（文件:行）

1. 读数截 0：`libraries/SITL/SIM_Aircraft.cpp:635`（读数加噪声后返回）；`libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp:49` `state.distance_m = MAX(0, dist)`——负读数截成 0；`libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp:62-70` `< min_distance()`（`RNGFND1_MIN 0.01`）→ `OutOfRangeLow`；枚举 `AP_RangeFinder.h:214-216`（OutOfRangeLow=2，Good=4，dataflash RFND.Stat 与之一致）。
2. 0 读数**不**进 EKF：`libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp:47` 只有 `status()==Good` 的样本存入中值滤波缓冲；OutOfRangeLow 期间缓冲耗尽（3 个新鲜样本、500 ms 窗，:60-68）后不再产生融合数据。
3. 退气压计：`AP_NavEKF3_PosVelFusion.cpp:1209` freshness=500 ms；:1217-1218 `POSZ==RANGEFINDER` 且 fresh 才用测距；:1240-1262 `lostRngHgt` → `fallback_to_baro` → `activeHgtSource=BARO`；气压计测量 = `baro − baroHgtOffset`（:1368），offset 是对当前估计的 LPF（`AP_NavEKF3_Measurements.cpp:809-812`），源切换时 `ResetPositionD(-hgtMea)`（:977-981）→ **估计连续，保持真值**。
4. 有效小读数被融合：`PosVelFusion.cpp:1335` `hgtMea = MAX(rng·cosTilt, rngOnGnd)`，`rngOnGnd = MAX(RNGFND1_GNDCLR, 0.05)`（`Measurements.cpp:29`）→ 读数 0.02 按 0.05 融合，即"贴地高度"；观测噪声 `posDownObsNoise = sq(constrain(EK3_RNG_M_NSE,0.1,10))`（:1347；默认 0.5，`AP_NavEKF3.cpp:286-293`）→ NSE=0.1 时 P[9][9] 被压到 0.01 量级，估计一步吸附到 ≈0.05 m（实测 0.04）。
5. 门限：`PosVelFusion.cpp:936-948` `hgtTestRatio = innov² / ((0.01·EK3_HGT_I_GATE)²·varInnov)`，空中 maxTestRatio=1（即 gate/100 个 σ）：200→2σ、500→5σ；默认 500（`AP_NavEKF3.cpp:220-226`，注释明说"降低更易拒好测量"）。
6. 超时重置：无垂直速度源（`EK3_SRC1_VELZ 0`）→ `hgtRetryTime_ms = hgtRetryTimeMode12_ms = 5000`（`AP_NavEKF3.h:495-496`；`PosVelFusion.cpp:1403-1410`）；`hgtTimeout` → `ResetHeight()`（:963-977），`stateStruct.position.z = -hgtMea`（:287-295）= 直接吸附到当前测距读数，无 STATUSTEXT。
7. 围栏：`FENCE_ALT_MAX 2.5 + FENCE_MARGIN 0.3` 触发线 2.8 m，`Max Alt fence breached` → LAND（三个 FINAL 复现运行均在注入后 ~6.1 s 触发）。

## 结论（对应清单第 2 项的三个问题）

- **0 被当成什么**：`OutOfRangeLow`（RFND.Stat=2），不是 Good；飞控（EKF）**不把 0 融合**。
- **EKF 这时用什么高度**：退到气压计（源切换、估计连续），故障窗内高度估计保持正常——**0 本身不造成塌缩**。
- **FINAL 为什么恢复慢**：真正的故障不是"0"，而是真值 > 1.55 m 后读数变回"有效但只有 0.02–0.08 m"被融合、NSE=0.1 一步吸附；恢复后的正确读数 innov≈1.7 m 被 2σ 门限（200）拒绝，只能等 5 s 高度超时把高度硬置到 3.6–3.7 m（此时已越围栏）。NSE=0.5 + 5σ（500）时 innov 在门限内，1.6 s 收敛。
- **仿真假象**：截 0（`MAX(0,dist)`）是 SITL 后端行为；真实 MTF-01 低于量程下限的输出形式不同（可能输出最小量程值或错误标志）。但**失败的主因不是截 0 窗**，而是"偏移 1.5 m 的传感器在真值 1.55–2 m 区间给出有效小读数"——这一段与传感器型号无关，截 0 假象只影响前 ~0.9 s（该段气压计兜底、无影响）。若真实传感器输出"最小量程值"，坏读数会更早进入融合，机理相同、甚至更快。

## 复现（清单第 1 项）

| 组 | 完成 | 真值最高 m | f_est_err_max m | 触地 m/s | 结局 |
|---|---|---|---|---|---|
| 原门限 500/0.5/1（v4_orig_a_r1..3） | 3/3 | 2.064–2.069 | 1.56–1.58 | 0.50–0.51 | completex3（V2：3/3，dev_base 0.82）|
| FINAL 200/0.1/0（v4_h200n01_a_r1..3） | 0/3 | 3.603–3.688 | 1.57–1.64 | 0.50–0.51 | LAND:fencex3（V2：0/3，真值 3.6–3.7 围栏）|

环境复现与上一轮一致（a 格：爬升 dn15 1 s，航向 30/150/270）。
