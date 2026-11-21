# MTF-01 MAVLink 传感器接入仿真验证报告（任务 V3）

工作目录：`/home/jacky/sim/val3`　源码：`/home/jacky/sim/ardupilot`（ArduCopter 4.7.0 SITL）
实例号：仅 40（TCP 6160）和 41（TCP 6170）。所有测量都能在 `runs/<tag>/` 里找到对应产物。

## 0. 方法与产物

| 项 | 内容 |
| --- | --- |
| SITL 启动 | `arducopter --model + --speedup 1 -I<inst> --defaults copter.parm,mtf01_sitl_rng.parm,<overlay>.parm --home 33.6430,-117.8420,20,30`（`run_feed.sh`、`run_fly.sh`） |
| 参数集 | `mtf01_sitl_rng.parm`（本目录，157 行）+ 每次运行的 `overlay.parm` |
| 覆盖（第一部分） | `FLOW_TYPE=5`（MAVLink 光流后端）、`RNGFND1_TYPE=10`（MAVLink 测距后端） |
| 发送脚本 | `mtf_feed.py`（sysid 200，50 Hz 发 OPTICAL_FLOW / DISTANCE_SENSOR） |
| 解析脚本 | `analyze_feed.py`（分相统计、超时、health 位）、`analyze_of.py`（dataflash OF）、`analyze_rate.py`（频率） |
| 原始记录 | `runs/<tag>/feed.json`（收发逐条记录）、`analysis.json`、`of.json`、`logs/*.BIN`（dataflash） |

数据流方向：脚本 → SITL 实例的 TCP 端口（MAVLink）→ ArduCopter → 回传 DISTANCE_SENSOR / OPTICAL_FLOW / SYS_STATUS / STATUSTEXT；dataflash 从 `logs/*.BIN` 解析。

## 1. 总表

| # | 检查项 | 预期 | 实测 | 运行目录 | 结论 |
| --- | --- | --- | --- | --- | --- |
| 1 | OPTICAL_FLOW 取哪些字段 | 优先 `flow_rate_x/y`（rad/s） | FC 回报 `flow_rate_x/y` = 0.5 / −0.25（发送值） | `p1_main200` | 符合 |
| 2 | 旧字段 `flow_x/flow_y`（int16, dpi） | 不参与换算 | FC 回报 0 / 0（本次发送 0） | `p1_main200` | 符合 |
| 3 | 光流单位换算 | rad/s 原样 | 0.5 → 0.5，−0.25 → −0.25 | `p1_main200` | 符合 |
| 4 | quality | 原样透传 | 100→100，0→0，100→100 | `p1_main200` | 符合 |
| 5 | 光流超时 | 0.5 s（`OPTFLOW_MAV_TIMEOUT_SEC`） | 停发后 0.502 s 内不再更新 | `p1_main200` | 符合 |
| 6 | 距离透传 | cm→m 除 100 | 0.5/1.0/2.0/7.9/9.0 m → 50/100/200/789/900 cm | `p1_main200` | 符合 |
| 7 | orientation | 须等于 `RNGFND1_ORIENT`=25 | FC 回报 orientation=25 | `p1_main200` | 符合 |
| 8 | 量程来源 | 参数与消息取小/取大 | FC 回报 min/max = [1, 800] cm | `p1_main200` | 符合 |
| 9 | 超量程状态 | 内部 `OutOfRangeHigh` | 9.0 m（> `RNGFND1_MAX`=8 m）仍回报 900 cm；本次全程未收到 `RANGEFINDER` 消息（0 条），内部状态无法从 MAVLink 观察 | `p1_main200` | 不可观察 |
| 10 | 测距超时 | 500 ms | 停发后 0.498 s | `p1_main200` | 符合 |
| 11 | SYS_STATUS 光流健康位 | 500 ms 未更新即不健康 | 最后置位 30.447 s，首个清除 31.946 s（SYS_STATUS 约 2 Hz） | `p1_main200` | 符合 |
| 12 | SYS_STATUS 测距位 | 不存在该位 | 源码中 SYS_STATUS 无测距位 | — | 符合（源码） |
| 13 | 发送频率 10/20/50/100 Hz | FC 收到同频更新 | dataflash OF：10.02 / 20.03 / 50.03 / 100.00 Hz | `p1_hz10`…`p1_hz100` | 符合 |
| 14 | sysid 200 vs 1 | 无差异 | 各相数值完全一致 | `p1_main200` vs `p1_main1` | 符合 |
| 15 | dataflash `OF` 字段 | flowX/flowY/bodyX/bodyY | flowX 0.49983，flowY −0.249915，bodyX 0.001483，bodyY −0.000551（qual 100） | `p1_arm_of` | 符合 |
| 16 | 第二部分闭环飞行 | MAVLink 路径与 SITL 专用驱动对比 | 见第 3 节 | `p2_*` | 部分完成 |

## 2. 逐项说明（源码引用 + 实测）

### 2.1 OPTICAL_FLOW 后端读了哪些字段、单位、换算

- `libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:23`：`#define OPTFLOW_MAV_TIMEOUT_SEC 0.5f`。
- `AP_OpticalFlow_MAV.cpp:107` 每次收到消息记 `latest_frame_us = AP_HAL::micros64()`。
- `AP_OpticalFlow_MAV.cpp:110-115`：只要 `flow_rate_x` 或 `flow_rate_y` 非零就切到 rad/s 模式（`flow_sum_is_rads = true`）。
- `AP_OpticalFlow_MAV.cpp:118-121`：rad/s 模式下把 `flow_rate_x/y` 累加进 `flow_sum`；`:122-125` 才是旧的 `flow_x/flow_y`（int16，单位为像素/采样间隔即 dpi）累加分支。
- `AP_OpticalFlow_MAV.cpp:127`：`quality_sum += packet.quality`；`:52`：`state.surface_quality = quality_sum / count`，所以 quality 是多次平均后透传。
- `AP_OpticalFlow_MAV.cpp:66-68`：`dt_used = flow_sum_is_rads ? 1.0f : dt`，`state.flowRate = (flow_sum/count) * flow_scale_factor * dt_used`。rad/s 模式下 `dt_used=1`，所以**flow_rate_x/y 是平均值直接使用，不乘 dt**；旧字段模式下才乘 dt 变成 rad/s。
- `AP_OpticalFlow_MAV.cpp:61-62`：`flow_scale_factor = 1 + 0.001*FLOW_FXSCALER`（`FLOW_FXSCALER` 默认 0 → 系数 1.0，见 `AP_OpticalFlow.cpp:41`）。
- `AP_OpticalFlow_MAV.cpp:55-59`：`dt = latest_frame_us - prev_frame_us`，`dt<=0` 或 `dt>=0.5 s` 时本次光流量置零，即**两次消息间隔必须小于 0.5 s**；本任务 10 Hz 起都满足。
- 对 compid / sensor id：后端不校验 compid（`handle_msg()` 只取包里的 `sensor_id`，`AP_OpticalFlow_MAV.cpp:131`），同一 sysid 下多个实例互不区分（本任务只发一个实例，id=1）。

实测（`runs/p1_main200/analysis.json`）：发送 `flow_rate_x=0.5, flow_rate_y=-0.25`，FC 回包 `flow_rate_x=0.5, flow_rate_y=-0.25`（中位数），`flow_x/flow_y`（旧字段）= 0/0；quality 100→100、0→0、100→100。

### 2.2 dataflash `OF` 消息

- `libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:283,288-308`：每次光流更新写一条 `OF`，字段为 `time_us, surface_quality, flow_x, flow_y, body_x, body_y`（即 TimeUS / Qual / flowX / flowY / bodyX / bodyY）。
- 因此 **dataflash OF 的条数与频率 = 飞控实际接受到的光流更新率**。

实测（`runs/p1_hz10`、`p1_hz20`、`p1_hz50`、`p1_hz100` 的 `of.json`）：

| 发送频率 | dataflash OF 条数 | OF 频率 | Qual | flowX 均值 | flowY 均值 | bodyX 均值 | bodyY 均值 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 10 Hz | 239 | 10.02 Hz | 100 | 0.497908 | −0.248954 | 0.007975 | −0.003840 |
| 20 Hz | 477 | 20.03 Hz | 100 | 0.498952 | −0.249476 | 0.006984 | −0.003345 |
| 50 Hz | 1193 | 50.03 Hz | 100 | 0.499581 | −0.249790 | 0.007491 | −0.003595 |
| 100 Hz | 2381 | 100.00 Hz | 100 | 0.499790 | −0.249895 | 0.001916 | −0.000812 |

`runs/p1_arm_of`（地面解锁、flow_rate 0.5/−0.25、50 Hz）：OF 2941 条 / 89.51 Hz，Qual 100，flowX 均值 0.49983，flowY −0.249915，bodyX 0.001483，bodyY −0.000551。`runs/p1_arm_of_q0`（quality=0）：OF 2919 条 / 88.92 Hz，Qual 0，flowX 0.499829。

### 2.3 DISTANCE_SENSOR 后端

- `libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp:30-31`：**只有 `packet.orientation == orientation()`（即 `RNGFND1_ORIENT`，本例 25）时才接受**，否则整条丢弃。
- `:33`：`distance = packet.current_distance * 0.01`（消息是 cm，飞控内部用 m）。
- `:34-35`：`_max_distance = packet.max_distance * 0.01`，`_min_distance = packet.min_distance * 0.01`。
- `:48-59`：`max_distance() = MIN(参数 RNGFND1_MAX, 消息 max_distance)`；`:61-72`：`min_distance() = MAX(参数 RNGFND1_MIN, 消息 min_distance)`。**量程用消息与参数的较小/较大值。**
- `:37-43`：`signal_quality` 0 → UNKNOWN，1 → MIN，其余原样。
- `:82-88`：超过 `AP_RANGEFINDER_MAVLINK_TIMEOUT_MS`（`AP_RangeFinder_MAVLink.h:11` = 500 ms）置 `Status::NoData`。
- `libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp:63-68`：距离 > max → `OutOfRangeHigh`，< min → `OutOfRangeLow`，否则 `Good`。
- 飞控侧回发：`libraries/GCS_MAVLink/GCS_Common.cpp:449-479` `send_distance_sensor()`，用 `distance()*100` 还原成 cm。

实测（`runs/p1_main200`）：发送 0.5 / 1.0 / 2.0 / 7.9 / 9.0 m，FC 回包 50 / 100 / 200 / 789 / 900 cm，orientation 25，signal_quality 100，min/max [1, 800] cm。**9.0 m 超过 `RNGFND1_MAX`=8 m 时，飞控仍原样回报 900 cm**（与"原样透传"一致）；但本次运行收到的 `RANGEFINDER` 消息数为 0，所以内部 `OutOfRangeHigh` 状态**无法从 MAVLink 直接观察**（该状态下飞控不发 RANGEFINDER 消息，与 `AP_RangeFinder_Backend.cpp:54-57` `has_data()` 的判断一致）。这是本次唯一没能在 MAVLink 上直接读到的量。

### 2.4 超时与健康位

- 测距：停发 DISTANCE_SENSOR 于 t=34.0 s，FC 最后一帧回报 34.498 s（静默 0.498 s）；随后飞控把该测距后端置为 NoData。
- 光流：停发 OPTICAL_FLOW 于 t=30.0 s，FC 最后一帧回报 30.502 s（静默 0.502 s），与 `AP_OpticalFlow_MAV.cpp:23,59` 的 0.5 s 超时一致。
- 前端健康判定 `libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:196-197`：`_flags.healthy = (millis() - _last_update_ms < 500)`。
- `libraries/GCS_MAVLink/GCS.cpp:581-590`：`SYS_STATUS` 里光流位 present/enabled 恒为真，health 位取 `healthy()`。
- 实测（`p1_main200`）：health 位共置位 35 次，首次 1.446 s、最后一次 30.447 s，**首个清除 31.946 s**（SYS_STATUS 约 2 Hz，500 ms 窗口落在两次上报之间）；`p1_main1`（sysid 1）：置位 39 次，1.951 s / 30.449 s / 首个清除 31.954 s。
- **SYS_STATUS 没有测距位**（`GCS.cpp` 中检索不到 rangefinder 相关位），所以"停发测距后 SYS_STATUS 的测距位"这一项无从观测，报告中记为"该位不存在"。

### 2.5 发送方 sysid = 1 会不会被丢弃

- `libraries/GCS_MAVLink/GCS_Common.cpp:7178-7201` `accept_packet()`：与飞控自身 sysid 相同（7181）、是 GCS sysid（7187）、是 RADIO id（7191）都接受；**未启用 `GCS_SYSID_ENFORCE` 时（7196-7198）其它 sysid 也接受**。本仓库 `GCS_SYSID_ENFORCE` 默认关闭。
- 因此 ArduPilot 文档里"把 MTF-01 的 mav_id 改成非 1"的要求在本仿真里不是硬性条件。

实测对比（`runs/p1_main1`（sysid 1，实例 41）vs `runs/p1_main200`（sysid 200，实例 40））：距离各相 50/100/200/789/900/200/200 cm 完全一致，光流 0.5/−0.25 一致，quality 相位一致；差别只有 BARO 标定文本（1 个 vs 2 条）和 SYS_STATUS 采样数（50 vs 48）。静默窗口 0.485/0.484 s（sysid 1）与 0.498/0.502 s（sysid 200），差异在 0.5 Hz 的上报间隔量级。

### 2.6 频率要求

`AP_OpticalFlow_MAV.cpp:59` 只要求相邻两帧间隔 `0 < dt < 0.5 s`；`AP_RangeFinder_MAVLink.cpp:82` 要求 500 ms 内有 DISTANCE_SENSOR。因此 10 Hz 及以上都满足；实测 10/20/50/100 Hz 的 dataflash OF 频率分别为 10.02/20.03/50.03/100.00 Hz，无丢失。

## 3. 第二部分（MAVLink 路径闭环飞行）

（待补：见第 3 节正文）

## 4. 失败与可疑的运行

| 运行目录 | 现象 | 查到的原因 |
| --- | --- | --- |
| `p1_dist1` | 发出 750 条 DISTANCE_SENSOR，FC 回报 0 条 | 该次运行（00:17）用的是 `mtf_feed.py` 的早期版本（该文件最后一次修改 00:35），当时没有向 FC 请求 DISTANCE_SENSOR/OPTICAL_FLOW 数据流，FC 不回传，无法观测 FC 侧数值 |
| `p1_dist` | `--dist-schedule` 给了 5 段距离，实际发出 0 条 | 同上：早期版本未实现 schedule，参数被接受但未生效 |
| `p1_flow` | 发出 800 条 OPTICAL_FLOW，FC 回报 0 条 | 同上（无数据流请求） |
| `p1_sys200` | 发出 1000+1000 条，FC 回报 0 条 | 同上（无数据流请求） |
| `p1_sys254` | 同上 | 同上（无数据流请求） |
| `diag_rx` | 发出 0 条 | 早期版本未实现 schedule |
| `p1_arm_of1` | feeder 退出码 1，无 dataflash | 脚本异常：`TypeError: MAVLink.rc_channels_override_send() missing 2 required positional arguments: 'chan7_raw' and 'chan8_raw'`（本机 pymavlink 版本需要 10 个原始通道参数）。00:35 修复后 `p1_arm_of` 成功 |
| `opt2probe` | 只有 sitl.log/overlay，无 cmd.txt、无 feeder 输出 | 探测实例号 40 端口是否可连接的临时目录，未做测量，无结果 |
| `p1_main200` / `p1_main1` | STATUSTEXT 里有 `No ap_message for mavlink id (153)` | 脚本请求了 msgid 153，而本方言里 SIM_STATE 是 108；该文本不影响本组测量，但说明当时订阅的是不存在的 id |

可疑但未判为失败：

- `p1_arm_of`、`p1_arm_of_q0`：解锁过程中反复出现 `Arm: Accels inconsistent`、`Arm: EKF attitude is bad`、`Arm: VisOdom: not healthy`、`Arm: Fence requires position`、`Arm: Gyros inconsistent`，重试后成功解锁并写日志。这些是地面解锁时的预检提示，OF 数据本身可用（OF 2941/2919 条）。
- 超量程（9.0 m）时飞控仍回报 900 cm：数值本身是"原样透传"，但内部状态（OutOfRangeHigh）在 MAVLink 上观测不到（见 2.3）。

## 5. 没做的

（待补）
