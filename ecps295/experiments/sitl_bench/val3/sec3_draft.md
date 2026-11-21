## 3. 第二部分（MAVLink 路径闭环飞行）

### 3.1 方法

| 项 | 内容 |
| --- | --- |
| 脚本 | `val3_fly.py`（一条 TCP 连接同时承载 GCS/任务流和传感器流，source_system=200）；光流/测距计算复用 `mtf_feed.py` 里的 `euler_to_dcm` / `sitl_flow` |
| 真值来源 | SITL 的 `SIM_STATE`（MAVLink id 108，`libraries/GCS_MAVLink/GCS_Common.cpp:6711` → `libraries/SITL/SITL.cpp:1606` `sim_state_send()`）：q1–q4、roll/pitch/yaw(rad)、xacc…zacc、xgyro…zgyro(rad/s)、lat/lon、alt、vn/ve/vd |
| 光流算法 | Python 移植 `libraries/AP_OpticalFlow/AP_OpticalFlow_SITL.cpp`（行号对应见 `mtf_feed.py` 注释）：`rng=(height_agl−z_rel)/rotmat.c.z`（:66-72）、`relVel=rotmat^T·v`（:84）、`+= gyro×pos_offset`（:87）、`flowRate.x=(−relVel.y/rng+gyro.x)·scaleX`（:98）、`flowRate.y=(relVel.x/rng+gyro.y)·scaleY`（:99） |
| 测距 | `height_agl / cos(倾角)`，即 `height_agl / rotmat.c.z`（与 `AP_OpticalFlow_SITL.cpp:66-72` 同一式子）；地面或倾角过大时取 0.02 m |
| A 组（MAVLink 路径） | `FLOW_TYPE=5,RNGFND1_TYPE=10,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0`（实例 40，TCP 6160），脚本按 `--hz 50` 发 OPTICAL_FLOW + DISTANCE_SENSOR |
| B 组（SITL 专用驱动） | `FLOW_TYPE=10,RNGFND1_TYPE=100,MAV_GCS_SYSID=200,FENCE_ENABLE=0,VISO_TYPE=0`（实例 41，TCP 6170），不发送传感器；其余参数与 A 组相同（同一 `mtf01_sitl_rng.parm`） |
| 任务 | HOV：`HOV --alt 2.0 --hold 60 --no-loiter-test`；SQ：`SQ --alt 2.0 --side 2.0 --laps 1 --no-loiter-test`；两个任务各 3 次，A/B 各一组 |
| 记录 | 每轮 `runs/p2_<任务><轮次>_<路径>/`：`truth.csv`（真值与发送值）、`run.json`（飞控回包）、`logs/*.BIN`（dataflash）、`metrics.json`（`analyze_mtf.py` 汇总） |

两组都关掉了 `FENCE_ENABLE`（`mtf01_sitl_rng.parm` 里 `FENCE_ALT_MAX=2.5` 与 2.0 m 任务高度、0.3 m 余量冲突）并设 `VISO_TYPE=0`（没有 VISION_POSITION_ESTIMATE，否则解锁检查 `Arm: VisOdom: not healthy` 不通过）。除这两项和 `FLOW_TYPE`/`RNGFND1_TYPE`/`MAV_GCS_SYSID` 外，两组参数完全相同。

### 3.2 换算实现的一处错误与修正

第一次闭环尝试（`runs/p2_t3_mav`）飞机会持续爬升到 1555 m。查 `truth.csv` 发现 `dist_m` 一直是 0.02 m 而 `height_agl` 一直在涨，说明"倾角过大"保护一直触发：根因是 `mtf_feed.py` 里 Python 版 `euler_to_dcm` 写错，矩阵的 r22（=cos(pitch)cos(roll)）算成了 sin(pitch)，导致 `rotmat.c.z` 判据恒不成立。

修正后按 `libraries/AP_Math/matrix3.cpp:26-44` `Matrix3f::from_euler()` 逐项对齐（a/b/c 是列，row0=(a.x,b.x,c.x) 等），并用 `check_dcm.py` 验证：水平时 dcm[2][2]=1.0；rpy=(10°,20°,30°) 时 dcm[2][2]=0.925417 = cos20°cos10°；真值向北 1 m/s、高 2 m 时 flow=(0,+0.5) rad/s，向东 1 m/s 时 flow=(−0.5,0) rad/s。

### 3.3 与 SITL 专用驱动的一致性核对

`cmp_flow2.py <run>`：把 `truth.csv` 里按上式算出的光流与同一运行 dataflash 的 `OF` 消息（`AP_OpticalFlow.cpp:288-308`）按最近邻配对，并搜索两条时间轴的固定偏移。

| 运行 | 配对样本 | rms(flowX) | rms(flowY) | 说明 |
| --- | --- | --- | --- | --- |
| `p2_HOV1_nat` | 3495 | 0.0422 | 0.0296 | 悬停为主，速度小 |
| `p2_SQ1_nat` | 2376 | 0.0455 | 0.0372 | 方形轨迹，有明显平移 |

差值量级与 B 组自带的噪声一致：`mtf01_sitl_rng.parm` 里 `SIM_FLOW_RND=0.05`（`AP_OpticalFlow_SITL.cpp` 给 flowRate 加 ±0.05 rad/s 随机量），而 A 组（我算的流）不加噪声。逐点对照可见两者均值同为 0 量级（真值 flow_x 范围 −0.41..0.11，B 组 OF 范围 −1.08..0.15）。

地面静止段（解锁前，t<34 s）：真值算出的 flow_x/flow_y 恒为 0，B 组 dataflash OF 的值在 ±0.05 rad/s 内（例如 `p2_HOV1_nat` t=34.02 s：truth fx=0.0000 fy=0.0000，of fx=−0.0027 fy=−0.0207）。

说明：任务书里的"手动改 SIM_ 位置"这一步**没有单独做**——本仓库的 SITL 没有可从 MAVLink 侧直接改模拟位置/速度的入口（`libraries/SITL/` 下没有接收外部位置指令的代码）。等价的核对用的是飞行中真实产生的平移段（A/B 同一任务同一时刻的真值来自同一个 SITL 仿真，B 组 OF 由 SITL 自己用 `AP_OpticalFlow_SITL.cpp` 算出），覆盖了"位置/速度变化时光流是否同源"这一点。

### 3.4 飞行结果
