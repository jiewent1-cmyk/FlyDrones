# 上机验证清单（GPS + 罗盘 + MTF-01，架构 (i)）

## A. 卸桨可做

1. 刷参数 `gps_flow_recommended.parm`。确认 `SERIAL3_PROTOCOL 5`，重启后 GCS 出现 `GPS 1: detected u-blox`。
2. 室外空旷处静置 3 min：`GPS_RAW_INT` fix_type ≥3，记录卫星数、HDOP、h_acc。**再把飞机放进网笼中心测同样 3 min**：记录 `GPS.Lat/Lng` 的漂移范围（最大偏离、标准差）。这个数决定 GPS 模式在笼内是否可用（判据：最大偏离 <0.5 m 才考虑；SITL 中 σ0.5 m 已使任务 RMS 0.38 m、最大 0.9 m）。
3. 罗盘：确认 `COMPASS_ORIENT`（按 M100 Pro 安装方向，现值 6 是旧罗盘的），做罗盘校准。手持飞机转 4 个方向，GCS 航向与手机指南针相差 <10°。
4. compassmot：装桨前不能做带推力的 compassmot，见 B。卸桨时先看：电机不转、接通/断开大电流负载（如果有）时 `MAG` 读数变化。
5. 源开关：拨三段开关，GCS 依次出现 `Using EKF Source Set 1/2/3`。上电时开关在中位，应出现 `Using EKF Source Set 2`（SITL 已验证上电按开关位置选源组）。
6. 室内（无星）开机：开关中位 + 机载电脑发原点 → `EKF3 IMU0 origin set`，LOITER 可解锁。若机载电脑没发原点 → 应看到 `Arm: Fence requires position`（SITL Q5_in_noorg 3/3）。
7. 室外有星：开关低位（GPS）→ `EKF3 IMU0 is using GPS`；确认机载电脑在 3D fix 时不再发 `SET_GPS_GLOBAL_ORIGIN`（看 `GPS_GLOBAL_ORIGIN` 的经纬度是否等于实际场地）。
8. 光流方向/比例/测距检查沿用 mtf01 报告的起飞前检查。
9. 手持飞机在笼内走 2 m，开关在中位（光流）：`LOCAL_POSITION_NED` 位移 ≈2 m、方向正确；切到低位（GPS）看 EKF 位置是否跳变、跳多少——这就是带桨切换时飞机会实际移动的距离。>1 m 不要在笼内带桨切换。

## B. 必须带桨做（先在 STABILIZE/ALT_HOLD，低高度）

1. compassmot（`COMPASS_MOTCT 2`）：按官方流程做，结果干扰百分比 <30%；之后悬停中把油门从低推到高，`ATT.Yaw` 与 `MAG` 推算航向的差变化 <5°。**超过 10° 就设 `EK3_SRC2_YAW 0`（光流模式不用罗盘）**，依据 Q2：FMmot2（航向随油门变 ~11°）跟踪 0.17 m，是无罗盘 0.06 m 的 3 倍。
2. 光流模式（开关中位）LOITER 1 m 悬停 30 s，记录漂移；然后 2 m。
3. 光流模式 AUTO 2 m 正方形（笼内正式方案）。
4. 只在空旷场地：GPS 模式 LOITER 悬停 60 s，记录漂移。悬停中拨开关 GPS→光流（SITL 跳变 0.1 m，应平稳），再光流→GPS（会被拉到 GPS 位置，SITL 0.5–1.5 m，偏移 3 m 时触发围栏 LAND）。
5. 只在空旷场地：GPS 模式下遮挡/拔 GPS 天线测失效行为（预期：`EK3_SRC_OPTIONS 1` 时靠光流速度继续；SITL IGfa_loss 3/3 完成）。不要在笼内做。
6. 在笼内**不要**用 GPS 模式飞任务，也**不要**在任务中途从光流切到 GPS。
