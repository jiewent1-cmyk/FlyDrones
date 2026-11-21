### 3.5 消息延迟（speedup=1）

- 真值侧：脚本以 20 Hz 请求 `SIM_STATE`，所以真值本身可能已陈旧最多 50 ms；发送循环实测 45.4 Hz（`truth.csv` 间隔中位数 21 ms，p95 27 ms，飞行段内无 >0.3 s 的空洞，唯一的 0.8 s 空洞在 t=5.4 s 的启动阶段，此时还没解锁）。
- 回传侧：脚本以 5 Hz 请求 `DISTANCE_SENSOR`（`val3_fly.py` 的 `streams()`），飞控回包间隔 0.2 s，所以回传延迟的观测分辨率是 0.2 s。
- 实测（`latency.py <run>`，把飞控回包的 DISTANCE_SENSOR 与发送同值的那条记录配对）：

| 运行 | 配对回包数 | min | p10 | 中位数 | max |
| --- | --- | --- | --- | --- | --- |
| `p2_HOV1_mav` | 529 | 0.001 s | 0.001 s | 0.011 s | 1.761 s |
| `p2_SQ1_mav` | 404 | 0.001 s | 0.001 s | 0.011 s | 2.001 s |

max 偏大是因为按"值相同"配对时会把 0.2 s 内重复的同值回包配到更早的那次发送上，不是链路抖动。整体上飞控接收侧延迟在 10 ms 量级，测量被 0.2 s 的回包周期限制，无法分辨更小的延迟。

### 3.6 飞控回包与 EKF 状态

- A 组（MAVLink 路径）dataflash `OF`：`p2_HOV1_mav` 共 3504 条、45.71 Hz、Qual 恒 100；B 组（SITL 专用驱动）1647 条、19.95 Hz、Qual 恒 51（`SIM_FLOW_RND=0.05` 造成）。A 组 OF 条数/频率与脚本实际发送速率一致。
- 飞控确实把 MAVLink 光流用进了 EKF：dataflash STATUSTEXT 里有 `EKF3 IMU0 fusing optical flow`、`EKF3 IMU1 fusing optical flow`、`EKF3 IMU0 started relative aiding`、`EKF3 IMU1 started relative aiding`（`runs/p2_HOV1_mav/logs/*.BIN`）。
- A 组在落地（110.09 s `Disarming motors`）之后才出现 `EKF variance`（112.54 s）、`EKF3 IMU0/IMU1 stopped aiding`（115.65 s），B 组没有这两条。原因：脚本退出后不再发 OPTICAL_FLOW/DISTANCE_SENSOR，0.5 s 后飞控丢掉光流/测距的相对定位输入。**这些文本都在解锁飞行结束之后，不在飞行段内。** 两组都有 `GCS Failsafe`（115.65/115.73 s），原因是脚本关闭连接。

### 3.7 两组差异小结

- 完成情况：12 次运行全部 `mission_complete=True`、解锁成功、飞到目标高度并落地。
- 高度：MAVLink 路径 A 组的 `alt_max` 比 B 组高约 0.09 m（2.163–2.168 vs 2.074–2.075），`hgt_est_rms` 0.058–0.060 m vs 0.020–0.021 m（MAVLink 路径的测距/高度估计误差更大）。
- 水平位置：A 组 `pos_est_err_max` 0.063–0.071 m vs B 组 0.194–0.200 m（悬停任务）；方形任务上 A 组 1.455 m vs B 组 1.519 m，`track_rms` 0.569 vs 0.566。
- 落点误差 `land_offset`：A 组 0.022–0.049 m，B 组 0.033–0.190 m。
- 以上差异没有超出两条路径各自 3 次运行的散布范围之外（同一路径 3 次的数值几乎相同，差异主要来自路径之间的量测方式差异：A 组 `RNGFND1_TYPE=10` 用 MAVLink 测距，B 组 `RNGFND1_TYPE=100` 用 SITL 测距，两者的高度量测噪声模型不同）。
