# 项目进度 — 2026-10-14（23:30 PDT）

> `ecps295-sim` 分支上仿真与 RL 工作在全部实验机器上的进度快照。
> English: [00_status.en.md](00_status.en.md) · 报告索引：[README.md](README.md)

## 各阶段

| 阶段 | 状态 | 一句话结论 | 报告 |
|---|---|---|---|
| S4–S8 数字孪生、MiniFly v0 → v4、无 GPS 导航、消融 | ✅ | v4 + 路线 B 可无 GPS 飞行；LCb、扫视、路线 B、传出副本都是关键组件 | [一](01_twin_and_minifly.zh.md) [二](02_gps_free_navigation.zh.md) [三](03_s8_ablation.zh.md) |
| A0–A4、B1–B3：GPS 网笼中的 CMA-ES | ✅ | es3 在 GPS 网笼里更好，但在真机传感器配置下不成立 | [四](04_cmaes_optimisation.zh.md) |
| H0–H3 硬件对齐 | ✅ | lowbox 接触与解码器无关；h2_s3 作为保守版通过 | [五](05_hardware_alignment.zh.md) |
| C1 传出副本 | ✅ | v4_eff 把空场误转向减半，成为默认版本 | [五](05_hardware_alignment.zh.md) |
| 光流导航下的 C0 / C2 | ✅ | 大脑必需（去掉后接触增加约 10–40 倍）；真实接线优于 5/5 个打乱接线，但较弱 | [六](06_connectome_controls_ood.zh.md) |
| C4 bypass 对 intact（8 + 8 个种子） | ✅ | bypass 更好（Gazebo p = 0.015，快仿真 p = 0.010）；读出层不是瓶颈 | [六](06_connectome_controls_ood.zh.md) |
| D 分布外测试（6 种模式，快仿真 + Gazebo） | ✅ | 去纹理时 intact 更稳（快仿真 p = 0.001；Gazebo 50 个世界 p = 0.001） | [六](06_connectome_controls_ood.zh.md) |
| E 蘑菇体可塑性 | ✅ 阴性 | 6 种设置、48 次生命：学到的权重不比打乱的好 | [八](08_mushroom_body.zh.md) |
| lowbox 结构性改动 S1–S3 | ✅ | 不后退：无效；空间记忆：不显著；侧后测距：有接触的运行 −38% | [七](07_range_sensing_shield.zh.md) |
| N 路线评审：测距安全层 S4、测距接进大脑 S5 | ✅ 快仿真 + Gazebo | 5 颗 ToF + 安全层：lowbox 快仿真 0/150、Gazebo 0/50（27° 圆锥）；s5 为 1/50，安全层介入少 42% | [七](07_range_sensing_shield.zh.md) |
| N1 扩大样本（第二个大脑种子） | ✅ | s5 复现：两组种子合计 lowbox 19/100 → 2/100（p = 0.0001），cage20 6/16 → 0/16 | [七](07_range_sensing_shield.zh.md) |
| S6/S7 更便宜的传感器构型、8×8 ToF + 局部 3D 地图 | ✅ Gazebo / 🔄 复验 | **前后两块 VL53L7CX + 地图（w2map）：lowbox 0/50、cage20 0/8**，唯一两种场景都 0 接触的构型；第二个大脑种子复验中 | [七](07_range_sensing_shield.zh.md) 第 8 节 |
| **硬件定型** | ✅ 10-14 22:20 | **前后各一块 VL53L7CX 8×8 + 局部 3D 地图 + 安全层**，跑在 Orange Pi 上 | [七](07_range_sensing_shield.zh.md) 第 8 节 |
| N3 安全层在环的 CMA | 🔄 第二轮 | 第一轮（5 颗单区、加权适应度）要么牺牲安全、要么牺牲覆盖率，不采用；第二轮在 w2map 上，适应度 v6（覆盖率为目标、五项安全代理为硬约束），5 个种子 | [七](07_range_sensing_shield.zh.md) 第 8 节 |
| N4 压力测试、N5 预注册验收、N6 ToF 台架与真机 | ⬜ | 按 w2map 重新设计（EKF 漂移注入、高度波动、提速、传感器退化）；N6 等 VL53L7CX 到货 | — |
| F 安全层 + L0–L4 真机分级 | ⬜ | slew 修复已完成；首飞计划 10-28 | — |

## 各台机器及当前任务

| 机器 | 用途 | 当前（2026-10-14 23:30 PDT） |
|---|---|---|
| ROG 笔记本（Ryzen 9 5900HX，RTX 3070 Laptop） | Gazebo 8.15，3 个锁步实例 | w2map、w1map 第二个大脑种子复验（各 58 次），之后 w2map 巡航 0.65 m/s 压力测试；01:00–07:00 自动暂停 |
| Mac mini M4 | 快仿真，单核最快 | N3 第二轮（w2map），CMA 种子 1、2、4（每个约 2.5 h） |
| Jetson AGX Orin | 快仿真补充算力 | N3 第二轮，CMA 种子 3、5（每个约 8 h） |
| 实验室服务器（Xeon 6330，8 × RTX 4090，共享） | 多实例 Gazebo + 仿真时钟 | 本项目暂不占用，仅在必要时使用 |
| 台式机（R5 7500X3D，RTX 5060） | Gazebo 节点，2 个实例 | 关机（机主在用） |

所有机器上快仿真锚点逐位一致（`python -m rl.gates all`，本代码树 12/12 通过）。ROG 和台式机在洛杉矶时间 01:00–07:00
不跑任何任务，运行脚本会自动暂停。

## 本分支的数据量

| | 数量 |
|---|---|
| `rl/results/gazebo/hw_runs.csv` 中的硬件对齐 Gazebo 运行 | 5,357 |
| `rl/results/twin/hw/tests.csv.gz` 中的快仿真测试回合（对照臂、种子、分布外） | 18,370 |
| `rl/results/twin/n/tests.csv.gz` 中的快仿真回合（S1–S5 筛选、接触诊断） | 6,126 |
| `rl/results/twin/mb/tests.csv.gz` 中的快仿真回合（E 阶段） | 10,944 |
| `rl/results/twin/hw/` 中的 CMA-ES 运行 | 21（h2 × 8、c4 × 8、c1 × 4、n3 × 1） |

## 最终硬件（10-14 定型）

| 部件 | 接到 | 作用 |
|---|---|---|
| Matek 3901-L0X（光流 + 下视 ToF） | 飞控 UART4（GPS 只保留 I²C 罗盘） | 无 GPS 时 EKF 的速度和高度来源 |
| Orange Pi Zero 3W | 飞控 UART3（MAVLink2），Vx 供电 | 大脑、局部 3D 地图、安全层 |
| 前后各一块 VL53L7CX 8×8 ToF（60° × 60°） | Orange Pi I²C TWI0 / TWI1，3.3 V | 避障测距 → 地图 → 安全层；前板读数同时进大脑的 LCn |
| USB 摄像头：ELP OV7725 120° 或 Waveshare OV9281（全局快门） | Orange Pi USB-C | 大脑的视觉输入；台架对比后二选一 |
| 3D 打印件（光流底座、Pi 罩 + 电池平台、GPS 支架） | 机架 | 整机估计 300–350 g |

上飞前要先做两块 VL53L7CX 的台架：笼网和黑箱检出率、箱顶比传感器低 1–15 cm 时的读数、60° 视场边缘的区会不会扫到桨、
Orange Pi 上每帧建图的耗时。

## 待决定的事

- **论文定位：** 从"连接组控制器避障"改为"连接组控制器 + 测距安全层，安全层在环优化"；连接组的问题仍由 C0/C2/C4 和
  分布外实验回答。

## 论文

目标 ICUAS 2027（全文截止 2027-02-05），按系统论文来写。接下来的节点：G1（11-01）仿真对照，G2（11-22）机载 GUIDED
悬停 + 处理器在环，G3（12-20）真机路线 B。
