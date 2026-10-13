# 项目进度 — 2026-10-13

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
| D 分布外测试（6 种模式，快仿真 + Gazebo） | ✅ / 🔄 | 去纹理时 intact 更稳（快仿真 p = 0.001，Gazebo p = 0.028）；Gazebo notex 扩充进行中 | [六](06_connectome_controls_ood.zh.md) |
| E 蘑菇体可塑性（可选） | 🔄 刚开始 | Mac mini 和 Jetson 上已开始初筛；代码尚未合入分支 | — |
| lowbox 结构性改动（不向盲区移动、记忆、侧向 ToF） | ⏸ 暂缓 | H1 原因分析指向这里 | [五](05_hardware_alignment.zh.md) |
| F 安全层 + L0–L4 真机分级 | ⬜ | slew 修复已完成；首飞计划 10-28 | — |

## 各台机器及当前任务

| 机器 | 用途 | 当前（2026-10-13 03:00 PDT） |
|---|---|---|
| ROG 笔记本（Ryzen 9 5900HX，RTX 3070 Laptop） | Gazebo 8.15，3 个锁步实例 | D 扩充：notex lowbox_s20–49 × 17 个配置（511 次） |
| Mac mini M4 | 快仿真，单核最快 | E 阶段初筛（蘑菇体） |
| Jetson AGX Orin | 快仿真补充算力 | E 阶段初筛 |
| 实验室服务器（Xeon 6330，8 × RTX 4090，共享） | 多实例 Gazebo + 仿真时钟 | 本项目暂不占用，仅在必要时使用 |
| 台式机（R5 7500X3D，RTX 5060） | Gazebo 节点，2 个实例 | 关机（机主在用） |

所有机器上快仿真锚点逐位一致（`python -m rl.gates all`，本代码树 12/12 通过）。

## 本分支的数据量

| | 数量 |
|---|---|
| `rl/results/gazebo/hw_runs.csv` 中的硬件对齐 Gazebo 运行 | 4,427 |
| `rl/results/twin/hw/tests.csv.gz` 中的快仿真测试回合 | 18,370 |
| `rl/results/twin/hw/` 中的 CMA-ES 运行 | 20（h2 × 8、c4 × 8、c1 × 4） |

## 论文

目标 ICUAS 2027（全文截止 2027-02-05），按系统论文来写。接下来的节点：G1（11-01）仿真对照，G2（11-22）机载 GUIDED
悬停 + 处理器在环，G3（12-20）真机路线 B。
