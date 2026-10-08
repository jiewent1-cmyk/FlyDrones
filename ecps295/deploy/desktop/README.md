# 台式机部署（Ubuntu 24.04 + NVIDIA）

目标机器：R5 7500X3D、RTX 5060、32 GB 内存，Ubuntu 24.04 装在外置 NVMe 上。
部署完成后，台式机的运行环境与 ROG、服务器一致：

| 组件 | 版本 |
|---|---|
| Gazebo Harmonic | OSRF apt 源；ROG 为 8.15.0，服务器为 8.10 |
| ArduPilot | Copter-4.7.0，加 `sitl/ardupilot_sitl.patch` 中的两处补丁（JSON 测距位掩码、障碍物上方的光流换算） |
| ardupilot_gazebo | `082a0fe` |
| conda 环境 | `flydrones` 和 `ardupilot`，都是 Python 3.11，版本锁定见 `requirements_*.txt`；numpy 2.4.6 和 scipy 1.17.1 必须一致，快仿真才能逐位复现 |

代码目录：

| 路径 | 内容 |
|---|---|
| `~/sim/FlyDrones` | `ecps295-sim` 分支 |
| `~/sim/hw/{ecps295,src}` | hw 运行树，与服务器 `gz_batch.py --hw` 使用的目录结构相同 |
| `~/sim/` | 启动脚本：`gz_g4.sh`、`run_matrix*.sh`、`run_ablation.sh`、`run_resume.sh`；`ECPS_TREE` 指定运行树，默认是 hw 树 |

## 用法

```bash
# 1. 把本目录拷到台式机（或在台式机上 git clone 分支后进入 ecps295/deploy/desktop）
bash setup_desktop.sh            # 首次安装驱动后需要重启，重启后再运行一次，会从断点继续
bash ~/sim/verify_desktop.sh     # 验收，约 15 分钟，运行时机器上不要有其他任务
```

`setup_desktop.sh` 可以重复运行，已完成的阶段会自动跳过，完成标记在 `~/sim/.deploy/`。常用选项：
- 只跑指定阶段：`bash setup_desktop.sh apt conda`
- 强制重做已完成的阶段：加 `FORCE=1`
- 指定编译并行数：`JOBS=8`

各阶段按顺序如下：

| 阶段 | 内容 | 需要 sudo |
|---|---|---|
| check | 系统版本、磁盘（≥ 30 GB）、网络 | |
| driver | 如果 `nvidia-smi` 不可用，执行 `ubuntu-drivers install`。RTX 50 系需要 570 以上的 open 驱动；装完需要重启，开了 Secure Boot 的话还要登记 MOK 密钥 | 是 |
| apt | Gazebo Harmonic、ardupilot_gazebo 的编译依赖、`cam_bridge.py` 用到的系统 Python gz 绑定 | 是 |
| conda | 把 Miniforge 装到 `~/miniconda3`（路径与 ROG 相同，启动脚本都写死了这个路径），再建两个环境 | |
| flydrones | 克隆分支，以可编辑方式安装，并生成 MiniFly 的 v2、v3、v4 权重 | |
| ardupilot | 拉取 Copter-4.7.0，打补丁，编译 SITL | |
| gzplugin | 编译 ardupilot_gazebo 插件 | |
| hw | 生成 hw 树。默认用分支代码，再打上 `hw_run_g4.patch`（服务器 `run_g4.py` 的仿真时钟和对照臂改动，都由环境变量开启，默认不生效）。设置 `HW_TAR=...` 时，改用 `pack_hw_tree.sh` 从服务器打包的树。旧树保留为 `~/sim/hw.old` | |
| launchers | 把 ROG 的启动脚本复制到 `~/sim`，并改成通过 `ECPS_TREE` 选择运行树 | |

## 验收（`verify_desktop.sh`）

1. **版本和权重**：记录各组件版本；三个 MiniFly 权重的 SHA-256 必须与 `reference.json` 一致。
2. **`python -m rl.gates all`**：感觉门禁、语义冒烟测试、锚点四项都必须 PASS。锚点通过，说明快仿真与服务器、Mac mini 逐位一致。
3. **Gazebo 闭环**：在 lowbox_s0 上运行 v4（光流 + 路线 B，120 s 巡航，种子 0）。先单实例跑一次，再分别用 3 个和 4 个实例并行（`NS="1 3 4"`，所有实例飞同一回合）。
   - 输出每个实例的整段 RTF，以及撞击、险情、航程等结果，旁边列出 ROG 和服务器同一回合的结果作为对照。
   - 单个回合本身波动很大：ROG 上航程 21.8–25.8 m，服务器上是 12.8 m 加 1 次撞击。所以"接近"的判断标准是：没有飞控降落、没有超过 1.3 m 高度、没有卡住不动，各项数值落在参考值附近。
   - RTF 判定沿用 ROG 的规则：整段 RTF ≥ 0.95 才算有效。3 个和 4 个实例下的中位数，决定这台机器以后用几路并行。

结果写在 `~/sim/runs/desktop_verify/`，包括 `verify.log` 和 `summary.json`。

## 已验证的部分（2026-10-08，在 Mac 上）

- 分支代码（`25a9753`）在全新的锁定版本环境中运行 `rl.gates all`，12 项全部通过，4 个锚点与仓库中的 `anchors.json` 一致。
- 生成的 `minifly_v3/v4.npz` 哈希与 ROG、服务器一致。
- 启动脚本的改写已用 GNU sed 测试过，验收汇总脚本已用模拟数据跑通。
- 还没有在真实的台式机上运行过：apt、驱动、ArduPilot 编译和 Gazebo 闭环部分，要等台式机装好 Ubuntu 之后才能验证。

## 注意

- 服务器的 hw 树现在还在 H 阶段，正被就地修改：10-08 时代码比 `anchors.json` 新，锚点门禁对不上。所以默认从分支生成 hw 树；等 H 阶段的负责人确认这棵树已经冻结（或者已提交），再用 `pack_hw_tree.sh` 和 `HW_TAR` 同步。
- 内核更新以后要先确认 `nvidia-smi` 正常。ROG 停电重启后进了没有 NVIDIA 模块的新内核，Gazebo 退化成软件渲染，RTF 不达标。
- 跑 Gazebo 批量实验时，不要在同一台机器上同时跑快仿真或 CMA-ES。即使用 `nice 19`，也会把 RTF 拉到 0.95 以下。
