# ecps295/rl：用 CMA-ES 优化 MiniFly 解码器，以及配套的快仿真、对照实验与 Gazebo 多实例评估

> 状态：2026-10-07。A0、A2、A3、B1 已完成；A1/B2 的 Gazebo 复测与 A4/B 适应度 v2（3 个种子）进行中（见文末"待办"）。
> 路线与依据见 `docs/`：`RL_plan_20261004_zh.md`（初版计划）、`community_RL_survey_and_roadmap_20261006_zh.md`（社区项目调研与定制路线 A–F）。

本目录只新增文件，不修改 `ecps295/` 里已有的模块。在 GPS 模式下，快仿真用本仓库当前的 `ecps_pilot / ecps_decoder / ecps_retina` 跑出的结果，与开发时使用的 10-04 快照逐位一致（已验证）。

## 1. 内容

| 路径 | 作用 |
|---|---|
| `twin.py` | 快仿真：NED 坐标、与 Gazebo 对齐的鱼眼相机（hfov 120°、下俯 12°、伽马、胶带、地垫胶带条、网线纹理、云层、吊灯），按轴分别处理的滑动碰撞 |
| `episode.py` | 单个巡航回合，计分口径与 `run_g4.py` 一致；附带触发器指标（`trigger_metrics.py`） |
| `worlds.py` | 程序化随机笼子（cage20 不参与训练），45% 带贴围栏边的箱垛；固定的 24 回合留出集 |
| `params.py` | 17 个解码器参数（`default` / `wide` 两套边界），以及 bypass 臂的 8 个额外增益 |
| `cmaes_run.py` | CMA-ES 主循环：公共随机数、每代对照 v3_2、每 5 代评估留出集、按留出集选最优；参数包括 `--arm`、`--w-cov`、`--x0-yaml` |
| `arms.py` | 对照臂：`brainoff`、`shuffle<k>`（Maslov–Sneppen 换边，精确保持出入度和权重符号）、`randread<k>`、`bypass`（绕过 LIF）、`black`、`frozen`、`noloom` |
| `batch.py`、`validate.py`、`compare.py`、`export_arms.py` | 并行批量运行、cage20 留出评估与统计、快仿真与 Gazebo 对照、导出最优配置 |
| `trigger_metrics.py` | 触发器指标（2026-10-06 预注册）：威胁回合、扫视召回率与潜伏期、每分钟虚警、链式触发率、慢速比例 |
| `stats.py` | A2 配对统计：精确 McNemar（救回/新增）、配对差值 bootstrap 95% CI + Wilcoxon、按指标 Holm 校正、打乱实例分层 bootstrap 与位次 |
| `gates.py` + `anchors.json` | A3：开环感觉门控、闭环语义冒烟测试、快仿真结果 SHA-256 锚点（本机 / Jetson 逐位一致）。`python -m rl.gates all` 失败时退出码为 1 |
| `b1_empty.py` | B1 空场扫视率（只有网笼的世界，每次扫视都是虚警） |
| `server/` | 云服务器上 Gazebo 多实例并行用的脚本：`gz_inst.sh`（端口 +10·I、GZ_PARTITION、每个实例单独复制模型、共享内存路径）、`gz_batch.py`、`simclock.py`、`gz_table.py`，以及 NUMA 绑核 A/B 和各个排队脚本 |
| `configs/` | es2、es3、A1 各臂、B2 各点的最优配置，以及 `v4_oc5`（v4 的居中视野改回 5 列）。网络权重指向 `../../minifly/*.npz`，用 `python my_minifly.py v3/v4` 生成 |
| `results/twin/` | 每次 CMA-ES 运行的记录：`gens.jsonl`、`val.jsonl`、`best_val.json`、`config.json`、`episodes.jsonl.gz`、`cmaes.log`；`cage20_holdout/` 是 slew 修复前后的快仿真 64 种子对照 |
| `results/gazebo/gazebo_runs.csv` | 服务器上每一次 Gazebo 运行一行：结果、触发器指标、扫视转角中位数、整段 RTF |

## 2. 复现

```bash
cd ecps295 && python my_minifly.py v3 --out minifly/minifly_v3.npz        # 网络权重（不入库）
export PYTHONPATH=../src:.
python -m rl.episode --config rl/configs/v3_2_es3.yaml --seconds 120      # 单回合（cage20）
python -m rl.cmaes_run --run es_x --gens 30 --k 12 --popsize 12 --seconds 90 --procs 12 --space wide
python -m rl.cmaes_run --run a1_shuffle1 --arm shuffle1 --gens 20 --k 12 --popsize 12 --seconds 90 --procs 12 --space wide
```

快仿真在 Jetson AGX Orin（numpy 1.26）和云服务器（numpy 2.4）上逐位一致。12 进程合计速度：Jetson 约 19.5 倍实时，ROG 约 30 倍实时。

**Gazebo 多实例（云服务器，无 sudo）**：用 conda 安装 Gazebo Harmonic 8.10，ArduPilot Copter-4.7.0 打上 `sitl/ardupilot_sitl.patch`，ardupilot_gazebo 用 `082a0fe`。

```bash
bash server/gz_inst.sh <I> ecps295_cage20.sdf <tag> "<run_g4 args>"       # 一个隔离实例
python server/gz_batch.py --slots 12 --seeds 0-31 --prefix fix --arm es3=minifly/v3_2_es3.yaml --arm brainoff=minifly/a1_intact.yaml@brainoff
```

服务器上的 `run_g4.py` 有三处小改动，都由环境变量控制，默认不生效，所以没有提交到这里：

1. 在 `import time` 之后加入：设置了 `ECPS_SIMCLOCK` 时执行 `simclock.install(path)`，用 Gazebo 相机帧上的仿真时间驱动 `time.*`。在这台机器上锁步只能跑到约 0.36 倍实时，因为单核较慢，插件和 SITL 之间每 1 ms 都要往返一次；用了仿真时钟后，RTF < 1 的运行同样有效。
2. 用 `rl.arms.connectome_for(...)` 加载网络，构造完 Pilot 后调用 `rl.arms.install(ARM, ...)`，其中 `ARM = $ECPS_ARM`。
3. 在 `gz_camera_drone.py` 中，共享内存路径的默认值改为 `os.environ.get("ECPS_SHM", ...)`；新版 `run_g4.py` 支持 `--shm` 参数时，由 `gz_inst.sh` 直接传入。

## 3. 目前的结果

### 3.1 slew 修复（提交 8b8eef9）改变了基线

`SafetyGovernor` 原来把"负值回到 0"当成变号来限速。修复后，Gazebo cage20（GPS 巡航 120 s，各 32 个种子）的结果：

| 版本 | 有接触的运行 | 平均接触 | 险情 | 覆盖率 |
|---|---|---|---|---|
| v3_2，有缺陷 | 10/32 | 0.59 | 1.28 | 0.664 |
| es3，有缺陷 | 2/32 | 0.06 | 0.66 | 0.612 |
| **v3_2，已修复** | **8/32** | **0.25** | **0.84** | 0.651 |
| **es3，已修复** | **2/32** | **0.12** | **0.81** | 0.627 |

修复后 es3 对比 v3_2 的显著性：有接触的运行 p = 0.08，接触次数 p = 0.051，险情 p = 0.78，覆盖率 p = 0.17。修复前 es3 显著的优势，有相当一部分其实是 v3_2 受缺陷影响的那部分。

### 3.2 覆盖率为什么下降（B1）

用 Gazebo 的数据（修复前，各 32 次）对比 es3 和 v3_2：

| 指标 | v3_2 | es3 |
|---|---|---|
| 平均速度（m/s） | 0.197 | 0.209 |
| 慢速比例 | 9.0% | 3.4% |
| 虚警（次/分钟） | 1.03 | 0.92 |
| 扫视转角中位数 | **76°** | **173°** |
| 转角 >135° 的扫视占比 | 2% | 99% |
| 重新进入已飞过格子的比例 | 42% | 48% |

es3 并没有变慢或变犹豫，而是每次扫视都原地掉头、沿来路返回，所以总在已飞过的区域里来回，覆盖率因此下降。

### 3.3 与 v4（S8 的完整系统）在 cage20 GPS 下的对比（均已修复 slew）

| 版本 | 有接触的运行 | 平均接触 | 险情 | 覆盖率 | 每 100 m 撞击 |
|---|---|---|---|---|---|
| v3_2 | 8/32 | 0.25 | 0.84 | 0.651 | 约 0.98 |
| v4 | 21/32 | 0.91 | 1.94 | 0.653 | 约 3.5 |
| es3 | 2/32 | 0.12 | 0.81 | 0.627 | 约 0.45 |

三个版本的接触**全部**发生在网笼边上箱垛的下层箱子 `stack_low`（v3_2 8 次，v4 29 次，es3 4 次）。

这个结果和 S8 并不矛盾：S8 测的是 v4 内部各组件的贡献，场景是 lowbox、光流加路线 B。S8 没有和 v3_2 比较过；而且在 lowbox 里，阶梯 v3 本身也不比 v4 差。

我的假设是：v4 把 near 居中视野从 5 列改回了 3 列，看不到斜着经过的箱角。`configs/v4_oc5.yaml` 用来验证这个假设，Gazebo 复测进行中。

### 3.3b v4_oc5：v4 把居中视野改回 5 列（cage20 GPS，30 个种子，配对对照 v3_2）

| 版本 | 有接触的运行 | 每 100 m 撞击（与 v3_2 的配对差 [95% CI]） | stack_low 上的接触 |
|---|---|---|---|
| v4 | 21/32 | +2.62 [+1.43, +3.85]，Holm p = 0.002 | 29 |
| v4_oc5 | 11/30 | +1.17 [−0.26, +2.99]，p = 0.23 | 16 |
| es3 | 2/32 | −0.52 [−1.37, +0.52]，p = 0.23 | 4 |

把居中视野改回 5 列，v4 的接触几乎减半（对 v4 p = 0.022），覆盖率不变，所以 `outer_cols: 3` 是 v4 在 cage20 退步的主要原因之一。剩余差距可能来自 v4 网络里新增的神经元或解码器版本。另外，**在配对检验下，es3 对 v3_2 的改进不显著**（McNemar 救回 8 / 新增 2，Holm p = 0.22）。

### 3.3c 空场扫视率（B1，快仿真，32 个种子 × 60 s，只有网笼）

v3_2 每分钟 1.38 次，es3 1.22 次，a1_intact 1.19 次；约 2/3 到 9/10 的运行至少误扫视一次。这是各版本共同的问题，优化没有解决它。C 阶段的 efference copy（随速度和转角抬高逼近门限）就是针对这一点。

### 3.4 A1 对照臂（快仿真，同一协议，各自留出集上的最优代）

所有臂都从 v3_2 出发，在 wide 参数空间中优化 20 代 × 12 个候选 × 12 个回合。

| 臂 | 最优代 | 留出集得分 F | 有接触的回合比例 | 覆盖率 |
|---|---|---|---|---|
| intact | 19 | −0.161 | 0.25 | 0.48 |
| bypass | 4 | +0.169 | 0.00 | **0.27** |
| shuffle1 | 4 | −0.122 | 0.29 | 0.53 |
| shuffle2 | 9 | −0.202 | 0.29 | 0.54 |
| shuffle3 | 14 | −0.087 | 0.17 | 0.37 |
| shuffle4 | 19 | −0.040 | 0.25 | 0.53 |
| shuffle5 | 4 | −0.205 | 0.33 | 0.52 |
| randread1 | 9 | +0.249 | 0.00 | **0.10** |
| （v3_2，未优化） | — | −0.510 | 0.33 | 0.50 |

初步来看，打乱连线后再用同样预算优化，结果和真实连线相当。bypass 和 randread 的最优解是"几乎不动"，说明适应度函数允许这种退化解，下一轮需要加上最低覆盖率约束。Gazebo 上的 32 种子复测进行中，结论以那次为准。

### 3.5 B2 覆盖率权重扫描（从 es3 热启动，快仿真留出集）

w_cov = 2、4、8 时，覆盖率分别为 0.58、0.57、0.62（v3_2 是 0.50），有接触的回合比例分别为 0.38、0.21、0.38（v3_2 是 0.33）。Gazebo 复测进行中。

## 4. 局限与注意事项

- 快仿真对接触偏悲观：逼近信号约弱 25%，所以避障触发偏晚。快仿真只用来筛选，结论以 Gazebo 为准。
- 本目录所有 Gazebo 数字都来自云服务器（Gazebo 8.10，加仿真时钟），而 ROG 是 8.15、要求 RTF ≥ 0.95。服务器上的 v3_2 基线与 ROG 一致：有接触的运行 31%，ROG 上 5 个种子是 40%。
- 8b8eef9 之前的所有结果（包括 S8）都是在 slew 缺陷下跑出来的。
- 第一轮试验 es1（17 个参数，default 空间）的数据留在 ROG 的 `~/sim/rl_exp/`，没有收录在这里。

## 5. 待办

- 完成 A1 + B2 的 Gazebo 复测（15 组配置 × 32 种子），用 `stats.py` 出配对结论，并更新本文件。
- 进行中（Jetson）：B 适应度 v2（`--fitness v2`：覆盖权重 2、平滑接近代价 0.5·mean exp(−(clr/0.3)²)、覆盖率低于 0.40 时按差额 ×3 扣分）+ `turncap` 空间（扫视时长 1.2–2.5 s，约 60–140°，杜绝 173° 掉头），3 个 CMA 种子、各自独立的训练世界流（A4）。
- B3：把 Governor 和围栏的介入计入代价。
- 之后进入 C（efference copy 参数、通路增益和泄漏、批量化快仿真）和 D（lowbox、光流导航、ghost 等分布外条件），与 S8 的场景对齐。
