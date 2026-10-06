# 果蝇连接组控制项目的 RL / 优化做法调研，与 ECPS295 的定制技术路线

> 2026-10-06 · 起点文档：`FlyDrones_MiniFly_社区调研_20261005.md`
> 方法：6 路并行深读。用 GitHub API 直接读 13 个仓库的文档和关键代码（共约 170 次调用），重点核对代码与文档是否一致；另在本项目代码上做了一次核实（第 4.3 节）。
> 文中其他项目的所有数字均为**作者自报、未复现**。过程材料位于 `.research/20261006-flybrain-rl-routes-c983/`。

## 摘要

上一份社区调研只说明了"大家都在冻结连接组、只训读出"。这次深入代码后发现，各项目真正拉开差距的地方**不在优化器，而在评估纪律**。下面五点决定了本项目接下来该做什么。

**第一，规模越大的学习方案，塌缩得越彻底。** garyb9 先后试过学习编码器、联合 SAC、单解码器 SAC，结果全部塌缩：编码器电流饱和；一解冻 actor，克隆来的技能就被毁掉。作者最后把整条学习路线标为 deprecated，原话是"RL fitted the simulator rather than exercising the connectome" [来源](https://github.com/garyb9/fly-drone/blob/main/docs/results/encoder-v6/DEPRECATION.md)。fly-craftax 的 PPO 线性读出跑了 7.7 万步，最后学到的是"一个带少量内驱抖动的时钟"，视觉全黑也照样能用 [来源](https://github.com/liuzihe02/fly-craftax/blob/main/tracker.md)。真正跑出可用结果的，都是低维、带强先验的方法：garyb9 的小 MLP 加闭环 warm start、haltere 的 3 行读出岭回归、flyjump 的 243 参数 CEM。本项目用 CMA-ES 调 17 个解码器参数，属于这一类，方向是对的。

**第二，"行为来自连接组"这一说法，几乎在每个认真做对照的项目里都被削弱过。** flyjump 中，不经连接组、参数量对齐的直连网络，10 个种子的平均成绩与连接组版本**完全一样**（92.2 对 92.2）[来源](https://github.com/cobanov/flyjump/blob/main/docs/experiment.md)。fly-self-driving 的平路任务上，打乱后的图不差于真实图 [来源](https://github.com/suanmiao/fly-self-driving/blob/main/docs/results.md)。abbosaliboev 的连接组、规则控制器、随机控制器成功率都是 16.7% [来源](https://github.com/abbosaliboev/fly-brain-drone/blob/main/experiments/results/benchmark_summary.csv)。由此得到一个关键的方法学结论：**对照组必须用同样的优化预算重新优化**，否则对照不公平。本项目 es3 的 −90% 接触，目前还没有经过任何对照。

**第三，es3 的覆盖率下降，很可能是"变保守"，不是"变聪明"。** fly-craftax 发现，部分成就其实是策略自己的动作诱发的 [来源](https://github.com/liuzihe02/fly-craftax/blob/main/tracker.md)；huang12chen 在黑夜条件下"准确率上升"，实际是用漏检换来了少报警 [来源](https://github.com/huang12chen/flywire-brain-drone/blob/main/REPORT.md)。两者都提醒我们，必须把触发器的召回、虚警和慢速比例拆开报告。es3 学到的参数（扫视门限压到 2 Hz、不应期 5.6 s、刹车几乎全靠 DNp01）也说明，它整体变得更"一惊一乍"。

**第四，上真机的头号失效模式，是自身运动诱发的伪 looming。** Parth-Joshi0 的 Tello 实飞中，约 1 分钟内触发了 19 次逃逸，37% 的帧处于逃逸状态，节奏正好等于"闪避 + 不应期"的周期 [来源](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Drone/flight_logs/tello_escape_flight_test.log)。后来靠 efference copy（随速度和转角抬高 looming 门限）才压了下来。本项目的快仿真和 Gazebo 都比真实鱼眼"干净"，所以这个问题在仿真里被低估了。

**第五，本项目自己的安全层有一个可以立刻修的缺陷。** 在 kistik1 的 SafetyGovernor 中发现的 slew 不对称问题，经核实**同样存在于本项目**：负向命令松开后要 150 ms 才回零，正向命令则立即回零（见 4.3 节）。这意味着向一侧的扫视和后退，结束时都会比反方向拖得更久。

**定制路线的一句话版本：** 先补证据（6 臂对照矩阵，各臂同预算重优化，配对统计）和指标（拆出覆盖率损失的来源），再优化（多目标 CMA-ES 找回覆盖率、通路增益加稳定门槛扩展搜索空间），然后做泛化（多个世界、光流导航、失纹理和 ghost 条件），最后修安全层并按 L0–L4 分级上真机。蘑菇体可塑性作为独立的生物学模块，按 AleksiAero 的结构实现，并与关闭学习的版本做配对比较。

---

## 1. 起点：已经有的和缺的

本项目当前的系统：MiniFly v3_2（946 个 LIF 神经元，手工连接）→ EcpsDecoder（DN 线性读出加刹车/扫视/谨慎状态机）→ SafetyGovernor 加围栏转向 → ArduPilot GUIDED 速度指令。

优化方面，在按 Gazebo 标定过的快仿真上，用 CMA-ES 调了 17 个解码器参数，训练用程序化生成的随机笼子，用固定的留出集选参，cage20 完全留出不参与训练。es3 在 Gazebo cage20 上各跑 32 个种子的结果：

| 指标 | v3_2 | es3 | p 值 |
|---|---|---|---|
| 有接触的运行比例 | 31% | 6% | 0.022 |
| 平均接触次数 | 0.59 | 0.06 | 0.009 |
| 险情 | 1.28 | 0.66 | 0.03 |
| 覆盖率 | 0.664 | 0.612 | 0.006（变差） |

和社区项目相比，本项目有三处明显的优势：

- Gazebo + ArduPilot SITL 的仿真链路，比大多数项目用的 MuJoCo/PyBullet 理想位姿更接近真机；
- 已经做到了训练、选参、测试三套数据分离，32 种子的统计检验也比社区常见的 n=5–10 更扎实；
- 算力分层：Jetson 跑快仿真优化，服务器跑 Gazebo 多实例复测。

但对照社区的做法，还缺四样东西：

1. 没有任何对照来证明收益来自 MiniFly，而不是来自状态机、SafetyGovernor 或 CMA-ES 本身；
2. 没有把覆盖率下降拆解到具体的触发器行为；
3. 只有单次 CMA-ES 运行，不知道不同运行种子之间的离散度；
4. 上真机的分级流程和安全层的细节都还没有落地。

---

## 2. 各项目怎么做优化

### 2.1 garyb9/fly-drone：冻结全脑、PPO 训练小解码器，以及一长串诚实记录的失败

**做了什么。** 冻结 MaleCNS 全脑（166,700 个 LIF 神经元），只学一个 2,022→32→32→4 的 MLP 解码器。观测是下行神经元和 VNC 运动神经元的活动迹，动作是 4 维速度和偏航率，决策频率 25 Hz。PPO 的配置很小：rollout 约 1024，lr 3e-4，warm start 之后降到 1e-5；单个房间任务只训练 30k–60k 步 [来源](https://github.com/garyb9/fly-drone/blob/main/docs/training.md)。验收结果是转向 100%、躲避 96%（balanced 0.92）。

**奖励设计的核心是防止刷分。** 转向进度用势函数差 `10(|β_{t-1}|−|β_t|)`，整回合求和恰好等于方位角的净减少量，来回振荡挣不到分。自由漫游中，威胁项改为 `Φ(d)=exp(−(d/1.5)²)` 的势差，而且只在前后两步都存在威胁时生效，防止靠"制造或消灭威胁"刷分。覆盖奖励按"首次进入一个 1 m 网格"计 +0.5 [来源](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/env.py)。looming 任务还加了"偏离起点"的惩罚，并规定障碍物进入 2 m 时无人机仍需在起点 0.25 m 以内，提前逃走不算躲避成功 [来源](https://github.com/garyb9/fly-drone/blob/main/docs/validation.md)。

**因果证明分两层。** 第一层是训练前的感觉门控 `sensory_assay`：左右刺激必须让特征可分（差值 >1e-4），静默感受器后必须与暗场完全一致（差值 <1e-6），不通过就直接抛异常、拒绝开训 [来源](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/assay.py)。第二层是评估时在 50 个留出种子上配对比较四种条件：完整、特征置零、感受器静默、特征打乱。判据用 **balanced 成功率**，即左右两侧成功率取较小值，这样"永远朝一侧转"的盲策略混不过去：v3 版本在感受器静默条件下的 48% 成功率全部来自左侧，这一层判据把它拦下了 [来源](https://github.com/garyb9/fly-drone/blob/main/docs/validation.md)。

**两个最值得记住的教训。**

- 一是 warm start 的数据必须在闭环中采集。只用静止帧标定的解码器，换到新编码器后转向成功率跌到 8%，原因是自身转动产生的 looming 不在静止帧的分布里。改为"带噪教师闭环飞行采集"之后，仅 warm start 就达到 10/10 [来源](https://github.com/garyb9/fly-drone/blob/main/docs/validation.md)。
- 二是高维 RL 几乎必然塌缩。816 维编码器动作存在"常数或饱和电流"这种退化最优解，任务奖励、克隆锚定、预测辅助目标、活跃度门槛、熵调节都没能阻止 [来源](https://github.com/garyb9/fly-drone/blob/main/docs/results/encoder-v6/CONCLUSION-2026-09-18.md)。

他们的工程纪律值得照抄：阈值预注册、事后不放宽；条件之间按种子逐个配对；用 paired bootstrap 和 Wilson 区间；每轮候选都要过 `round_eligible` 守卫，所有塌缩的策略都是被这道守卫拦下的 [来源](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/roam_eval.py)。

### 2.2 skulitom/haltere：3 行读出的闭式蒸馏，以及 PD 影子基线

**模型。** haltere 用的不是脉冲网络，而是"连接组约束的速率 RNN"：从 MaleCNS 中选 3 万个神经元，突触结构和递质符号固定，学习的是有界的边增益、神经元的增益、偏置和时间常数 [来源](https://github.com/skulitom/haltere/blob/main/haltere/brain/model.py)。早期版本用可微仿真做 BPTT。

**后期主力是 3 行读出的闭式蒸馏。** 用 DAgger 收集"大脑实际访问到的状态 + 手写快速 PD 给出的标签"，然后以父模型为中心做加权岭回归，同时惩罚相邻指令的变化：

```
min Σw‖y − x(P+Δ)ᵀ‖² + ridge‖Δ‖² + smooth·mean‖step·(P+Δ)‖²
```

求解只改油门、横滚、俯仰这 3 行读出。求解后代码会对比全部参数，任何其他参数被改动就直接报错 [来源](https://github.com/skulitom/haltere/blob/main/haltere/train/fast_motor_tracking.py)。一次完整蒸馏在 CPU 上约 45 分钟。

**PD 影子基线是这个项目方法论上最有价值的设计。** 在同一套 pilot、相机和速度上限下，改由 PD 控制电机，大脑照常运行、只记录不输出，视频上标注"BRAIN IN SHADOW"。2026-09-23 的冻结对照中，大脑 0/2 完赛、PD 1/2 完赛，但两者都撞在了同一根柱子上。由此可以分辨出：主要瓶颈在几何感知，跟踪是次要瓶颈 [来源](https://github.com/skulitom/haltere/blob/main/docs/flight_cards/2026-09-23_matched_full_races.md)。

**其他值得借鉴的做法：**

- **门槛先冻结、再评分。** brain-09 有 22 个候选，最新几版大脑都没能通过门槛。
- **数据家族降级。** 某一类赛道一旦用于调参，就降为开发数据，下一次对外报告的对比必须换一批没碰过的赛道 [来源](https://github.com/skulitom/haltere/blob/main/docs/generalization_program.md)。
- **消融组做得完整。** MLP 组去掉连接组，用来证明任务本身可学；reservoir 组冻结内部；robust 组做 ±35% 物理随机化；smooth 组加延迟和平滑惩罚。关键发现是：读出层必须接入前运动神经元，R² 能到 0.7–0.8；只读翅运动神经元时 R² 只有 0.4–0.5，闭环 0% 能稳住 [来源](https://github.com/skulitom/haltere/blob/6e86b8b82f77228ec71919a5c50583ad83258d2b/README.md)。

**结果并不好看。** 没见过的赛道 0/5 完赛，见过的赛道也比人类慢约 3.4 倍。

### 2.3 fly-craftax（PPO）与 flyjump（CEM）：两份"对照否定了连接组贡献"的报告

**fly-craftax** 冻结 MaleCNS 全脑（JAX 实现），用 PPO 训练一个**线性**读出：1,314 个 DN 发放率 → 7 个动作，参数零初始化。总共 76,800 步，在 4090 上跑了 58.6 分钟，只跑了一个种子；回报大约在第 40 次更新就进入平台期 [来源](https://github.com/liuzihe02/fly-craftax/blob/main/flycraftax/ppo.py)。

它的对照脚本 `eval_ppo_controls.py` 在新的 key 上回放训练好的读出，对比了几种条件：视觉全黑、视觉冻结在第一帧、切断全部输入、开环 FORWARD/DO 交替、随机策略。全黑条件下存活 204、完整条件 176，两者的动作分布几乎一样。作者的结论是"一个带少量内驱抖动的时钟"[来源](https://github.com/liuzihe02/fly-craftax/blob/main/scripts/eval_ppo_controls.py)。

**flyjump** 用一个 80 细胞的 MaleCNS 子图，配合一个 243 参数的 MLP 读出，用对角高斯 CEM 优化：种群 64、精英 8、共 80 代 [来源](https://github.com/cobanov/flyjump/blob/main/src/lib/training.ts)。它的选参协议值得完整照搬：

- 每代所有候选共用同一批 3 条新的随机赛道（公共随机数）；
- 候选 0 固定为当前验证冠军；
- 只有在验证集上严格变好才替换冠军；
- 训练、验证、测试三套种子完全分离。

**最关键的是对照结果。** 每个控制器用 10 个训练种子，在 100 条留出赛道上测试：连接组版本平均完成 92.2，参数量对齐的直连网络 8-20-3 也是 92.2。作者明确写道"任务不需要连接组"[来源](https://github.com/cobanov/flyjump/blob/main/docs/experiment.md)。另一个细节是：作者主动把"电路输出置零"这个对照改了名字，因为它只能说明"游戏信息必须经过电路"，并不能说明"电路的计算有帮助"。

### 2.4 fly-self-driving 与 AleksiAero：有界增益和泄漏、增益扫描、蘑菇体学习

**fly-self-driving 训练的是速率模型，不是脉冲网络。** 它在整个 MaleCNS 上为每条边学一个增益、为每个神经元学一个泄漏，共 2,570 万个参数。两者都用 sigmoid 压进固定区间：`w = base·(0.05+0.90σ(g))`。这样连接既不会被删掉，也不会被加上，更不会变号；结构约束完全靠参数化实现，没有加任何正则项 [来源](https://github.com/suanmiao/fly-self-driving/blob/main/flyhard-patches/connectome.py)。

工程上有两个关键点：

- **状态在决策之间延续。** 只改这一项，平路任务就从 3/20 提到 17/20。
- **DAgger。** 街道任务上，只做行为克隆是 12/20，加 DAgger 后三组留出集分别是 20·19·19 [来源](https://github.com/suanmiao/fly-self-driving/blob/main/docs/results.md)。

它的对照结果并不一边倒：平路上打乱图不差于真实图，46k 参数的 MLP 又比两者都好；只有在带交通的街道任务上，真实图才领先（20 对 16/20）。另外有一个值得注意的细节：训练损失几乎没有下降（0.00260→0.00288），说明**闭环评估才是唯一可信的判据** [来源](https://github.com/suanmiao/fly-self-driving/blob/main/results/street/street-v5-measured-stateful.json)。

**AleksiAero 的增益扫描 `scan_gain.py`。** 对全局突触权重做扫描，每个取值先给 0.5 s 偏航光流，再给 0.5 s 静默。合格判据有三条：撤掉刺激后活动要自行熄灭，偏航 DN 的左右方向要正确，looming 要能激活 GF。按此选定 0.12 mV，而 Shiu 2024 的原值 0.275 mV 会让整个网络进入自持放电 [来源](https://github.com/AleksiAero/Aleksi-Aero-Fly-Brain-Drone/blob/main/scripts/scan_gain.py)。

**AleksiAero 的蘑菇体学习**可以当作实现模板：

- 规模：4,064 个 KC、44,042 条可塑的 KC→MBON 突触；
- 资格迹时间常数 τ=2 s，学习率 η=0.0012；
- 只有"高于基线 5 Hz 以上"的相位性多巴胺才参与教学；
- 突触因子 g∈[0,1]，只做乘性抑制，并以 τ=900 s 缓慢恢复；
- 每 50 ms 批量写回一次；
- 教学信号：撞墙速度 >0.6 m/s 触发 PPL1 惩罚 1 s，轻碰 0.3 s；充电触发 PAM 奖励。

[来源](https://github.com/AleksiAero/Aleksi-Aero-Fly-Brain-Drone/blob/main/ros2_ws/src/fly_drone_sim/fly_drone_sim/mushroom_body.py)

但它**没有证明学习让飞行变好**。唯一的定量结果是开环测试中奖励视图的效价上升了 +5~+7；闭环的多生命周期实验只给了脚本，没有结果。而且代码里默认开着一个手写的趋光转向，会和学习效果混在一起 [来源](https://github.com/AleksiAero/Aleksi-Aero-Fly-Brain-Drone/blob/main/README.md)。

### 2.5 评估方法学：ClutchMedia、huang12chen、abbosaliboev

**ClutchMedia 的对照设计最完整。** 它有四组对照：

- 真实连线；
- 打乱连线：对 post 列做全局置换，保留出度、入度以及每条边的符号和权重；
- 随机读出：从同侧 DN 中随机抽同样数量的神经元替代真实读出组；
- 关闭大脑。

统计上用 2000 次 bootstrap 求置信区间 [来源](https://github.com/ClutchMedia775/fly-brain-drone/blob/main/sim/flybrain_step.py)。它的读出层是按生理学先验手工设定的，没有针对真实连线拟合，所以对照是干净的。

但它有三个弱点：只有 3 个打乱实例；其中 shuffle1 仍保留了 50% 的规避率，所以 README 里"打乱连线破坏全部行为"的说法有夸大；"关闭大脑"等于完全不动，作为对照太弱 [来源](https://github.com/ClutchMedia775/fly-brain-drone/blob/main/results/phase56_summary.json)。

**huang12chen 的指标拆分最值得照抄。** 它把指标拆成：

- 触发召回率和虚警率；
- 方向误差，<30° 才算成功；
- GF 首次放电潜伏期。

还用"静止基线 + action_saved"的配对物理积分，算出机动**真正**救回了多少：只有 +1.0 到 +1.5 个百分点 [来源](https://github.com/huang12chen/flywire-brain-drone/blob/main/web/evaluate.js)。

回归防护方面也有几样可以直接复用 [来源](https://github.com/huang12chen/flywire-brain-drone/blob/main/web/jitter_sim_log.md)：

- 结果 JSON 用 SHA256 锚定；
- 语义冒烟门槛（例如强威胁 ≥60% 放电、无威胁 ≤5% 放电）；
- 用抖动模拟估计"模型没变、验收却被种子噪声误判为失败"的概率，约为 8%。

**abbosaliboev 给出了最有价值的负面结果。** 在统一时间常数、固定一步延迟的 LIF 模型里，真实拓扑产生不了 T4/T5 方向选择性。它还发现物理混控器的偏航符号接反了，在此之前的所有闭环演示，无人机都在往控制器意图的反方向转 [来源](https://github.com/abbosaliboev/fly-brain-drone/blob/main/docs/scientific_assumptions.md)。

### 2.6 安全与上真机：Parth-Joshi0、kistik1、tiago369、DylanZhangzzz

**Parth-Joshi0：独立安全层和 efference copy。** 数据流是 `Controller.decide → SafetyLayer → DroneInterface`，安全层只能让命令变得更安全，不会产生新的意图。安全层的阈值必须**高于**导航器自身的避障阈值。这条规则来自一个真实 bug：两个转向决策器对同一个光流读数给出相反判断，引起振荡，振荡又抬高光流，形成正反馈 [来源](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Controllers/safety_layer.py)。

efference copy 的写法是 `floor = 0.8 + 0.5·v_fwd + rotation_floor`，其中 rotation_floor 随机体转角抬高并以 0.7 的系数衰减。真机版本又加了几层：命令幅度大时以及命令结束后 0.6 s 内忽略 looming，忽略画面下 1/3，再次触发需要信号先降到 0.3 以下 [来源](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/NeuralPathways/flybrain_controller.py)。

上真机的分级流程是 [来源](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Drone/README.md)：

1. 无硬件冒烟测试，**每次硬件会话前必跑**；
2. 拆桨感知测试，从不调用起飞；
3. 标定，R²<0.5 禁飞；
4. 只闭合单个回路的短时实飞，15 s 自动降落，起飞后 2 s 内屏蔽大脑输出；
5. 全任务。

需要注意，Tello 真机实飞时**并没有经过 SafetyLayer**，这一层实际上只在仿真里生效 [来源](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Controllers/README.md)。

**kistik1：Mavic Air 桥和 SafetyGovernor。** 主要做法有：

- 300 ms 收不到命令就发零命令；
- 每次握手生成会话 token，序号必须严格递增；
- 只能在手机上 ARM；
- 双重限幅（Python 端 ≤0.15–0.20，Android 端 ≤0.25）；
- 先用 mock bridge 测断连、重放、畸形包。

至今只做到拆桨台架测试，只发过零命令，没有飞过 [来源](https://github.com/kistik1/FlyDrones/blob/main/docs/MAVIC_AIR.md)。它的 MAVLink 后端用 `SET_POSITION_TARGET_LOCAL_NED`、`BODY_OFFSET_NED`、只开速度和偏航率的 type_mask，可以直接复用；但它的 `emergency_stop` 是强制 disarm，飞机会直接掉下来 [来源](https://github.com/kistik1/FlyDrones/blob/main/src/flydrones/drones/mavlink.py)。

**tiago369：门控加缓升。** 起飞时地面后退会产生光流突发，导致约 145° 的非指令偏航自旋。修复方法是：只在高度误差 <0.1 m 时才打开光流门，门一旦打开就锁存，并在 1 s 内线性升高增益 [来源](https://github.com/tiago369/fly_robot_control/blob/master/fly_controller/config/controller_manager_m4.yaml)。至于它宣传的"haltere 0.53 s 恢复，而 PID 超过 12 s"，经核查两者的姿态和角速率增益逐项相同，这个差异不能当作生物机制优于 PID 的证据 [来源](https://github.com/tiago369/fly_robot_control/blob/master/fly_brain/include/fly_brain/haltere_reflex.hpp)。

**DylanZhangzzz：解剖坐标视觉映射。** 每个 L2 视觉神经元对应一条已发表的视线方向，经相机内参投影后采样；看不到的通道保持为空，不做外推。它明确拒绝接入 MiniFly，因为 MiniFly 没有真实的神经元 ID [来源](https://github.com/DylanZhangzzz/Fruit-fly-vision-bridge/blob/main/docs/flydrones.md)。对本项目来说，可以借鉴的是它的验证方式：零输入、断开网络、半增益三种因果对照，以及阈值冻结后的前瞻性复测。

---

## 3. 横向对比：共识与教训

| 项目 | 学什么 | 怎么学 | 预算 | 对照 | 结论可信度 |
|---|---|---|---|---|---|
| garyb9 | 32-32 MLP 解码器（2,022 维输入） | 闭环 warm start → PPO | 30k–60k 步/任务 | 四种条件 + balanced 判据 + 预注册阈值 | 高（房间任务）；自由漫游无一通过 |
| haltere | 3 行读出（另有早期 BPTT） | DAgger + 父中心岭回归 | 约 45 分钟 CPU | MLP、reservoir、PD 影子基线、冻结门槛 | 中；没见过的赛道 0/5 |
| fly-craftax | 线性读出 1,314→7 | PPO | 76,800 步，1 个种子 | 全黑、冻结视觉、切断输入、开环、随机 | 高，结论是"学到的是时钟" |
| flyjump | 243 参数 MLP 读出 | 对角 CEM | 每次 15,680 回合 × 10 个种子 | 直连网络（参数量对齐）、置零、规则、随机 | 高，结论是"不需要连接组" |
| fly-self-driving | 每边增益 + 每神经元泄漏，2,570 万参数 | BPTT + DAgger | 约 35 分钟 H100 | 保度打乱、MLP、线性 | 中；每条件 1 个种子 |
| AleksiAero | KC→MBON 可塑性 | 三因子规则 | — | 大脑开/关 | 低；没有闭环学习结果 |
| ClutchMedia | 不学习 | — | — | 打乱 ×3、随机读出 ×3、关闭大脑 | 中 |
| **本项目 es3** | 17 个解码器参数 | CMA-ES | 30 代 × 12 候选 × 12 回合 | **暂无** | 统计上显著，但来源没有证明 |

把这张表合起来看，可以归纳出五条共识。

**第一，可用的结果都出自低维、强先验的优化。** garyb9 的大规模 SAC、fly-craftax 的高维线性 PPO 都没有成功；garyb9 的小 MLP 加闭环 warm start、haltere 的 3 行岭回归、flyjump 的 243 参数 CEM 都能工作。CMA-ES 在 17 维上用完整协方差，比 flyjump 的对角 CEM 更合适。flyjump 可以借鉴的是它的选参协议，而不是优化器本身。

**第二，对照必须"同预算重优化"。** flyjump 的直连网络、fly-self-driving 的 MLP 和打乱图，都是用同样的流程训练出来的，所以它们与连接组打平的结论才有分量。ClutchMedia 不训练任何参数，所以可以直接替换连线来比较；但本项目的解码器是为 MiniFly 调过的，如果直接把连线换成打乱版本来比，对照组天然吃亏，结论站不住。

**第三，"关闭大脑"只能证明信息经过了大脑，不能证明大脑的计算有用。** flyjump 作者主动把这个对照改了名，就是这个原因。真正有说服力的是"绕过大脑的直连对照"。

**第四，判据要防止刷分。** 具体有几种手段：balanced（左右取较小值）、势函数塑形、"提前逃走不算躲避"、action_saved（与什么都不做相比，真正多救回了多少）、触发器的召回和虚警分开报告。只看一个综合准确率，容易误判。

**第五，工程纪律比算法更决定结论能不能站住。** 具体包括：预注册且冻结的阈值；按种子配对的设计；多个训练种子的复制实验；三套种子分离；数据家族一旦用过就降级；结果哈希锚定；语义冒烟测试；在 SITL 里实际发指令验证符号。garyb9 的所有塌缩都是被守卫拦下的，abbosaliboev 的偏航接反也是靠实测发现的。

---

## 4. 用这些经验诊断 es3

### 4.1 收益到底来自哪里，目前还说不清

es3 让接触减少了 90%，但本项目的回路里有好几层东西都能避障：MiniFly 的 DN 读出、解码器的刹车/扫视/谨慎状态机、SafetyGovernor 加围栏转向，还有 CMA-ES 调出来的阈值。

es3 学到的参数本身就是一条线索：扫视门限压到了下限 2 Hz，几乎有动静就转；不应期拉长到 5.6 s，避免连续扫视；刹车几乎完全依靠 DNp01（权重 0.3），DNp03 的刹车权重几乎降到 0；谨慎保持期缩短到 0.8 s。这说明优化器主要在调**状态机的时序**，而 MiniFly 只提供了一个"有没有东西在逼近"的粗信号。如果这是真的，那么一个把 EcpsRetina 的 looming 特征直接接到同一套状态机、用同样预算优化的直连对照，很可能和 es3 打平。这正是 flyjump 和 fly-craftax 遇到的情况，所以必须先测。

### 4.2 覆盖率下降更像是"变保守"

es3 的路程（26.8 m）比 v3_2（25.6 m）还长，覆盖率却低了 8%，说明它更多地在已经飞过的安全区域里反复绕。参照 huang12chen 的指标拆分，需要分别回答几个问题：

- 空场里的扫视率是不是升高了？
- 扫视后 2 s 内又扫视一次的"链式触发"比例是多少？
- 在相同平均速度下，接触率是否仍然更低？
- 围栏转向和 Governor 的介入时间变了多少？

只有把这些拆开，才能决定是在适应度里补覆盖奖励，还是在状态机里修触发逻辑。

### 4.3 主控核实：本项目 SafetyGovernor 的 slew 不对称

子任务 06 在 kistik1 的代码里推断出一个 slew 限幅缺陷。kistik1 和本项目来自同一个上游，所以我在本项目的代码（`src/flydrones/safety.py` 第 91 行）上做了实测。设定 slew 为 2.5/s、周期 50 ms：

- yaw 从 +0.5 松开，下一个周期立即归零；
- yaw 从 −0.5 松开，依次经过 −0.375 → −0.25 → −0.125 → 0，**需要 150 ms**。

原因是 `math.copysign(1, 0.0)` 等于 +1，于是"负值回零"被误判为"变号"，按变号的规则限速了。

这会带来三个影响：

1. 后退（forward 为负）的释放比前进慢 150 ms；
2. 两个方向的扫视转向，释放速度不一样；
3. 已经跑完的 v3_2、es2、es3 全部带着这个缺陷，CMA-ES 也可能已经在迁就它，比如它把后退时长调成了 1.24 s。

修复只需一行：把条件改成 `abs(v) > abs(prev) or v * prev < 0`。但修完之后，**v3_2 和 es3 都要重新建立基线**，因为参数是在有缺陷的环境下优化出来的。

### 4.4 只有一次运行

es3 只来自一次 CMA-ES 运行。flyjump 的经验是，10 个训练种子的完成数从 78 到 100 不等；本项目的 es1 也曾在留出世界上输给 v3_2。所以需要用 3–5 个种子复跑，报告结果的分布，而不是只挑最好的一次。

---

## 5. 定制技术路线

整条路线分六个阶段，A、B 可以并行，C 依赖 A、B 的结论，D、E、F 在 C 之后。所有阈值都在看到结果之前写进 `acceptance.json`，并记录它的哈希。

### A. 证据补齐：对照矩阵（约 1 周，优先级最高）

**A0：先修 slew 缺陷，再重建基线。** 在快仿真和 Gazebo 中，用 v3_2 和 es3 各跑 32 个种子。从这里开始，所有新结果都以修复后的版本为准。

**A1：建立对照矩阵。** 除了 C0 和 C5，每一臂都用**与 es3 相同的 CMA-ES 预算**（相同代数、种群、训练笼子分布和留出集）重新优化，然后在 Gazebo cage20 上用同样 32 个种子配对评估：

| 臂 | 内容 | 回答的问题 |
|---|---|---|
| C0 | es3（修复 slew 后重新优化） | 基准 |
| C1 关闭大脑 | DN 读出恒为基线，只保留状态机和 Governor，同预算优化状态机参数 | 去掉大脑，状态机和安全层自己能做到什么程度 |
| C2 打乱连线 ×5 | 每类投射在类内置换 post，保留每条边的符号和权重以及出入度；另加 1 组全局置换。每个实例各自重新优化 | 连线结构是否重要（参照 ClutchMedia，但实例数从 3 增加到 5） |
| C3 随机读出 | 用同样数量的非 DN 神经元替代读出组 | 读出的解剖身份是否重要 |
| **C4 直连对照** | EcpsRetina 的 looming、光流、blank、near 特征直接线性接到同一套状态机，参数量对齐，同预算优化 | **最关键**：MiniFly 有没有比直接用视觉特征更好 |
| C5 感觉消融 | 相机全黑、画面冻结、去掉 looming 通道（只做评估，不重新优化） | 行为是否依赖视觉 |

**判读规则（预注册）：**

- C0 必须在接触和险情上**显著优于** C1 和 C3。
- C0 对 C4 只要求**不显著更差**。即便打平也照实报告：参照 FLYNN 的文献结论，连接组拓扑的优势往往在分布外条件下才会显现，所以 C4 还要进入 D 阶段的分布外测试再比较。
- C2 采用位次检验：C0 是否优于全部 5 个打乱实例。

**A2：统计方法。**

- 二元指标（本回合是否接触）用 McNemar 检验；并参照 action_saved 的思路，同时报告"救回了几个回合"和"新引入了几个接触回合"。
- 连续指标用配对 bootstrap，给出**差值**的 95% 置信区间。
- 多个指标一起检验时做 Holm 校正。
- 打乱实例之间的方差单独作为一个层级报告。

**A3：两项便宜的防护。**

- **训练前的感觉门控**，参照 garyb9：在快仿真里给左右侧 looming 刺激，DNp03 和 DNp01 必须可分；静默 Retina 后输出必须与暗场一致，否则拒绝启动 CMA-ES。
- **语义冒烟测试**，参照 huang12chen：左侧 looming 必须引起向右扫视，静态纹理不得触发扫视，blank 必须进入谨慎状态。为每项设置放电率门槛，CI 中带上快仿真结果的哈希锚点。

### B. 指标拆分与覆盖率恢复（约 1 周，可与 A 并行）

**B1：给评估加上触发器层面的指标。** 用 Gazebo 的真值定义"有效威胁"：沿当前速度方向 TTC < τ 且距离 < d，τ 和 d 预先登记。在此基础上报告：

- 扫视召回率、每分钟虚警次数、从 looming 起始到扫视指令的潜伏期、方向正确率；
- 空场扫视率（在没有障碍物的世界里测）；
- 链式触发率（扫视后 2 s 内再次扫视的比例）；
- 慢速比例；
- Governor 和围栏转向的介入时间；
- 同等平均速度下的接触率。

先用这些指标诊断 es3，把覆盖率损失定位到具体的触发器行为。

**B2：多目标优化。** 先在适应度里改两项：

- 覆盖奖励改为"首次进入新网格"计分，参照 garyb9 的 +0.5/格；
- 接近障碍的惩罚改为势函数差 `γΦ(d′) − Φ(d)`，`Φ = exp(−(d/σ)²)`。

然后用 3–4 个不同的覆盖率权重各跑一次 CMA-ES（加权标量化），或者直接用 MO-CMA-ES 或 NSGA-II，得到"接触对覆盖"的 Pareto 前沿。你从中选一个工作点，比如"接触率不超过 10% 的前提下覆盖率最高"。

**B3：把 Governor 介入计入代价。** 参照子任务 06 的建议：如果介入率很高，说明行为是靠安全层兜出来的，这会削弱"行为来自 MiniFly"的论证。

### C. 扩展搜索空间与优化协议（约 1–2 周）

**C1：把 efference copy 参数纳入搜索。** 在 EcpsRetina 的 looming 输入上加一个随速度和转角升高的门限：`floor = f0 + a·v_fwd + b·|Δψ|`，带衰减；另加扫视后的屏蔽时间 T_hold。参数 a、b、T_hold 加入 CMA-ES。这一项同时针对两个问题：上真机后的伪 looming（Parth 的教训），以及 B1 中可能暴露出的空场误扫视。

**C2：通路增益和泄漏。** 参照 fly-self-driving 的有界参数化 `w = w0·(a + (b−a)·σ(θ))`：符号固定，连接不增不减。为 MiniFly 的每一类投射设一个增益（约 15–30 个），为关键细胞类型各设一个泄漏（约 10 个）。总维度控制在 60 以内，CMA-ES 仍可处理。同时加一道 **scan_gain 式的稳定性门槛**，候选不通过就直接给惩罚：撤掉刺激后放电要回落，HS/VS→DNg02 的方向要正确，looming 要能触发 DNp01。增益放开以后，必须用 A 阶段的 C2/C4 对照重新检验。

**C3：读出层的闭式求解（可选）。** 参照 haltere 的父中心岭回归：在快仿真里用 DAgger 收集"DN 发放率特征 + 带特权几何信息的避障教师指令"，闭式求解读出权重，加平滑惩罚，并用审计代码确认只有读出层被改动。CMA-ES 只保留不可微的阈值和时长参数。要注意 haltere 的经验：学生最多接近教师，不会超过教师。所以这一项适合用来降低维度、给 CMA-ES 一个更好的初始点，而不能作为"MiniFly 有独特贡献"的证据。

**C4：优化协议（照搬 flyjump 和 haltere）。**

- 每代所有候选共用同一批笼子（现有做法）；
- 当前冠军固定作为候选 0 回注评估；
- 只有在留出集上严格变好才替换冠军；
- 记录收敛平台，平台之后提前停止；
- **每个配置用 3–5 个 CMA-ES 种子复跑**，报告分布；
- cage20 继续完全留出；程序化世界里一旦有某一类被用于调参，就降级为开发数据。

**C5：快仿真提速。** 参照 fly-self-driving 的锁步批量评估：把 N 个回合的 LIF 状态堆成 `[neurons, batch]` 一起推进，撞墙的个体移出活跃集合。numpy 向量化的收益很可能超过多开进程，C2 的通路增益搜索也正需要更多评估量。

### D. 泛化与分布外测试（约 1 周）

- **更多 Gazebo 世界**：cage10、lowbox_s0–s4、wall_plain、wall_offset，以及光流导航模式（NAV=flow），这些在服务器上都已经可以运行。
- **ghost 条件**：参照 garyb9，去掉障碍物的 visual、保留 collision，检验躲避是否真的来自"看见"。
- **分布外条件**：失纹理墙、遮住相机的一半、低照度（降低相机增益）。按 FLYNN 的结论，C0 对 C4 的优势如果存在，最可能在这里显现。
- **haltere 式影子基线**：让手写的反射控制器负责控制，MiniFly 照常运行但只记录、不输出，用来分辨失败主要来自感知还是决策。
- **门槛预注册**：每个世界都写明接触率上限（要求 95% 置信区间的上界也满足）、覆盖率不低于 v3_2 的 X%、空场扫视率上限。

### E. 蘑菇体可塑性（生物学模块，约 1–2 周，可选）

结构按 AleksiAero 的模板，但做三处改造：

- **KC 层**：新增 200–500 个 KC，用随机稀疏连接（每个 KC 接 4–7 个输入）加全局抑制，把活跃比例校准到 3–10%。输入是 EcpsRetina 的方位分箱特征，加适应性 z-score。
- **学习规则**：照搬 AleksiAero，即资格迹 τ≈2 s，相位性多巴胺 = 实际值 − 基线 − 阈值，g∈[0,1] 只做乘性抑制并以 τ≈900 s 恢复，每 50 ms 写回一次，正好与本项目 20 Hz 的回路节拍对齐。
- **教学信号**：PPL1 来自接触（1 s）、险情、Governor 介入或围栏转向（各 0.3 s）；PAM 来自"进入新的覆盖网格"，直接针对覆盖率损失。

**不要加任何手写的趋向项**，避免 AleksiAero 那种混淆。实验设计是学习开 / 学习关、相同种子、多生命周期在快仿真里训练，然后冻结 g，在 Gazebo 上做 32 种子的配对比较。真机上只加载冻结后的 g，在线学习需要另做安全评审。

### F. 安全层修补与分级上真机

**F1：安全层修补。**

- 修 slew 缺陷（A0）。
- 大脑看门狗从 0.5 s 收紧到 100 ms（2 个 20 Hz 周期），超时后发零速度。
- GUID_TIMEOUT 在真机配置 `fhb_delta.parm` 里已经设为 1 s，保留。注意它只在光流导航模式下加载，GPS 模式的仿真用的仍是默认的 3 s。
- ARM 只能通过遥控器或板上按钮。
- 命令通道加会话 token 和递增序号，参照 kistik1。
- 失联后切到 LOITER 或 FLOWHOLD 悬停，**不切 LAND**（纯光流定位时 LAND 的水平保持质量取决于光流）。
- 软件**不自动 disarm**，disarm 只绑定在遥控器开关上。
- 在 SITL 里实际发送 +yaw_rate 和 +vy，确认机体系符号（abbosaliboev 的教训）；每次改动机体系或解码器之后都要重新确认。
- 起飞后加"门控 + 缓升"：高度误差 <0.1 m 且 ToF 有效之后，用 1–2 s 线性打开解码器输出（tiago369 的做法）。

**F2：分级验收**（结合 Parth、kistik1、haltere 的做法）：

| 级别 | 内容 | 预注册门槛 |
|---|---|---|
| L0 无硬件 | 在 Orange Pi 上回放 Gazebo 录制的鱼眼帧，跑完整回路；用 mock MAVLink 跑真实主循环。**每次上机前必跑** | 单步 p99 < 25 ms；输出与 Jetson 逐帧一致（≤1e-4） |
| L1 拆桨台架 | 不起飞；手持障碍物逼近，转动机体标定光流 | 静止 60 s 误触发为 0；逼近时 5/5 触发；光流标定 R² ≥ 0.5，否则禁飞 |
| L2 自稳悬停 | ALT_HOLD 或 FLOWHOLD 下验证 3901-L0X 的数据质量，大脑只记录 | 30 s 悬停漂移 < 0.3 m |
| L3 单通道闭环 | GUIDED 模式，前进速度为 0，只放行扫视的 yaw（限幅 0.3 rad/s），60 s 自动结束，遥控器一键切 LOITER | 空场扫视率 ≈ 0；链式触发率 < 10% |
| L4 低速巡航 | 前进速度 ≤ 0.3 m/s，围栏离网 1 m，逐步放开到仿真速度 | 在 N 种布局 × M 次中 0 接触，覆盖率 ≥ Gazebo 的 X% |

每一级都要做**飞前卡片**（记录预测、代码与参数的哈希）和飞后人工复核；保留每一次尝试的记录；阈值在飞之前写入配置并冻结，失败了就如实记录，不在事后调整阈值。

### 不做的事

- **不做高维 RL**：不学编码器、不做联合训练、不用 PPO/SAC 训练大读出。garyb9 和 fly-craftax 已经证明，这条路在不可微的连接组上会塌缩，或者学成一个"时钟"。如果以后要用 RL，也只做残差，并保留锚定。
- **不把 FlyWire 全脑的结论直接套到 MiniFly 上**，比如"转向 DN 是冗余的"。MiniFly 是手工连接的，结构不同。
- **不去掉神经元的正负号**（fly-self-driving 用的是全正权重）。MiniFly 的 PVLP 侧抑制依赖负权重。
- **不引用 tiago369 的"haltere 优于 PID"**，两者增益逐项相同，这个差异不能说明问题。
- **不把"关闭大脑"当作证据**；不在同一个验证集上既选参又报告结果。

---

## 6. 算力分配与时间线

| 阶段 | 快仿真优化 | Gazebo 评估 | 估计时间 |
|---|---|---|---|
| A0 修 slew，重建基线 | Jetson：两个版本各 64 种子，约 30 分钟 | 服务器：2 × 32 次，约 1 小时 | 半天 |
| A1 对照矩阵 | 需要 8 次 CMA-ES 重新优化（C1、C2×5、C3、C4）。Jetson 每次约 6 小时；如果在服务器上并行跑快仿真，约 1.5 小时一次 | 服务器：约 (5 + 5 + 3) × 32 ≈ 400 次，约 6 小时 | 2–4 天 |
| A4 es3 多种子复跑 | 4 次 CMA-ES | 4 × 32 次 | 1–2 天 |
| B 多目标 | 3–4 次 CMA-ES | Pareto 点 × 32 次 | 2–3 天 |
| C 扩展搜索空间 | 先做批量化改造；增益 + 泄漏搜索 2–3 次 | 抽样复核 | 1–2 周 |
| D 泛化 | — | 6–8 个世界 × 2–3 臂 × 32 次 | 3–4 天 |
| E 蘑菇体 | 多生命周期训练 | 学习开 / 关 × 32 次 | 1–2 周 |
| F 上真机 | — | L0 在 Orange Pi 上 | 随硬件进度 |

A 阶段的 CMA-ES 重新优化可以放到服务器的快仿真上跑：服务器有 112 核，numpy 的快仿真不需要 Gazebo。这需要先验证服务器上的快仿真和 Jetson 逐位一致，可能受 numpy 版本差异影响。ROG 继续留给另一个会话。

---

## 7. 待确认事项

**没有读到原文、需要谨慎引用的细节：**

- garyb9：`sac.py`（91 KB）的实现细节；文档写每步 40 个物理步，代码是 8 次 `advance`。
- haltere：fakefit 和 reservoir 的定量结果；门槛 harness 在其他分支上，没有读到。
- fly-craftax：数据来自单次运行，作者明言在当前 commit 上无法复现原数字。
- AleksiAero：`scan_gain` 没有提交扫描输出表。
- kistik1：mock bridge 的实现没有核实。
- ArduPilot GUIDED 的速度超时行为和 FLOWHOLD 行为，需要在 SITL 里确认。

**本项目需要实测的：**

- slew 修复后，v3_2 和 es3 的指标变化有多大；
- 服务器上的快仿真是否和 Jetson 逐位一致；
- es3 的空场扫视率和链式触发率；
- Orange Pi 上完整回路的单步 p99 耗时。

---

## 附录：主要来源

- garyb9/fly-drone：[training.md](https://github.com/garyb9/fly-drone/blob/main/docs/training.md) · [env.py](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/env.py) · [assay.py](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/assay.py) · [validation.md](https://github.com/garyb9/fly-drone/blob/main/docs/validation.md) · [roam_eval.py](https://github.com/garyb9/fly-drone/blob/main/python/fly_drone/roam_eval.py) · [DEPRECATION](https://github.com/garyb9/fly-drone/blob/main/docs/results/encoder-v6/DEPRECATION.md)
- skulitom/haltere：[fast_motor_tracking.py](https://github.com/skulitom/haltere/blob/main/haltere/train/fast_motor_tracking.py) · [model.py](https://github.com/skulitom/haltere/blob/main/haltere/brain/model.py) · [generalization_program.md](https://github.com/skulitom/haltere/blob/main/docs/generalization_program.md) · [flight card 09-23](https://github.com/skulitom/haltere/blob/main/docs/flight_cards/2026-09-23_matched_full_races.md)
- fly-craftax / flyjump：[ppo.py](https://github.com/liuzihe02/fly-craftax/blob/main/flycraftax/ppo.py) · [eval_ppo_controls.py](https://github.com/liuzihe02/fly-craftax/blob/main/scripts/eval_ppo_controls.py) · [tracker.md](https://github.com/liuzihe02/fly-craftax/blob/main/tracker.md) · [training.ts](https://github.com/cobanov/flyjump/blob/main/src/lib/training.ts) · [experiment.md](https://github.com/cobanov/flyjump/blob/main/docs/experiment.md)
- fly-self-driving / AleksiAero：[connectome.py](https://github.com/suanmiao/fly-self-driving/blob/main/flyhard-patches/connectome.py) · [results.md](https://github.com/suanmiao/fly-self-driving/blob/main/docs/results.md) · [scan_gain.py](https://github.com/AleksiAero/Aleksi-Aero-Fly-Brain-Drone/blob/main/scripts/scan_gain.py) · [mushroom_body.py](https://github.com/AleksiAero/Aleksi-Aero-Fly-Brain-Drone/blob/main/ros2_ws/src/fly_drone_sim/fly_drone_sim/mushroom_body.py)
- 评估方法学：[flybrain_step.py](https://github.com/ClutchMedia775/fly-brain-drone/blob/main/sim/flybrain_step.py) · [phase56_summary.json](https://github.com/ClutchMedia775/fly-brain-drone/blob/main/results/phase56_summary.json) · [evaluate.js](https://github.com/huang12chen/flywire-brain-drone/blob/main/web/evaluate.js) · [jitter_sim_log.md](https://github.com/huang12chen/flywire-brain-drone/blob/main/web/jitter_sim_log.md) · [scientific_assumptions.md](https://github.com/abbosaliboev/fly-brain-drone/blob/main/docs/scientific_assumptions.md)
- 安全与上真机：[safety_layer.py](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Controllers/safety_layer.py) · [flybrain_controller.py](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/NeuralPathways/flybrain_controller.py) · [Drone/README](https://github.com/Parth-Joshi0/Fly-Brain-Drone/blob/main/Drone/README.md) · [MAVIC_AIR.md](https://github.com/kistik1/FlyDrones/blob/main/docs/MAVIC_AIR.md) · [mavlink.py](https://github.com/kistik1/FlyDrones/blob/main/src/flydrones/drones/mavlink.py) · [controller_manager_m4.yaml](https://github.com/tiago369/fly_robot_control/blob/master/fly_controller/config/controller_manager_m4.yaml)

*过程文件（6 份子报告、原始代码摘录、日志）位于 `.research/20261006-flybrain-rl-routes-c983/`。*
