# V5 对 abs_fix 工具链的改动（全部在 /home/jacky/sim/val5 内的副本上，原目录只读未动）

## 脚本改动

| 文件 | 改动 |
|---|---|
| `abs_sim.py` | ① `sys.path` 指向 `/home/jacky/sim/val5`；② 新增故障注入开关（见下）；③ `comp.csv` 新增事件行 `block`、`obs_badyaw`；④ RESULT_JSON 增加 `inj`（注入计数）和 `t_auto`（AUTO 开始的飞控时钟） |
| `abs_analyze.py` | metrics.json 增加 `fixes_sent`、`gate_innov_rej`、`inj`、`auto_armed`、`n_obs_bad`、`n_obs_badyaw`、`n_block_rows`、`n_block_vis`（其余指标算法未改） |
| `run_one.sh` | `D` 指向 `/home/jacky/sim/val5`；实例号由外部传入，只用 20–23 |
| `batch.py` | `D` 指向 val5；并行实例号 `[20, 21, 22, 23]` |
| `collect.py` | `D` 指向 val5（`runs/*/metrics.json` → `results.csv`） |
| `tables.py` | 未用；V5 汇总另写 `tables_v5.py` |
| 新增 | `gen_v5.py`（任务清单）、`tables_v5.py`（汇总表）、`show.py`（按 tag 查某次运行） |

## 新增的故障注入开关（abs_sim.py）

| 开关 | 作用 | 实现位置 |
|---|---|---|
| `--bad-yaw t0:dur:deg` | t0 起 dur 秒内，所有观测的标记航向加 deg 度误差（位置不动） | `companion_step()` 里 `yaw_meas += radians(deg)`，观测标成 `obs_badyaw` |
| `--block t0:dur` | t0 起 dur 秒内所有标记不可见（遮挡） | `companion_step()` 里置 `vis=None`，每个观测周期记一行 `block`；本应可见的时刻也记，用来证明注入生效 |
| `--lat-switch t0:dur:lat` | t0 起 dur 秒内观测释放延迟改成 lat 秒（消息延迟） | `companion_step()` 里 `pending.append((t + lat, ...))` |
| `--comp-stop T` / `--comp-stop T:DUR` | T 秒后机载电脑停发全部消息（含 5 Hz 保活 ODOMETRY 和心跳）；给了 DUR 就在 T+DUR 恢复 | `fly()` 里 `comp["on"]=False/True`；`pump()` 里心跳随 `comp["on"]` 停发 |
| `--comp-stop-rc` | 附加开关：机载电脑停止时连 RC 覆盖（油门 1500）也停发 | `pump()` 里 RC 覆盖受 `comp["on"]` 控制 |

原有 `--badfix t0:dur:dx:dy` 照旧用（持续位置错检）。

**机载电脑中断的口径**：本仿真里机载电脑同时是唯一的 MAVLink 对端（sysid 254）。飞控的 GCS 失联判定
（`FS_GCS_ENABLE 5` → LAND）看的是这个 sysid 的"任何流量"，而 RC 覆盖消息也算流量（`GCS_Common.cpp:4224`）。
所以主口径下中断时**保留飞手 RC 覆盖**、停发机载电脑的心跳与修正消息：这是"机载电脑挂了"，不是"遥控链路挂了"。
`--comp-stop-rc` 才是"连遥控链路一起断"。

## 注入是否生效的核对办法

| 注入 | 核对依据 |
|---|---|
| 位置错检 | `comp.csv` 里 `ev=obs_bad` 的行数（= metrics 的 `n_obs_bad`）> 0；该行 `mx-my` 与真值 `tx-ty` 差 1 m |
| 航向错检 | `comp.csv` 里 `ev=obs_badyaw` 的行数（= `n_obs_badyaw`）> 0 |
| 遮挡 | `comp.csv` 里 `ev=block` 的行数（= `n_block_rows`）> 0，其中本应能看到标记的行数（= `n_block_vis`）> 0 |
| 中断 | `run.log` 里出现 `COMPANION STOPPED` / `COMPANION RESUMED`；中断期间 `n_sent` 不再增加 |
| 延迟 | `comp.csv` 的 `sent`/`obs` 行：释放时刻 `t` 减捕获时刻 `info` 在故障窗口内 ≈ 0.5 s（平时 0.1 s） |
| 门限拒绝 | `n_gated`（机载一致性门限拒掉的条数）、`gate_innov_rej`（v2 新息门限拒掉的条数） |

## 批处理中发现并修掉的一个注入缺陷（--comp-stop T:DUR）

第一轮批量跑完后核对计数发现：`--comp-stop 50:10`（断 10 s 再恢复）的 10 次运行里，
`inj.comp_stop` / `inj.comp_resume` 都是 163–226 次，而不是 1 次。原因是 `fly()` 里的状态机：
恢复之后 `comp["on"]` 又变回 True，下一轮循环立刻又满足“停发”条件，于是机载电脑在恢复之后
以每轮（≈0.1 s）一次的频率反复停/发，等于“断 10 s + 之后一直半通不通”，不是“断 10 s 然后一直恢复”。

修法（`abs_sim.py` 的 `fly()`）：加 `cs_done = {"stop": False, "resume": False}`，停发和恢复各只允许发生一次。

- 受影响的运行：全部 10 次 `*_stop10_*`（sq4 5 次、sq1 5 次）。
- 已删除旧运行目录并用修好后的脚本重跑这 10 次，结果以重跑为准（见 report.md §4）。
- 不受影响的：`--comp-stop T`（不恢复）本来就是只停一次，计数为 1，没有这个缺陷。
- 重跑后核对：`inj.comp_stop = 1`、`inj.comp_resume = 1`（`sq4_stop10_r1`、`sq1_stop10_r1`），
  中断期间 `n_sent` 不增加，恢复后继续增加。
