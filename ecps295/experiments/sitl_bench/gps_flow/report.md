# 加装 GPS + 罗盘后的定位架构：SITL 研究报告（2026-11-19，ROG，实例 I20–I23）

ArduCopter 4.7.0 SITL（未改源码、未重新编译），在 `~/sim/mtf01/mtf01_sitl_rng.parm`（MTF-01 + `EK3_SRC1_POSZ 2`）上叠加 GPS / 罗盘 / 源组配置。
有效运行 84 次（28 个配置 × 3），误差全部以 SITL 真值（dataflash `SIM2` 位置、`SIM` 航向）为准。
任务：AUTO，Takeoff → 2 m 高、2 m 边长正方形（每个角悬停 8 s）→ LAND，与 mtf01 的 2 m/2 m 任务相同，只是悬停时间从 2 s 改成 8 s，用来量悬停漂移。

## 一、结论

**推荐架构 (i)，用遥控三段开关切换，不用 Lua。**
- 源组 1（开关低位）：GPS 位置 + GPS 速度 + 罗盘 + 测距高度。
- 源组 2、3（中位、高位）：光流 + 罗盘 + 测距高度。
- `EK3_SRC_OPTIONS 1` 保持现值。
- 完整参数见 `gps_flow_recommended.parm`。

**在 6 m 网笼里飞 2 m 任务，用源组 2（光流）。** GPS 只用来给原点和 Home，室外开阔场地才用源组 1。

| 问题 | 结论（中位数 [范围]，n=3） |
|---|---|
| 1 架构 | (i) 光流组任务跟踪 RMS 0.079 m（IF_poor），GPS 组 0.38 m（好 GPS）/ 1.06 m（差 GPS）。(ii)「GPS 位置 + 光流速度」在差 GPS 下为 1.07 m（II_poor），和纯 GPS 一样差：只要 GPS 位置被融合，误差就跟着 GPS 走。(iii) 官方 `ahrs-source-gps-optflow.lua` 是 (i) 加自动切换，**MatekF405-TE 固件不含脚本**（见第 1 节），所以只能手动拨开关。 |
| 2 罗盘 | 罗盘正常时，光流跟踪 0.054 [0.045–0.067] m，无罗盘 0.062 [0.045–0.097] m：**没有可分辨的改善**。固定偏 10° / 20°：0.085 / 0.098 m，略差。**随油门变化**的干扰最伤：航向变化约 11° 时 0.174 m，约 35° 时 0.587 m（最大 1.84 m）。判据：飞行中航向误差变化超过约 5–10° 时，用罗盘不如不用。GSF 偏航（`EK3_SRC1_YAW 8`、无罗盘）在 GPS 组 0/3 能解锁（`Arm: Need Position Estimate`），起飞前不可用。 |
| 3 笼内 GPS | 我注入的 GPS 位置误差是 σ0.5 m / σ1.5 m 的慢变随机游走。GPS 组任务 RMS 0.38 / 1.06 m，最大 0.89 / 2.03 m（最大一次 2.87 m）。光流组 0.06–0.08 m。GPS 跳变 3 m、持续 10 s：飞机跟着跑，真值最大半径 3.6–5.6 m，1/3 触发围栏 LAND。改 `EK3_GLITCH_RAD 25` 或 `EK3_POS_I_GATE 300` 都没有区别。GPS 丢失：`EK3_SRC_OPTIONS 0` 时 0/3 完成，约 15 s 后 EKF failsafe LAND，落点偏 9.4–12.2 m；`EK3_SRC_OPTIONS 1` 时 3/3 完成，自动靠光流速度继续飞。 |
| 4 切换 | GPS→光流：EKF 位置跳 0.11 [0.08–0.14] m，无告警，平稳。光流→GPS：EKF 直接把位置重置到 GPS，跳变等于两者之差。自然差（约 1 m）时跳 1.1 m；人为加 1 m 时跳 1.95 m；加 3 m 时跳 3.7 m。飞机随后以约 0.5 m/s（受 `WP_SPD` 限制）实际移动 1.7–3.7 m。偏 3 m 时 2/3 触发 `Circle fence breached` → LAND，另 1/3 真值半径到 6.0 m。高度不受影响（0.02–0.04 m），也没有 EKF 告警。 |
| 5 原点/解锁 | 有 GPS 时，EKF 自己用 GPS 设原点，Home 在解锁时设。室内无星时机载电脑**仍必须发原点**：不发则 3/3 `Arm: Fence requires position`，发了 3/3 能解锁并完成任务。机载电脑在有 GPS 时继续发原点（偏 500 m）也没有造成冲突（0.40 vs 0.39 m）。`VISION_POSITION_ESTIMATE` 已经不再使用，`VISO_TYPE 0`。高度在 GPS 组里仍由测距决定（alt_rms 0.008–0.014 m），不建议改用 GPS 或气压高度。 |

最不确定的：
- 真实网笼里 M10 的位置误差有多大、怎么变化。我的 σ0.5 / 1.5 m 是假设。SITL GPS 没有水平噪声，我是自己用 `SIM_GPS1_GLTCH` 注入的。
- 这台小机上罗盘受电机干扰到底多大，只能实测。

## 二、问题逐项

### 1. 架构

| 架构 | 参数要点 | SITL 表现 | 判断 |
|---|---|---|---|
| (i) 组1 GPS / 组2 光流，RC 开关 | `EK3_SRC1_POSXY 3, VELXY 3, VELZ 0, POSZ 2, YAW 1`<br>`EK3_SRC2_POSXY 0, VELXY 5, VELZ 0, POSZ 2, YAW 1`<br>`EK3_SRC3_*` 同组2<br>`RC8_OPTION 90`、`GPS1_TYPE 1`、`COMPASS_ENABLE 1`、`ARMING_SKIPCHK 4104` | 光流组 0.079 m，GPS 组 0.38 / 1.06 m。上电即按开关位置选组（`Using EKF Source Set 2`） | **推荐** |
| (i) + `EK3_SRC_OPTIONS 1` | 同上 | GPS 组丢星后自动靠光流（IGfa_loss 3/3 vs IG_loss 0/3）。差 GPS 下 GPS 组 1.01 m vs 1.06 m，无差别。光流组 0.112 [0.064–0.136] m vs 0.079 [0.077–0.118] m，在离散度内 | 保持现值 1 |
| (ii) 光流速度 + GPS 位置，单组 | `EK3_SRC1_POSXY 3, VELXY 5, POSZ 2, YAW 1`，`EK3_SRC_OPTIONS 0` | 差 GPS 1.07 m（II_poor）。GPS 丢失 3/3 完成，0.44 m（II_loss） | 不推荐：GPS 位置误差原样进来，在笼里没有好处 |
| (iii) 官方 Lua 自动切换 | (i) 加 `SCR_ENABLE 1`、`RC9_OPTION 300`、`SCR_USER1..4` | 未仿真 | **该板不可用**：`MatekF405-TE/hwdef.dat` 中 `FLASH_SIZE_KB 1024`，且包含 `minimize_fpv_osd.inc`；`AP_Scripting_config.h` 规定 `AP_SCRIPTING_ENABLED = (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)`，所以固件里没有脚本。F405 只有 192 KB RAM，脚本堆也不够。 |

切换源码依据：`AP_NavEKF3_PosVelFusion.cpp` 第 546–626 行。水平位置源变成 GPS 时会 `ResetPositionNE` 到 GPS；变成 NONE（光流）时不重置。这正好解释了第 4 节两个方向的不对称。

### 2. 罗盘对光流定位的影响（室内配置，无 GPS，机载电脑发原点）

| 配置 | 航向误差（最大 / 飞行中变化，°） | track_rms | track_max | 悬停 RMS 到航点 | land_offset |
|---|---|---|---|---|---|
| F0 无罗盘（= mtf01 q2_RNG 配置） | 无意义 / 2.8 [2.3–13.4] | 0.062 [0.045–0.097] | 0.129 [0.118–0.213] | 0.077 [0.063–0.126] | 0.092 [0.048–0.175] |
| FM 罗盘正常 | 1.4 / 0.5 | 0.054 [0.045–0.067] | 0.132 [0.130–0.151] | 0.081 [0.073–0.086] | 0.143 [0.134–0.160] |
| FMdec10 恒偏 10° | 10.7 / 0.7 | 0.085 [0.070–0.119] | 0.174 [0.136–0.223] | 0.116 [0.104–0.179] | 0.125 |
| FMdec20 恒偏 20° | 20.8 / 0.7 | 0.098 [0.054–0.101] | 0.189 [0.114–0.227] | 0.133 [0.084–0.139] | 0.197 |
| FMmot2 电机干扰 `SIM_MAG_MOT_Y 2` | 12.2 / 10.9 [7.6–14.8] | **0.174** [0.151–0.202] | 0.331 | 0.288 | 0.277 |
| FMmot6 `SIM_MAG_MOT_Y 6` | 36.3 / 35 [23–44.6] | **0.587** [0.33–1.05] | 0.86 [0.55–1.84] | 1.03 | 1.51 [0.30–1.56] |

- 罗盘正常时没有可分辨的改善，所以**不能用“光流精度变好”作为加罗盘的理由**。加罗盘的理由是 GPS 组需要偏航。
- 恒定偏差 10–20° 影响很小。光流在机体系，EKF 在一个整体转过去的坐标系里自洽，任务也按 EKF 航向放置。
- 随油门变化的偏差是主要危险，FMmot6 时出现 `ground mag anomaly, yaw re-aligned`。
- **判据**：带桨悬停、改变油门时，航向变化在 5° 以内就用罗盘（`EK3_SRC2_YAW 1`）。超过约 10°，光流组就改为 `EK3_SRC2_YAW 0`，此时依据是 FMmot2 比 F0 差 2.8 倍、范围不重叠。这种情况下 GPS 组也不该用。
- 不用罗盘时 GPS 组的偏航：GSF（`EK3_SRC1_YAW 8`，`COMPASS_ENABLE 0`）3/3 都是 `Arm: Need Position Estimate`。GSF 要靠运动中的 GPS 速度收敛，地面静止时没有偏航，LOITER / AUTO 起不来。只能先在 ALT_HOLD 起飞、平飞一段再切模式。在 6 m 网笼里这样做不现实，**不推荐**。
- F0 的 alt_rms 0.022 m 比 FM 的 0.010 m 大，原因没有查。差值只有 1 cm，不影响结论。

### 3. GPS 质量差（源组 1，任务中）

可用的 `SIM_GPS1_*` 参数（以 `libraries/SITL/SIM_GPS.cpp` 为准）：
- `NUMSATS`：只改报告的星数。
- `ACC`：只改报告的精度（h_acc），不加误差。
- `NOISE`、`DRFTALT`、`ALT_OFS`：**只影响高度**。
- `VERR`：速度均匀噪声。
- `GLTCH_X/Y/Z`：经纬度偏移，单位是度。
- `JAM`：干扰模拟，位置会乱跳约 200 m。
- `LAG_MS`、`HZ`、`BYTELOS`、`LCKTIME`。
- **没有水平位置噪声，HDOP 固定 1.21**（`SIM_GPS_UBLOX.cpp`），所以 `GPS_HDOP_GOOD` / `EK3_CHECK_SCALE` 没法测。
- 水平误差是我在脚本里每 0.5 s 写一次 `SIM_GPS1_GLTCH_X/Y` 注入的：一阶 Gauss-Markov，τ = 20 s，σ 取 0.5 m（好）/ 1.5 m（差）。差 GPS 再加 `NUMSATS 6`、`ACC 2`、`VERR 0.3`。

| 配置 | track_rms | track_max | 悬停 RMS 到航点 | pos_est_err_max | land_offset | 完成 |
|---|---|---|---|---|---|---|
| IG_good（GPS 组，σ0.5） | 0.383 [0.375–0.541] | 0.889 [0.887–1.34] | 0.607 | 1.16 | 0.718 [0.34–1.14] | 3/3 |
| IG_poor（GPS 组，σ1.5） | 1.06 [1.03–1.56] | 2.03 [1.80–2.87] | 1.86 | 2.56 | 1.91 [1.11–2.64] | 3/3 |
| IF_poor（光流组，同样差 GPS 在场） | **0.079** [0.077–0.118] | 0.179 [0.167–0.257] | 0.133 | 0.20 | 0.153 | 3/3 |
| IG_step3（好 GPS + 3 m 跳变 10 s，`GLITCH_RAD 0`，`POS_I_GATE 500`） | 1.21 [0.54–1.73] | 3.10 [1.18–3.64] | 1.49 | 3.21 | 1.32 [0.69–3.78] | 2/3（1 次 Circle fence → LAND），真值半径 3.6–5.6 m |
| IG_step3_gr25（`EK3_GLITCH_RAD 25`） | 1.20 | 3.26 | 1.48 | 3.19 | 1.37 | 2/3 |
| IG_step3_pg300（`EK3_POS_I_GATE 300`） | 1.19 | 3.22 | 1.45 | 3.19 | 1.36 | 2/3 |
| IG_loss（AUTO 中关 GPS，`SRC_OPTIONS 0`） | 4.57 [3.59–4.82] | 10.4 [9.3–11.6] | – | 12.8 | **10.7 [9.4–12.2]** | 0/3：15 s 后 `stopped aiding` → `EKF variance` → LAND |
| IGfa_loss（同上，`SRC_OPTIONS 1`） | 0.376 [0.368–0.519] | 0.808 | 0.501 | 0.79 | 0.438 | 3/3，无告警 |
| II_loss（架构 ii，丢 GPS） | 0.438 | 0.82 | 0.541 | 0.95 | 0.602 | 3/3 |

- GPS 组的误差基本等于注入的 GPS 误差，几乎不受 EKF 参数影响。光流组的误差和 GPS 好坏无关。
- **笼内判断**：6 m 见方的网笼，从中心起飞，到网只有 3 m。2 m 正方形从中心角点出发时，航迹本身已经离网约 1 m 以内。GPS 组即使在 σ0.5 m 下最大偏离也有 0.9–1.3 m，σ1.5 m 时 1.8–2.9 m，3 m 跳变时 3.6–5.6 m。**笼内一律用源组 2（光流）**。
- `EK3_GLITCH_RAD`、`EK3_POS_I_GATE`：三种设置下 3 m 跳变的结果相同（差别远小于重复间离散度），**不推荐改**。`EK3_CHECK_SCALE`、`GPS_HDOP_GOOD` 无法在 SITL 验证，保持不变。`FS_EKF_ACTION 1` 保持。
- 能否自动退到光流：`EK3_SRC_OPTIONS 1` 时可以，因为 GPS 组里光流速度一直在融合，见 IGfa_loss。但它**不会换源组**，EKF 仍然认为源组 1 有效。GPS 恢复后会重新拉回 GPS 位置，这一步没测。不用 Lua 就没有按 GPS 精度自动换组的办法。

### 4. 切换过程（架构 i，σ0.5 GPS，开关通过 RC8 override）

AUTO 中，在 WP2 悬停时（AUTO 开始后约 15 s）切换。“偏移”指切换前 10 s 给 GPS 加的阶跃，这时 GPS 不在使用中，所以不影响飞行。

| 配置 | EKF 位置跳变 (m) | 切换后 10 s 真值位移 (m) | 最大水平速度 (m/s) | 高度变化 (m) | 告警 / 结果 |
|---|---|---|---|---|---|
| SW_G2F（GPS→光流） | 0.108 [0.083–0.136] | 0.95 [0.64–1.39]（属于正常航线移动） | 0.41 | 0.024 | 无，3/3 完成 |
| SW_F2G（光流→GPS，自然差） | 1.10 [0.52–1.35] | 2.21 [1.21–2.58] | 0.53 | 0.033 | 无，3/3 完成 |
| SW_F2G_d1（+1 m） | 1.95 [1.44–2.20] | 2.45 [1.89–3.32] | 0.53 | 0.018 | 无，3/3 完成 |
| SW_F2G_d3（+3 m） | 3.69 [3.35–4.13] | 1.74 [1.67–3.70] | 0.54 | 2.0（LAND） | 2/3 `Circle fence breached` → LAND；1/3 完成但真值半径 6.0 m |
| SW_hover（LOITER 悬停，光流→GPS→光流） | 0.83 [0.64–1.54] / 0.08 [0.06–0.16] | 0.41 / 0.22 | 0.19 / 0.13 | 0.27–0.41（见下） | 无 |

- 两个方向都不触发 EKF 告警，也不触发失控保护。速度被 `WP_SPD 0.4` / `LOIT_SPEED_MS 1` 限制住，所以不是一下子冲过去，而是以约 0.5 m/s 走过去。
- **在笼内，光流→GPS 方向必须看作“飞机会平移到 GPS 认为的位置”**。偏移 3 m 就会撞网或触发围栏。
- SW_hover 的高度变化 0.27–0.41 m，可能是脚本模拟的推杆停得晚造成的（mtf01 报告里也有同样现象），不能归因于切换。

### 5. 原点、Home、围栏、解锁、高度

| 配置 | 结果 |
|---|---|
| 室外 GPS，机载电脑不发原点（IG_good） | 约 20 s 后 `EKF3 IMU0 origin set`（由 GPS 设），约 41 s 后 `is using GPS`。解锁时设 Home，围栏以 Home 为圆心 |
| IG_comp0（机载电脑照发原点 + VISION） | 0.389 m，与 IG_good 的 0.383 m 相同 |
| IG_comp500（机载电脑发偏 500 m 的原点） | 机载电脑的原点先被采用，GPS 位置在其坐标系里约 500 m，Home / 任务 / 围栏都按 Home，0.402 m，无冲突 |
| Q5_in_noorg（室内无星，参数同室外，开关中位，不发原点） | 3/3 `Arm: Fence requires position`，无法解锁 |
| Q5_in_org（同上，机载电脑发原点） | 3/3 解锁并完成，0.099 [0.088–0.109] m |

- 机载电脑：室内必须继续发 `SET_GPS_GLOBAL_ORIGIN`。室外可以不发，建议改成“开机后没有 3D fix 才发”，免得本地坐标系离真实场地很远。
- `VISION_POSITION_ESTIMATE` 在 `EK3_SRC*_POSZ 2` 下不再被使用，`VISO_TYPE 0`，机载电脑停发即可。SITL 里两者没有冲突。
- 解锁：保持 `ARMING_SKIPCHK 4104`（跳过 GPS lock 和 GPS config 两项），室内才能解锁。GPS 组在无 fix 时不会被这一项放行，因为还有 `Need Position Estimate`（GSF 运行即是例子）和围栏的“需要位置”检查兜底。罗盘检查（bit 2）不跳过。
- `ARMING_SKIPCHK` 的位定义来自 `AP_Arming.cpp` 第 198 行：3 = GPS lock，12 = GPS Configuration。
- 围栏：`FENCE_RADIUS` 在两种模式下都以 Home 为圆心，用 EKF 位置判断。在 GPS 组里它只能防 EKF 认为的越界：GPS 偏 1–2 m 时，真值可能已经碰网而 EKF 不知道。光流→GPS 切换的位置重置会直接触发围栏（SW_F2G_d3）。`FENCE_ALT_MAX 2.5` 用测距高度判断，与 GPS 无关。
- 另一个现象（只见于烟雾测试 1 次，未重复）：差 GPS 下先飞一次 LOITER 落地，再解锁 AUTO 时报 `Arm: Vehicle breaching Circle fence`。原因是 GPS 漂移加上落点偏移使 EKF 位置离 Home 超过围栏。GPS 组在笼内可能连第二次解锁都过不了。
- 高度：所有 GPS 组运行 alt_rms 0.008–0.014 m，hgt_est_rms 0.013 m，与光流组相同。测距高度在 GPS 组里仍然合适。GPS 高度（SITL 无噪声，真机通常 ±1–3 m）和气压都不如测距，不用。代价是 mtf01 的测距故障结论原样适用（下跳 1.5 m 持续 5 s 会爬到 4.4 m）。

## 三、方法

- **工具链**：复制 mtf01 的 `mtf_sim.py`（改名 `gf_sim.py`）、`analyze.py`（改名 `gf_analyze.py`）、`run_one.sh`、`batch.py`，只改了以下几处：
  - 新增 `--ch8` / `--switch` / `--hover-switch`：在 RC8 上用 override 切源组，`RC8_OPTION 90`。
  - 新增 `--gps-walk`、`--gps-step`：GPS 水平误差。
  - 新增 `--no-origin` / `--origin-ofs` / `--no-vis`：控制机载电脑行为。
  - 新增 `--settle`、`--hold 8`。
  - 分析脚本把计划正方形按解锁时的**真航向**（`SIM.Yaw`）旋转，因为本次起始航向不再是 0。另外加了航向误差（`ATT.Yaw − SIM.Yaw`）、航点悬停 RMS（每个航点 `Reached` 前 7 s），以及切换事件（`RCIN.C8` 变化后 2 s 内 EKF−真值 的变化、10 s 内真值位移 / 速度 / 高度）。
- **参数**：`gen.py` 由 `base_rng.parm`（= mtf01_sitl_rng.parm）+ `FENCE_RADIUS 4`（mtf01 推荐）+ 配置覆盖层 + 每次重复的扰动，生成 `parms/<tag>.parm`。运行时再叠加 mtf01 的 FIELD_PARAMS（`WP_SPD 0.4`、`FENCE_MARGIN 0.3` 等）。
- **每次重复加的扰动**（r = 0/1/2）：
  - 起始航向 10° / 130° / 250°
  - 起飞点向北移 0 / 2.2 / 4.4 m
  - 风 1.5 / 3.0 / 2.0 m/s，来向 45° / 200° / 300°，`SIM_WIND_TURB 0.5`，`SIM_WIND_T 1`（风速不随高度衰减）
  - GPS 误差随机游走的种子 1 / 2 / 3
- **仿真设置**：speedup 5，实例 I20–I23，最多并行 4 个。SITL 中 GPS 状态显示为 6（RTK fixed），这是 SITL u-blox 的行为，不代表真机。
- 罗盘误差：
  - 恒偏：`EK3_MAG_EF_LIM 0` + `COMPASS_AUTODEC 0` + `COMPASS_DEC` = 0.197293 + 10° / 20°。0.197293 是真机在本场地的自动磁偏角。实测航向偏差 10.7° / 20.8°。
  - 电机干扰：`SIM_MAG_MOT_Y`（机体 y 向，单位 mGauss/A）2 / 6，`COMPASS_MOTCT 0`。

## 四、失败、无效、可疑、没做

- **自己造成的损失**：开批之前做了 2 次烟雾测试（IG_poor_r0、FMmot6_r0，旧设置：先 LOITER 再 AUTO，settle 12 s）。之后我用 `rm -rf` 删它们的目录时，正式批次的 IG_poor_r0 刚好用同名目录开跑，这次正式运行因此作废（`run.log: No such file`）。之后在 batch2 里重跑了 IG_poor_r0。烟雾测试的发现（settle 12 s 时 `Arm: Gyros inconsistent`；差 GPS 下第二次解锁被围栏拦）只作为观察，没有重复。
- 一开始 hold 窗口和切换跳变窗口算错了：前者把 `Reached` 当成到达时刻，后者只取了约 0.15 s。两处都已修正，并对全部运行重新分析，表中是修正后的数。
- GSF_good 0/3 和 Q5_in_noorg 0/3 是配置本身的结果，不是工具问题。
- SW_hover 的高度变化有疑问，见第 4 节。
- **没做**：
  - 罗盘差 + GPS 组（例如 FMmot6 配 GPS 组）。
  - GPS 恢复后的行为。
  - `SIM_GPS1_JAM`。
  - `LAG_MS`、`HZ` 的影响。
  - 不同的 `FS_EKF_THRESH`。
  - GPS 组下的围栏余量。
  - 任务中途手动 GSF。
  - 光流组在 `EK3_SRC2_YAW 0` 加 GPS 在场时的表现（F0 是无 GPS 版本）。
  - 光流→GPS 切换用的 GPS 偏移只有 1 m 和 3 m 两档，并且都是阶跃。
  - 用时约 70 分钟，超出 60 分钟预算。

## 五、SITL 回答不了的事

- **真实 GPS 在网笼里的误差**：多路径、遮挡、M10 的滤波，以及它们随时间的变化，都无法仿真。本报告的 GPS 组数字完全取决于我假设的 σ 和 τ。SITL 自带的 GPS 没有水平误差，HDOP 恒定，`GPS_HDOP_GOOD` / `EK3_CHECK_SCALE` / `AHRS_GPS_MINSATS` 的效果无从验证。
- **这台机的磁干扰**：大小、方向、和电流的关系，以及 QMC5883L 离电源线的距离，只能靠 compassmot 和带桨悬停实测。SITL 的 `SIM_MAG_MOT` 是线性模型。
- SERIAL3 上 M10 的实际配置、波特率、更新率，以及 QMC5883L 的 I2C 识别。
- 真实光流 / 测距质量（见 mtf01 报告）、机体动力学和 PID。SITL 的绝对数值只能拿来比较方案，**不能据此认为真机没问题**。
- RC 开关的实际通道和位置：SITL 里是 RC8 override，真机要按遥控器实际配置。

## 文件

- `report.md`（本文件）、`checklist.md`（上机验证清单）、`gps_flow_recommended.parm`
- `results.csv`（每次运行一行）、`summary.md`（每个配置的中位数 [范围]）
- 脚本：`gf_sim.py`、`gf_analyze.py`、`run_one.sh`、`batch.py`、`gen.py`、`collect.py`、`base_rng.parm`
- 任务清单 `jobs1–3.txt`、`jobs_smoke.txt`；日志 `batch1–3.log`、`batch_smoke.log`
- `parms/`（每次运行的参数），`runs/<tag>/`（`cmd.txt`、`run.parm`、`run.log`、`sitl.log`、`logs/*.BIN`、`metrics.json`）
