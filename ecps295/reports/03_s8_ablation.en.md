# Report 3 — S8 ablation: 1,260 closed-loop flights on the final-hardware twin

> Branch `ecps295-sim` · 2026-10-05/06 · ROG Strix G733QR, Gazebo Harmonic 8.15 + ArduPilot Copter-4.7.0 SITL
> Raw data and paired statistics: [`results/ablation_2026-10-06/`](../results/ablation_2026-10-06/) ·
> full Chinese report with every table: [`REPORT_zh.md`](../results/ablation_2026-10-06/REPORT_zh.md) ·
> 中文摘要：[03_s8_ablation.zh.md](03_s8_ablation.zh.md)

## Abstract

We ablated every custom change to the FlyDrones MiniFly and every system fix, one at a time, on the digital twin of
the final hardware: optical flow + ToF, **no GPS**, route B. In total we ran **1,260 closed-loop flights**, and every
one met the whole-run real-time factor ≥ 0.95; no run was excluded. Conditions are paired on the episode (same world,
same brain seed), and p-values are Holm-corrected within each metric.

| Finding | Evidence |
|---|---|
| **LCb (blank detector) carries wall avoidance** | without it, wall contact 0 % → 50 %, min clearance 0.68 → 0.10 m (p < 0.001) |
| **The saccade escape sets the safety margin** | without it, contacts per 100 m 1.07 → 10.5 (p = 0.002), wall clearance 0.19 m |
| **Route B makes GPS-free flight possible** | without it, 88 % FC landings, success 82 % → 10 % (p < 0.001) |
| **The efference copy is a height stabiliser** | without it, the drone flies 0.17 m lower and the low boxes become obstacles (6.25 / 100 m, p = 0.017) |
| **v4's ventral pathway is redundancy, not nominal safety** | time over boxes 21.8 → 9.4 s (p < 0.001), contacts unchanged |
| **All three safety mechanisms work under faults** | height watchdog 7/10 → 0/10 above the fence (p = 0.016), FC failsafe 0/10 → 10/10 landed (p = 0.002) |

## 1. Design

**Full system.** MiniFly v4 connectome (v3 + LCv → MDN), EcpsDecoder (saccade escape, caution, brake memory, efference
copy, yaw decoupling, retreat with a 2 s cap), EcpsRetina (`blank`, `near`, `ventral`), EcpsPilot (camera and
height-source watchdogs, fence turn-back), route B at 20 Hz, and the FC failsafe for the companion (`FS_GCS_ENABLE 5`).

| Experiment | Scene | Episodes | Conditions | Runs |
|---|---|---|---|---|
| E1 patrol | random low-box worlds inside the 20 ft cage, 120 s | 50 worlds (9 key conditions), 20 otherwise | 22 | 710 |
| E2 wall approach | 1.5 m walls: taped, plain, offset, offset plain, 20 s | 4 walls × 8 seeds = 32 | 15 | 480 |
| E3 faults | 10 E1 worlds, fault at 30 s | 10 | 7 | 70 |

**Metrics.** The primary safety metric is **contacts per 100 m** (path floored at 1 m). The raw contact rate is
misleading here, because upstream FlyDrones parks at the geofence and flies only 2.9 m. Success means no contact and
no FC landing. All positions come from simulator truth.

**Statistics.** Continuous metrics: Wilcoxon signed-rank with rank-biserial r. Binary metrics: exact McNemar. Means
and paired differences come with 95 % bootstrap CIs (10,000 resamples). Holm correction is applied per metric across
all condition-vs-full comparisons.

**Validity.** 157 of 780 parallel E1/E3 runs fell below RTF 0.95 and were re-run on a single instance. The criterion
never changed. A 16 h hardware log (1,917 samples) shows a CPU peak of 85 °C and no throttling.

## 2. Results

### 2.1 E1 — patrol in random low-box cages

![E1 forest plot](figures/fig2_e1_ablation_forest.png)

*Figure 1. Paired difference to the full system (full: 1.07 contacts/100 m [0.44, 1.87], 82 % success [70, 92]).
Blue: Holm p < 0.05.*

| Condition | n | Contacts / 100 m | Success | FC landings | Path m | Over boxes s | Max true height m |
|---|---|---|---|---|---|---|---|
| **Full system** | 50 | 1.07 | 82 % | 0 % | 21.0 | 9.4 | 0.95 |
| − all new cells (LCb, LCn, LCv) | 50 | **13.77** | **42 %** | 0 % | 22.6 | **14.8** | 0.92 |
| − saccade escape | 50 | **10.45** | 60 % | 0 % | **16.3** | **20.2** | **1.01** |
| − LCb | 50 | **7.50** | 58 % | 0 % | 18.9 | 10.1 | 0.97 |
| − LCv | 50 | 1.67 | 82 % | 0 % | **25.1** | **21.8** | 0.96 |
| − retreat | 50 | 1.06 | 86 % | 0 % | **25.1** | **21.7** | 0.95 |
| − route B | 50 | 1.93 | **10 %** | **88 %** | **10.4** | **4.5** | **1.37** |
| upstream FlyDrones | 50 | 10.22 | 70 % | **26 %** | **2.9** | **6.1** | **1.15** |
| − efference copy | 20 | **6.25** | 45 % | 0 % | 19.3 | **2.2** | **0.78** |
| − yaw decoupling | 20 | 0.84 | 90 % | 0 % | 21.3 | 15.1 | **1.09** |
| − fence turn-back | 20 | 0.75 | 95 % | 0 % | **6.0** | **1.8** | **1.03** |
| ladder v1.3 (upstream brain + new decoder) | 20 | **8.29** | 40 % | 5 % | 22.4 | 17.3 | 0.95 |
| GPS reference (v4) | 20 | 1.53 | 75 % | 0 % | 23.4 | **22.1** | 0.90 |

Bold: Holm p < 0.05 against the full system. The other n = 20 conditions (LCn, retreat cap, baro-rise check, caution,
brake memory, ladder v2.1 / v3, upstream + B) are listed in `e1.md`.

### 2.2 E2 — wall approach

![E2 clearance](figures/fig3_e2_wall_clearance.png)

*Figure 2. Minimum clearance per run (32 per condition). Only conditions without LCb, and the upstream brain, ever
touch the wall. The finer components (caution, brake memory, LCn) are each worth 6–8 cm of margin.*

### 2.3 E3 — fault injection

See Figure 3 of [Report 2](02_gps_free_navigation.en.md#4-s7b--companion-faults). Camera watchdog: 4/10 → 1/10
contacts (n.s. at n = 10); height-source watchdog: 7/10 → 0/10 above the 1.3 m fence (p = 0.016); FC failsafe:
0/10 → 10/10 landed when the companion hangs (p = 0.002).

## 3. Discussion

- **Plain walls are invisible to looming.** Upstream looming depends on flow, and a textureless wall produces almost
  none. Without LCb, the escape onset moves from 0.89 m to 0.20 m.
- **Turn, don't climb.** Under the 1.0 m soft ceiling an upstream climb escape has nowhere to go. Replacing it with a
  saccade (turn away + back-off) is worth roughly a 10× reduction in patrol contacts.
- **The efference copy turned out to be a height component.** It cancels the "rising" illusion that ground flow
  creates during forward flight. Without it the brain keeps pushing the drone down until low boxes are at eye level.
- **v4 should be presented as a fallback.** With route B working, flying over a box is harmless, so the ventral
  pathway changes only the time over boxes. Its value appears when the height estimate fails, as in S7d.
- **GPS parity.** The GPS reference (same v4 brain) does not differ from the GPS-free system in contacts or success.
  In this scene, flow + route B matches GPS.

## 4. Limitations

1. Sim-to-real: SITL baro has no prop wash or drift, and the 3901-L0X and VL53L0X are not bench-calibrated yet.
2. One scene family: cardboard boxes in a 20 ft cage.
3. n = 20 conditions are under-powered. "Not significant" does not mean "no effect" (for example, removing the
   caution period gives 4.12 vs 1.07 per 100 m, p = 1 after Holm).
4. **All S8 runs predate the SafetyGovernor slew-sign fix (`8b8eef9`).** Releasing a negative command took 150 ms
   instead of being immediate. On cage20 the fix changed v3_2's contact runs from 10/32 to 8/32 (Report 4).
   Post-fix re-runs of the key conditions are planned.

## 5. Reproduce

```bash
cd ~/sim/FlyDrones/ecps295
(cd gazebo && python3 make_quad.py)                 # lowbox_s0..49
python gen_ablation.py ~/sim/runs/ablation          # E1/E2/E3 matrices
bash ~/sim/run_ablation.sh ~/sim/runs/ablation 2    # 2 parallel Gazebo instances, auto re-run RTF < 0.95
python ablation_stats.py ~/sim/runs --ref full --episodes "^lowbox_s[0-9]+_k" --out final/e1
python reports/make_figures.py                      # figures in reports/figures/
```
