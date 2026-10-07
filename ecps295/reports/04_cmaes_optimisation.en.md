# Report 4 — Optimising the MiniFly decoder with CMA-ES, and what the controls say

> Branch `ecps295-sim` · code in [`ecps295/rl/`](../rl/) · 2026-10-04 … 10-07 · fast twin on ROG / Jetson AGX Orin,
> Gazebo re-tests on a cloud server (Gazebo 8.10 + sim clock) · status: A0, A2, A3, B1 done; A1/B2 Gazebo re-tests and
> fitness v2 in progress · 中文版：[04_cmaes_optimisation.zh.md](04_cmaes_optimisation.zh.md)

## Abstract

The v1 → v3_2 history in Report 1 was manual coordinate descent over about 40 hand-tuned parameters. We automated the
decoder part (17 parameters) with **CMA-ES** on a fast twin, kept training, selection and test worlds apart, and
re-tested the winners in Gazebo on the held-out 20 ft course cage (32 seeds each). The optimised decoder es3 cut the
runs with contact from **10/32 to 2/32** (p = 0.022, unpaired). A safety-layer bug found during the work (the slew
sign, `8b8eef9`) accounted for part of that gain. After the fix the paired comparison is **8/32 vs 2/32, Holm
p = 0.22, not significant**. es3 also covers less of the cage because every saccade became a 173° U-turn. Control arms
re-optimised with the same budget (degree-preserving shuffled connectomes) reach similar fitness on the fast twin. So
far the evidence does **not** show that the improvement depends on the real wiring. This is the main open question.

## 1. Method

| Item | Choice | Reason |
|---|---|---|
| Optimiser | CMA-ES, 12 candidates × 12 episodes × 90 s per generation | MiniFly is a non-differentiable event-driven LIF sim; 17–40 structured parameters suit CMA-ES. TU Delft's flow-driven SNN flyers were also trained with evolution |
| Parameters | 17 decoder parameters: yaw gain/readout, smoothing, brake terms and decay, escape/saccade thresholds, saccade duration/back-off/refractory/direction memory, caution hold/ramp | the knobs v1–v3_2 had tuned by hand |
| Fitness | F = 0.5·mean J + 0.5·CVaR₂₅(J) − P(contact) − 0.3·mean contacts | risk-aware; penalises the worst quarter of episodes |
| Variance control | common random numbers per generation, v3_2 scored on the same worlds every generation | makes generation-to-generation noise visible |
| Data split | procedural random cages for training · a fixed 24-episode hold-out for model selection · **cage20 never used** before the Gazebo test | no selection on the test scene |
| Fast twin | NED kinematics + ray-cast fisheye aligned with Gazebo (hfov 120°, 12° down, gamma, tape, net texture, lamps), real Retina/MiniFly/decoder code | ≈ 30× real time on 12 cores (ROG), bit-identical on Jetson and server |

## 2. Training

![CMA-ES convergence](figures/fig6_cmaes_convergence.png)

*Figure 1. Fitness per generation for es2 (default bounds) and es3 (bounds widened where es2's optimum sat on a bound).
Grey: hand-tuned v3_2 on the same worlds; orange dots: distribution mean on the hold-out set.*

Generation-to-generation noise is large (the grey v3_2 line moves as much as the candidates do), which is why the
comparison against v3_2 on the same worlds matters. On the hold-out set, both runs end well above v3_2 (−0.06 and
−0.18 vs −0.37).

## 3. Gazebo test on the held-out cage

![cage20](figures/fig7_cage20_gazebo.png)

*Figure 2. Gazebo cage20, GPS, 120 s patrols, 32 seeds per version. Left: runs with contact. Middle: coverage.
Right: why es3 covers less.*

| Version (after slew fix) | Runs with contact | Mean contacts | Near misses | Coverage | Contacts / 100 m |
|---|---|---|---|---|---|
| v3_2 | 8/32 | 0.25 | 0.84 | 0.651 | ≈ 0.98 |
| v4 | 21/32 | 0.91 | 1.94 | 0.653 | ≈ 3.5 |
| v4_oc5 (v4 with centering back to 5 columns, 30 seeds) | 11/30 | – | – | unchanged | +1.17 vs v3_2 (p = 0.23) |
| es3 | 2/32 | 0.12 | 0.81 | 0.627 | ≈ 0.45 |

Observations:

1. **The slew bug inflated the first result.** Before `8b8eef9`, `SafetyGovernor` rate-limited a negative command's
   return to 0 as if it were a sign change. That bug hurt v3_2 more than es3. After the fix, es3 vs v3_2 is
   McNemar rescued 8 / new 2, Holm p = 0.22.
2. **All contacts of all three versions were on the same object**, the lower box `stack_low` of the stack by the net.
3. **v4 regresses on cage20, and half of that is one parameter.** v4 had reverted the centering field from 5 to 3
   columns. Restoring 5 columns (v4_oc5) almost halves its contacts (29 → 16 on `stack_low`, p = 0.022 vs v4).
4. **Coverage loss comes from U-turns.** es3 is not slower (0.209 vs 0.197 m/s) and not more hesitant (slow fraction
   3.4 % vs 9.0 %). Instead its median saccade turn is 173° vs 76°, and 99 % of its saccades exceed 135°, so it keeps
   flying back over the cells it just covered.
5. **Empty-field false alarms are shared.** In a cage with only the net, the saccade rate is 1.38/min (v3_2), 1.22
   (es3) and 1.19 (a1_intact). Optimisation did not remove self-motion-induced looming.

## 4. Does the connectome matter? Control arms (A1)

Following the community survey (`rl/docs/community_RL_survey_and_roadmap_20261006_zh.md`): in several fly-connectome
projects, a shuffled or bypassed network did just as well once it was **re-optimised with the same budget**. We ran
the same protocol for every arm (20 gens × 12 × 12, starting from v3_2):

![Control arms](figures/fig8_a1_control_arms.png)

*Figure 3. Fast-twin hold-out results at each arm's best generation. Shuffle = Maslov–Sneppen edge swaps that keep
in/out degree and weight sign exactly.*

- The five shuffled connectomes reach fitness between −0.21 and −0.04, against −0.16 for the intact one. **On the
  fast twin, the intact wiring is not better than a degree-preserving shuffle.**
- `bypass` (no LIF) and `randread` (random readout) "win" on F by barely moving (coverage 0.27 / 0.10). F allows this
  degenerate solution. Fitness v2 therefore adds a coverage shortfall penalty below 0.40 and a smooth proximity cost.
- These are fast-twin numbers. The 32-seed Gazebo re-test of the arms is running, and the conclusion will rest on it.

## 5. Coverage weight sweep (B2, fast twin)

Warm-started from es3, w_cov = 2 / 4 / 8 gives coverage 0.58 / 0.57 / 0.62 (v3_2 0.50) with contact episodes
0.38 / 0.21 / 0.38 (v3_2 0.33). The Gazebo re-test is running.

## 6. Engineering for trustworthy numbers

- **A2 statistics** (`rl/stats.py`): exact McNemar with rescued/new counts, paired bootstrap CIs + Wilcoxon, Holm per
  metric, stratified bootstrap over shuffled instances.
- **A3 gates** (`rl/gates.py`, `anchors.json`): an open-loop sensory gate (DNp03/DNp01/DNg02 lateralisation; silenced
  equals dark), closed-loop smoke tests (saccade away from a box on either side, no saccade when hovering at the net,
  plain wall avoided), and SHA-256 anchors of fixed twin episodes that must match bit for bit across machines.
- **Gazebo at scale without sudo**: per-instance ports (+10·I), `GZ_PARTITION`, a model copy per instance, and a
  sim-time clock read from camera frames. This makes runs with RTF < 1 valid on a server limited to ≈ 0.36× lockstep.

## 7. Limitations and next steps

- The fast twin is pessimistic on contacts (looming ≈ 25 % weaker), so it is used for screening only.
- All Gazebo numbers here come from the server (8.10 + sim clock). Its v3_2 baseline matches the ROG (31 % vs 40 %
  over 5 seeds).
- Next steps: fitness v2 + `turncap` space (saccade 1.2–2.5 s, ≈ 60–140°) with 3 independent CMA seeds (A4); count
  Governor/fence interventions as cost (B3); then efference-copy and pathway-gain parameters (C), and the S8 scenes
  (low boxes, optical flow, ghosts) as out-of-distribution tests (D).
