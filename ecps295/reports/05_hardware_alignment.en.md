# Report 5 — Hardware alignment: re-doing the RL pipeline under the real sensor set

> Branch `ecps295-sim` · code in [`ecps295/rl/`](../rl/) · 2026-10-07 … 10-13 · Gazebo on the ROG laptop (8.15, lockstep)
> and a cloud server (8.10 + sim clock), fast twin on a Mac mini M4 and a Jetson AGX Orin · stages H0–H3 and C1 done ·
> figures from [`make_figures_hw.py`](make_figures_hw.py), numbers in [`hw_stats.txt`](hw_stats.txt) ·
> 中文版：[05_hardware_alignment.zh.md](05_hardware_alignment.zh.md)

## Abstract

Report 4 trained and accepted decoders in a GPS cage with a ground-truth twin. The real drone has no GPS: it flies on
the Matek 3901-L0X (optical flow + ToF) with route B, on the v4 model, over low boxes and a 1.1 m box stack. Re-tested
in that setting (ROG, `lowbox_s0–29`), **no RL decoder beat the hand-built ones**. We therefore inserted a hardware
alignment stage before everything else: an acceptance gate with the real sensor set (H0), a twin with an optical-flow
error model calibrated against Gazebo (H1), re-optimisation on the v4 base with hardware caps (H2), and paired
acceptance against v4 (H3). Two findings shape the rest of the project:

1. **On lowbox, contacts are insensitive to the decoder.** Twelve decoders, hand-built or optimised, all hit something
   in 6–15 of 50 worlds. 77 % of contacts are on the 1.1 m stack, during an escape, and the escape started at a median
   0.06 m from it: the drone brushes the stack from a direction its camera cannot see. Gazebo cannot even rank the
   models by contacts (split-half ρ < 0); continuous metrics (coverage, EKF error, time over boxes, clearance) can.
2. **One H2 seed passes as a conservative variant.** `h2_s3` keeps lowbox contacts unchanged and adds 0.11–0.13 m of
   clearance on walls and in cage20 (p < 0.001), at 0.05–0.12 lower coverage. A blank-cue efference term (C1) halves
   empty-field false alarms in the twin without hurting Gazebo, so **`v4_eff` is the new default**.

## 1. What changed between the GPS cage and the real drone

| Item | RL pipeline until 10-07 | Real drone (and from now on) |
|---|---|---|
| Navigation | twin: ground truth; Gazebo: GPS | no GPS; EKF3 on optical flow + ToF; route B height from the companion |
| Base model | v3_2 (no ventral pathway) | v4: LCv → MDN retreat, height watchdog, route B |
| Worlds | procedural cages, net-side stacks | lowbox: 6–10 boxes of 0.2–0.75 m + a 1.1 m stack 1.0–2.3 m from take-off; wall approaches; cage20 |
| Crossing a low box | no consequence in the twin | ToF drops, the EKF height/velocity is pulled off |
| Backing up | exact in the twin | no rear vision; under flow nav it drifts |

In the ROG `fx_*` matrix (1,209 Gazebo runs) the GPS-cage decoders hit the 1.1 m stack 2–4× as often as v4, mostly
while backing out of a saccade, and their EKF error was twice v4's. The twin ranked the same 30 worlds the other way.

## 2. H0 — a Gazebo gate with the real sensor set

The server tree `~/sim/hw/ecps295` runs optical flow + ToF + route B with a sim-time clock (`gz_batch.py --hw`).
Cross-machine check against the ROG on the same 50 lowbox worlds: runs with contact 4/4, 4/4, 7/6, 5/6 for v4, v3_2,
es2, es3 (≤ 1 apart), and coverage, time over boxes, EKF error and clearance all close, so server runs at RTF 0.15
remain valid. On wall approaches and cage20 the RL decoders were not worse than v4 (es2/es3 clearance +0.13 m on
walls, p < 0.001); the problem is specific to lowbox.

## 3. H1 — calibrating the twin, and what Gazebo can measure

The twin gained an optical-flow navigation model (EKF speed overestimate, terrain-lag, random walk; telemetry gives the
estimate), 10 Hz ToF, a barometer and lowbox-style procedural worlds. The planned gate was "the twin reproduces
Gazebo's ranking of 8 models on lowbox (Spearman ρ ≥ 0.6)". It failed in every one of 9 settings, and the reason is on
the Gazebo side:

![Lowbox reliability](figures/fig9_h1_lowbox_reliability.png)

*Figure 1. Left: runs with contact on lowbox_s0–49 (ROG). Right: split-half reliability of each metric across the 8
calibration models — draw two random halves of the 50 worlds and correlate the model means.*

- Every decoder hits something in about 1 run in 5 (6–15/50). The two-halves ranking of contacts is not just weak but
  negative (ρ ≈ −0.45; contacts/100 m −0.24): with 50 worlds the contact ranking is noise. Path length (0.93),
  coverage (0.81), EKF error and time over boxes (0.67) and clearance (0.64) are reproducible.
- The gate was therefore revised (10-08, after the fact, recorded as such) to the mean ρ of four reliable metrics
  (time over boxes, estimate error, coverage, clearance) ≥ 0.6. A per-episode randomised flow-error model
  (terrain lag 0.5–4 s, random walk 0.02–0.06, cruise 0.8 ± 0.1 m; `rl.twin.flow_dr`) passes on s0–29 (ρ 0.64) but
  not on the held-out s30–49 (0.36). The twin's absolute time over boxes is about one third of Gazebo's.
- **Why lowbox contacts do not respond to the decoder** (400 ROG flights): 77 % of contacts are on the 1.1 m stack,
  and the contact rate rises with the number of stacks in the world (0 → 0.08, 3 → 0.35) but hardly with the world
  itself (ICC ≈ 0.09). 77 % happen during an escape (backing 46 %, turning in place 42 %, forward 12 %), 40 % within
  1 s of its start. Of 1,589 escapes started within 1 m of the stack, the 96 that ended in contact had started at a
  median **0.06 m**; the 1,493 others at 0.55 m. Escapes started ≥ 0.2 m away hit in 1 % of cases.
- A server ablation of the escape structure (no saccade back-off / no ventral retreat / both, 169 runs) did not
  confirm "backing causes the hits" (10/50 → 12, 8, 5/49; p = 0.27). Conclusion: the drone **sees the stack too late or
  not at all**, so decoder tuning has reached its ceiling on lowbox. Structural fixes (never move toward unseen
  space, short-term spatial memory, side/rear ToF) are on the list but were deferred by decision.

## 4. H2 / H3 — re-optimising on the v4 base and accepting against v4

H2 searched the `v4hw` space (v4 parameters with hard caps: saccade 1.2–2.0 s, back-off ≤ 0.3 × 0.6 s, refractory
≤ 3 s, plus ventral parameters) with fitness v3 (back-off distance and estimate-error costs, coverage floor 0.35), the
randomised flow model and half lowbox worlds; 3 CMA seeds, 20 gens × 12 candidates × 12 episodes, the best hold-out
generation exported as `minifly/h2_s1..3.yaml`. In the twin none had fewer contacts than v4; the gain was coverage.

![H3 acceptance](figures/fig10_h3_acceptance.png)

*Figure 2. H3 on the server: paired differences against v4 on lowbox_s0–49, 32 wall approaches and cage20 × 32.*

| Candidate | lowbox contact runs (server) | walls: Δ clearance | cage20: contact runs, Δ clearance | Verdict |
|---|---|---|---|---|
| h2_s1 | 10 → 6/49 (p = 0.42) | **−0.20 m** (worse) | 6 → 6, ±0 | rejected |
| h2_s2 | 10 → 5/50 (p = 0.30) | +0.02 m | 6 → 11, more grazes (p = 0.03) | rejected (also worse on the ROG: 15/49) |
| **h2_s3** | 9 → 5/48 (p = 0.39) | **+0.13 m** (p < 0.001) | 6 → 1, **+0.11 m** (p < 0.001), near misses 0.75 → 0.16 | **passes, "conservative"** |

Pre-registered rule: contacts not significantly worse **and** at least one of contacts/100 m or clearance
significantly better. h2_s3 passes on clearance; its price is 0.05–0.12 lower coverage. An independent ROG re-run of
lowbox agreed (h2_s3 10 → 10 with less time over boxes and lower EKF error; h2_s2 10 → 15, significantly worse) and
also showed that the direction of lowbox contact differences can flip between machines — another sign that 50-world
contact counts are noise.

## 5. C1 — an efference copy for the blank cue

In an empty cage v4 makes 2.75 false turns per minute. Splitting the DNp03 input showed 81 % come from the blank cue
(LCb), 58 % of them within 1 s after a turn: the turn sweeps the view onto untextured area before the baseline
adapts. The looming efference term (a, b) did almost nothing; a blank term did: after a turn the LCb threshold rises
by c × turn and decays with τ (`vision.ecps_efference`, off by default).

![C1](figures/fig16_c1_efference.png)

*Figure 3. Left: twin, empty field (32 × 120 s). Right: Gazebo, v4 vs v4_eff paired on the same worlds and seeds.*

- `v4_eff` (c = 0.4, τ = 0.8 s) halves false alarms in the twin (2.75 → 1.22/min) and is not worse anywhere in Gazebo:
  lowbox 11 → 9/50 (ROG) and 10 → 6/50 (server), cage20 6 → 6/32, wall approaches without contact. Combined lowbox
  21 → 15/100, not significant. **It is the default from now on.**
- CMA over `v4hw_eff` (26 parameters, fitness v4 = v3 − 0.1 × false alarms/min) mostly left c at its lower bound,
  because the penalty is small next to episode noise; one seed (c1_s3) did learn c ≈ 0.20, τ ≈ 1.1 s, a = 0 — the hand
  design, found independently. Warm-starting from v4_eff (c1b_s1) kept the blank term (c = 0.36) but bought coverage
  with contacts (contact fraction 0.42 vs 0.17), so it was not adopted.
- `h2_s3_eff` (h2_s3 + the same term) was not better than h2_s3 and was dropped.

## 6. Status after hardware alignment

| Stage | Status | Result |
|---|---|---|
| H0 gate | ✅ | server + ROG agree within one run; problem specific to lowbox |
| H1 twin calibration | ✅ (gate revised) | contacts are not rankable in Gazebo; randomised twin passes the revised gate on s0–29, not on s30–49 |
| H2 / H3 | ✅ | h2_s3 accepted as a conservative variant; no candidate lowers lowbox contacts |
| C1 efference copy | ✅ | v4_eff becomes the default |
| Structural fixes for lowbox | ⏸ deferred | the cause analysis points here |

The control arms of the connectome question were then repeated in this setting — see
[Report 6](06_connectome_controls_ood.en.md).

## 7. Data and reproduction

- Gazebo runs: [`rl/results/gazebo/hw_runs.csv`](../rl/results/gazebo/hw_runs.csv) (groups `fx_*` ROG H0/H1,
  `h0low_* / h0cage_* / h0wall_*` server, `mc1_*` ROG C1).
- CMA runs (without the per-episode logs, which stay on the machines): [`rl/results/twin/hw/`](../rl/results/twin/hw/).
- Twin tests: [`rl/results/twin/hw/tests.csv.gz`](../rl/results/twin/hw/tests.csv.gz) (`rl.hw_arms`, `--conds`, `rl.ood`).
- `cd ecps295 && python reports/make_figures_hw.py` rebuilds figures 9–16 and `hw_stats.txt`.
- `python -m rl.gates all` passes 12/12 on this tree (sensory gate, closed-loop smoke tests, bit-identical anchors on
  Mac mini, server, ROG and Jetson).
