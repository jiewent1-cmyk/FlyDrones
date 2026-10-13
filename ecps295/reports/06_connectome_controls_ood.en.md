# Report 6 — Does the connectome matter? Control arms and out-of-distribution tests under the real sensor set

> Branch `ecps295-sim` · code in [`ecps295/rl/`](../rl/) and [`gazebo/ood_world.py`](../gazebo/ood_world.py) ·
> 2026-10-08 … 10-13 · Gazebo: server (C0/C2) and ROG laptop (C4, D); fast twin: Mac mini M4 and Jetson AGX Orin ·
> figures from [`make_figures_hw.py`](make_figures_hw.py), numbers in [`hw_stats.txt`](hw_stats.txt) ·
> 中文版：[06_connectome_controls_ood.zh.md](06_connectome_controls_ood.zh.md)

## Abstract

We repeated the control arms of Report 4 in the hardware-aligned setting of [Report 5](05_hardware_alignment.en.md)
(optical flow + ToF + route B, v4 base) and added out-of-distribution (OOD) tests. After **about 18,000 fast-twin
episodes and about 1,900 Gazebo flights**, the picture is mixed, and we report it as it is:

- **C0 — the network is needed.** Turning the brain off, or removing its looming input, raises contacts per 100 m
  about 10× on lowbox and 40× in cage20 (all p < 10⁻⁵).
- **C2 — real wiring beats degree-preserving shuffles**, consistently but weakly: lower contacts/100 m than all 5
  shuffles in both scenes, with two-level bootstrap CIs that just include 0.
- **C4 — but the readout is not the bottleneck.** With the identical optimisation budget, a *bypass* arm (no LIF
  network: 8 tunable gains map visual features straight to pseudo-DN rates) beats the intact network: 8 vs 8 seeds,
  Gazebo lowbox runs with contact median 2.5 vs 10.5 (exact Mann-Whitney p = 0.015), confirmed on an independent twin
  test without selection bias (p = 0.010). The twin ranking transfers to Gazebo (ρ = 0.75). The cost: bypass makes
  more spontaneous turns in an empty field (3.3 vs 1.8/min, p = 0.04).
- **D — the first result for the connectome.** Remove the texture from boxes and floor (*notex*) and bypass loses its
  advantage while intact does not: change in runs with contact +6.5 vs −2.0 in the twin (p = 0.001), reproduced in
  Gazebo (+1.0 vs −1.5, p = 0.028). Under wind, half-blind camera, low light or ToF faults bypass stays ahead or both
  degrade alike.

## 1. Arms

| Arm | What it is | Optimised? |
|---|---|---|
| v4 / v4_eff | hand-built MiniFly v4 (+ blank efference term) | no |
| brainoff | LIF network silent; the decoder sees no DN activity | no |
| noloom | looming input to the network removed | no |
| shuffle1–5 | Maslov–Sneppen edge swaps keeping in/out degree and weight sign exactly | no (C2 tests the wiring itself) |
| **intact `h2_s1..8`** | v4 base, CMA over the `v4hw` decoder space (22 dims) | 8 CMA seeds |
| **bypass `c4_s1..8`** | network removed; `v4hw` + 8 gains from visual features to pseudo-DN rates (30 dims) | 8 CMA seeds |

Intact and bypass use the identical protocol: fitness v3, the randomised optical-flow twin, half of the training
worlds lowbox, 20 generations × 12 candidates × 12 episodes, CMA seeds 1–8, best hold-out generation exported.

## 2. C0 / C2 — removing or rewiring the network

![C0 C2](figures/fig11_c0_c2_hw.png)

*Figure 1. Server Gazebo, 574 runs paired with v4 (left, middle), and the fast-twin prescreen with the same arms
(right, 2,782 episodes).*

| Arm | lowbox contacts/100 m | lowbox runs hit | cage20 contacts/100 m | cage20 runs hit |
|---|---|---|---|---|
| v4 (real wiring) | 1.8 [0.7, 3.1] | 10/50 | 1.1 [0.3, 2.2] | 6/32 |
| shuffle 1–5 | 3.6 – 5.9 | 11–15/50 | 1.4 – 4.4 | 6–9/32 |
| no looming | 21.4 | 33/49 | 47.2 | 32/32 |
| brain off | 18.2 | 33/50 | 45.2 | 32/32 |

- **C0 holds strongly** in both Gazebo and the twin.
- **C2 holds in direction, weakly in size.** Real wiring is better than 5/5 shuffles on contacts/100 m in both scenes;
  the two-level bootstrap (instances, then seeds) of "shuffle − real" gives lowbox [−0.07, +7.5] and cage20
  [−0.8, +5.7]. Shuffles avoid less: on lowbox 4/5 have smaller clearance and higher coverage. In the twin the effect is
  much clearer (cage20: 17–23/32 vs 6/32 for all five shuffles).

## 3. C4 — bypass against intact, seed by seed

![Bypass vs intact](figures/fig12_bypass_vs_intact.png)

*Figure 2. Each dot is one CMA seed. Left: best hold-out fitness (selected on the hold-out set, so biased upward).
Middle: independent twin test on 150 lowbox episodes outside the selection set. Right: Gazebo, ROG, lowbox_s0–49
paired with v4.*

| Seed | bypass: twin / Gazebo lowbox runs hit | intact: twin / Gazebo lowbox runs hit |
|---|---|---|
| 1 | 12/150 · **3/49** (p = 0.02) | 33/150 · 6/50 |
| 2 | 13/150 · **2/50** (p = 0.004) | 32/150 · 15/49 |
| 3 | 52/150 · 13/50 | 38/150 · 10/50 |
| 4 | 17/150 · 6/50 | 32/150 · 11/50 |
| 5 | 22/150 · **2/50** (p = 0.004) | 35/150 · 6/50 |
| 6 | 10/150 · **2/50** (p = 0.004) | 27/150 · 15/50 |
| 7 | 20/150 · 5/50 | 28/150 · **3/50** (p = 0.04) |
| 8 | 14/150 · **2/50** (p = 0.004) | 47/150 · 16/50 |
| v4 | 34/150 · 11/50 | |

- All 8 bypass seeds have better hold-out fitness than all 8 intact seeds (−0.61 … −0.96 vs −1.24 … −1.96,
  p = 0.00016) — but that is the selection set. On the independent twin test 7/8 bypass seeds hit less than v4;
  no intact seed does. Seed-level medians 15.5 vs 32.5 (p = 0.010); contacts/100 m 1.65 vs 5.34 (p = 0.007); cage20
  6.5 vs 12.5 (p = 0.005).
- In Gazebo, bypass totals 35 runs with contact against 82 for intact (median 2.5 vs 10.5, p = 0.015). c4_s3 is the
  counter-example in both simulators. A re-run of c4_s1 with new brain seeds (k + 100) repeated the direction (5 → 1);
  pooled over both rounds, 98 pairs, 14 rescued vs 2 new, McNemar p ≈ 0.004.

![Transfer](figures/fig13_twin_gazebo_transfer.png)

*Figure 3. Left: per configuration, twin vs Gazebo runs with contact (Spearman ρ = 0.75, p = 0.0009). Right: the cost
of bypass — more false turns in an empty field (c4_s6: 6.4/min).*

![Curves](figures/fig15_c4_h2_curves.png)

*Figure 4. Hold-out fitness during optimisation. Bypass starts better and stays better; most of its gains are
already present by generation 4. Many bypass gains end at a bound (c0 = 60, k_flow = 3, th_gf = 0).*

**Reading.** Under the same budget, routing optic flow straight to steering beats tuning only the readout of the
network. This is unfavourable to "the connectome helps" and is reported as such. Together with C0/C2 the consistent
interpretation is: the network's structure contributes (removing or rewiring it hurts), but within this task the
decoder that reads it is not where the performance is limited — a direct visuomotor map with tuned gains is at least
as good, at the price of more spontaneous turning.

## 4. D — out-of-distribution tests

`rl/twin.py` gained `TwinDrone(ood=...)` with its own random streams (the in-distribution path stays bit-identical):

| Mode | Change | Strength chosen on v4, lowbox_s0–19 (in distribution 2/20 runs hit) |
|---|---|---|
| lowlight | camera gain ×0.3, noise 0.02 | 6/20 |
| notex | boxes one flat colour, no shading, no tape, plain mat, no floor strips | 3/20 |
| halfcam | left or right half of the image covered | 8/20 |
| wind | steady drift 0.03 m/s + gusts 0.10 | 12/20 (drift 0.12/0.06/0.03/0 → 20/19/12/5) |
| tof | 10 %/s chance of a 1–4 s stuck reading + 3 % out of range; the flow scale uses the faulty range | 3/20 |
| ghost | boxes invisible but still solid | 11/20 |

`rl/ood.py` runs the 16 exported configurations + v4 on the same worlds, brain seeds and randomisation as the
independent test, so the in-distribution runs are a paired baseline (11,736 twin episodes, 0 errors, Mac mini +
Jetson). `gazebo/ood_world.py` builds notex and ghost copies of Gazebo worlds (notex: emissive single-colour boxes,
no tape, no sun shading, mat a uniform grey 0.425; ghost: collision only); 484 ROG flights.

![OOD](figures/fig14_ood.png)

*Figure 5. Change in runs with contact relative to in-distribution, per seed. The test asked by FLYNN-style
arguments: if the connectome helps, intact should degrade less.*

| Mode | lowbox Δ bypass / intact (p) | cage20 Δ bypass / intact (p) | Reading |
|---|---|---|---|
| **notex** | **+6.5 / −2.0 (0.001)** | +1.0 / −3.5 (0.05) | bypass loses its edge (11.5 vs 8.5 runs hit, p = 0.57); intact more robust |
| lowlight | +4.5 / 0.0 (0.23) | +1.5 / +1.0 (0.72) | advantage shrinks, n.s. |
| halfcam | +9.0 / +10.5 (0.38) | +24.5 / +19.0 (0.04) | cage20 saturates near 31/32 |
| wind | +17.5 / +21.5 (0.015) | +5.5 / +9.0 (0.13) | bypass degrades less |
| tof | 0.0 / −1.5 (0.72) | +0.5 / +2.5 (0.20) | both unaffected |
| ghost | +24 / +19 (0.20) | +10.5 / +18.5 (0.05) | avoidance is visual; part of bypass's cage20 edge is "blind" |

- **notex reproduces in Gazebo** (ROG, lowbox_s0–19 × 17 configurations): in distribution bypass hits less (median
  1 vs 5, p = 0.007); without texture both are at 2 (p = 0.96); change +1.0 vs −1.5 (p = 0.028). Caveat: 20 worlds per
  configuration, small counts; intact hitting *less* without texture may come from the emissive boxes contrasting more
  with the floor, not from intact getting better.
- **ghost in Gazebo** (cage20 × 8, v4 + seeds 1–4): every configuration hits heavily (intact 8/8 for all four seeds,
  v4 8/8; bypass 4, 8, 3, 8; p = 0.34). The twin's "bypass avoids partly without seeing" appears only on c4_s1/s3 and
  is weak in Gazebo.

## 5. What this means for the paper

- Claims that hold: the network is necessary (C0); the real wiring is better than shuffles in direction (C2); the
  twin ranking transfers to Gazebo (ρ = 0.75), so twin screening is legitimate.
- Claim that does not hold: "optimising the connectome readout beats a direct visuomotor map". It does not (C4), and
  the paper must say so.
- The one place where the network is more robust is texture removal — consistent with LCb/blank and looming cues
  that do not depend on surface texture, against a bypass that leans on flow magnitude. This is the most interesting
  thread to extend (notex lowbox_s20–49 is running on the ROG now, 511 runs).

## 6. Data and reproduction

- Gazebo: [`rl/results/gazebo/hw_runs.csv`](../rl/results/gazebo/hw_runs.csv) — `h0low_* / h0cage_*` (server C0/C2),
  `mc4_* / mc4r_*` (ROG C4 seeds; `mc4r` = brain seeds + 100), `md_*` (ROG D), `fx_h2_s*` (ROG intact seeds 1–3),
  `mc1_v4` (ROG v4 baseline).
- Twin: [`rl/results/twin/hw/tests.csv.gz`](../rl/results/twin/hw/tests.csv.gz), CMA runs in
  [`rl/results/twin/hw/`](../rl/results/twin/hw/); configs `minifly/c4_s1..8.yaml`, `minifly/h2_s1..8.yaml`.
- Commands:
  ```bash
  python -m rl.hw_arms --conds c4_s1 ... h2_s8 --out rl/out/seeds.jsonl     # independent test
  python -m rl.ood --conds c4_s1 ... h2_s8 --modes notex ghost ... --out rl/out/ood.jsonl
  python -m rl.ood --report --out rl/out/ood.jsonl --base rl/out/seeds.jsonl
  python gazebo/ood_world.py --mode notex ghost --worlds lowbox_s0 ... cage20
  ECPS_ARM=bypass python run_g4.py ...        # control arm in Gazebo (rl.arms), default intact
  ```
