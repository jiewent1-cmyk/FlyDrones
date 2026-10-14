# Project status — 2026-10-14

> Snapshot of the simulation and RL work on branch `ecps295-sim` across all experiment machines.
> 中文版：[00_status.zh.md](00_status.zh.md) · reports index: [README.md](README.md)

## Stages

| Stage | Status | One-line result | Report |
|---|---|---|---|
| S4–S8 twin, MiniFly v0 → v4, GPS-free navigation, ablation | ✅ | v4 + route B flies without GPS; LCb, saccade, route B, efference copy are load-bearing | [1](01_twin_and_minifly.en.md) [2](02_gps_free_navigation.en.md) [3](03_s8_ablation.en.md) |
| A0–A4, B1–B3 CMA-ES in the GPS cage | ✅ | es3 better in the GPS cage, but not under the real sensor set | [4](04_cmaes_optimisation.en.md) |
| H0–H3 hardware alignment | ✅ | lowbox contacts insensitive to the decoder; h2_s3 accepted as conservative variant | [5](05_hardware_alignment.en.md) |
| C1 efference copy | ✅ | v4_eff halves empty-field false alarms, default from now on | [5](05_hardware_alignment.en.md) |
| C0 / C2 under flow nav | ✅ | brain needed (≈ 10–40× contacts without it); real wiring beats 5/5 shuffles, weakly | [6](06_connectome_controls_ood.en.md) |
| C4 bypass vs intact (8 + 8 seeds) | ✅ | bypass better (Gazebo p = 0.015, twin p = 0.010); readout is not the bottleneck | [6](06_connectome_controls_ood.en.md) |
| D out-of-distribution (6 modes, twin + Gazebo) | ✅ | intact more robust without texture (twin p = 0.001; Gazebo on 50 worlds p = 0.001) | [6](06_connectome_controls_ood.en.md) |
| E mushroom-body plasticity | ✅ negative | 6 settings, 48 lifetimes: learned weights no better than shuffled ones | [8](08_mushroom_body.en.md) |
| Structural lowbox fixes S1–S3 | ✅ | no back-off: no effect; memory: n.s.; side/rear rangers −38 % contact runs | [7](07_range_sensing_shield.en.md) |
| N route review: range shield S4, ToF into the brain S5 | ✅ twin + Gazebo | 5 ToF + shield: lowbox 0/150 (twin), 0/50 (Gazebo, 27° cone); s5 1/50 with 42 % fewer shield interventions | [7](07_range_sensing_shield.en.md) |
| N1 larger sample (second brain seed) | 🔄 | v4 done (lowbox 8/50); s5 on the same set ~40/58, done ≈ 10-14 08:00 PT | [7](07_range_sensing_shield.en.md) |
| N3 CMA with the shield in the loop | 🔄 | seed 1 (s5): gains fitness by flying less, not adopted; seed 1 (sh5u) running | [7](07_range_sensing_shield.en.md) |
| N4 stress tests, N5 pre-registered acceptance, N6 ToF bench + real drone | ⬜ | N6 needs the hardware decision (5 × VL53L1X) | — |
| F safety layer + L0–L4 real-flight ladder | ⬜ | slew fix done; first flight planned 10-28 | — |

## Machines and what they are doing now

| Machine | Role | Now (2026-10-14, 00:40 PDT) |
|---|---|---|
| ROG laptop (Ryzen 9 5900HX, RTX 3070 Laptop) | Gazebo 8.15, 3 lockstep instances | s5 on the second brain seed (lowbox k+100, cage20 k8–15), 58 runs; pauses 01:00–07:00 PT |
| Mac mini M4 | fast twin, fastest per core | idle (N3 s5 seed finished 10-13 23:39) |
| Jetson AGX Orin | fast twin overflow | N3 seed 1 with the sh5u shield, generation 15/20 |
| Lab server (Xeon 6330, 8× RTX 4090, shared) | many Gazebo instances with sim clock | idle for this project; used only when needed |
| Desktop (R5 7500X3D, RTX 5060) | Gazebo node, 2 instances | off (in use by the owner) |

All machines reproduce the same twin anchors bit for bit (`python -m rl.gates all`, 12/12 on this tree). ROG and the
desktop run nothing between 01:00 and 07:00 Los Angeles time; their runners pause by themselves.

## Numbers behind this branch

| | Count |
|---|---|
| Hardware-aligned Gazebo runs in `rl/results/gazebo/hw_runs.csv` | 5,357 |
| Fast-twin test episodes in `rl/results/twin/hw/tests.csv.gz` (controls, seeds, OOD) | 18,370 |
| Fast-twin episodes in `rl/results/twin/n/tests.csv.gz` (S1–S5 screens, diagnosis) | 6,126 |
| Fast-twin episodes in `rl/results/twin/mb/tests.csv.gz` (stage E) | 10,944 |
| CMA-ES runs in `rl/results/twin/hw/` | 21 (h2 × 8, c4 × 8, c1 × 4, n3 × 1) |

## Decisions pending

- **Hardware:** add five VL53L1X single-zone ToF sensors (front pair at ±25°, right, rear, left; about 2.5 g on the
  companion's I²C) and feed them to both the brain and the shield (s5). A bench test (cage net, black boxes, box tops
  just below the sensor) must come before flight.
- **Paper framing:** from "connectome controller avoids obstacles" to "connectome controller + range shield, optimised
  with the shield in the loop"; the connectome question stays with C0/C2/C4 and the OOD tests.

## Paper

Target ICUAS 2027 (full paper due 2027-02-05), framed as a systems paper. Next gates: G1 (11-01) simulation controls,
G2 (11-22) companion GUIDED hover + processor-in-the-loop, G3 (12-20) route B on the real drone.
