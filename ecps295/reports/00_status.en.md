# Project status — 2026-10-13

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
| D out-of-distribution (6 modes, twin + Gazebo) | ✅ / 🔄 | intact more robust without texture (twin p = 0.001, Gazebo p = 0.028); Gazebo notex extension running | [6](06_connectome_controls_ood.en.md) |
| E mushroom-body plasticity (optional) | 🔄 started | first screen launched on the Mac mini and Jetson; code not on the branch yet | — |
| Structural lowbox fixes (no motion into unseen space, memory, side ToF) | ⏸ deferred | indicated by the H1 cause analysis | [5](05_hardware_alignment.en.md) |
| F safety layer + L0–L4 real-flight ladder | ⬜ | slew fix done; first flight planned 10-28 | — |

## Machines and what they are doing now

| Machine | Role | Now (2026-10-13, 03:00 PDT) |
|---|---|---|
| ROG laptop (Ryzen 9 5900HX, RTX 3070 Laptop) | Gazebo 8.15, 3 lockstep instances | D extension: notex lowbox_s20–49 × 17 configs (511 runs) |
| Mac mini M4 | fast twin, fastest per core | E stage screen (mushroom body) |
| Jetson AGX Orin | fast twin overflow | E stage screen |
| Lab server (Xeon 6330, 8× RTX 4090, shared) | many Gazebo instances with sim clock | idle for this project; used only when needed |
| Desktop (R5 7500X3D, RTX 5060) | Gazebo node, 2 instances | off (in use by the owner) |

All machines reproduce the same twin anchors bit for bit (`python -m rl.gates all`, 12/12 on this tree).

## Numbers behind this branch

| | Count |
|---|---|
| Hardware-aligned Gazebo runs in `rl/results/gazebo/hw_runs.csv` | 4,427 |
| Fast-twin test episodes in `rl/results/twin/hw/tests.csv.gz` | 18,370 |
| CMA-ES runs in `rl/results/twin/hw/` | 20 (h2 × 8, c4 × 8, c1 × 4) |

## Paper

Target ICUAS 2027 (full paper due 2027-02-05), framed as a systems paper. Next gates: G1 (11-01) simulation controls,
G2 (11-22) companion GUIDED hover + processor-in-the-loop, G3 (12-20) route B on the real drone.
