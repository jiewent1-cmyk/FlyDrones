# Project status — 2026-10-14 (23:30 PDT)

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
| N1 larger sample (second brain seed) | ✅ | s5 replicates: two seed sets pooled, lowbox 19/100 → 2/100 (p = 0.0001), cage20 6/16 → 0/16 | [7](07_range_sensing_shield.en.md) |
| S6/S7 cheaper sensor layouts, 8×8 ToF + local 3D map | ✅ Gazebo / 🔄 replication | **front + rear VL53L7CX + map (w2map): lowbox 0/50, cage20 0/8**, the only layout at 0 in both; second brain seed running | [7](07_range_sensing_shield.en.md) §8 |
| **Hardware decision** | ✅ 10-14 22:20 | **front + rear VL53L7CX 8×8 + local 3D map + safety shield** on the Orange Pi | [7](07_range_sensing_shield.en.md) §8 |
| N3 CMA with the shield in the loop | 🔄 round 2 | round 1 (5 single-zone ToF, weighted fitness) traded safety or coverage, not adopted; round 2 on w2map with fitness v6 (coverage objective, five safety proxies as hard constraints), 5 seeds | [7](07_range_sensing_shield.en.md) §8 |
| N4 stress tests, N5 pre-registered acceptance, N6 ToF bench + real drone | ⬜ | redesigned around w2map (EKF drift injection, altitude variation, speed, sensor degradation); N6 waits for the VL53L7CX boards | — |
| F safety layer + L0–L4 real-flight ladder | ⬜ | slew fix done; first flight planned 10-28 | — |

## Machines and what they are doing now

| Machine | Role | Now (2026-10-14, 23:30 PDT) |
|---|---|---|
| ROG laptop (Ryzen 9 5900HX, RTX 3070 Laptop) | Gazebo 8.15, 3 lockstep instances | w2map / w1map on the second brain seed (58 runs each), then w2map at 0.65 m/s cruise; pauses 01:00–07:00 PT |
| Mac mini M4 | fast twin, fastest per core | N3 round 2 on w2map, CMA seeds 1, 2, 4 (≈ 2.5 h each) |
| Jetson AGX Orin | fast twin overflow | N3 round 2, CMA seeds 3, 5 (≈ 8 h each) |
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

## Final hardware (decided 10-14)

| Part | Connects to | Role |
|---|---|---|
| Matek 3901-L0X (flow + downward ToF) | flight controller UART4 (GPS keeps only its I²C compass) | EKF velocity and height without GPS |
| Orange Pi Zero 3W | flight controller UART3 (MAVLink2), powered from Vx | brain, local 3D map, safety shield |
| 2 × VL53L7CX 8×8 ToF (60° × 60°), front and rear | Orange Pi I²C TWI0 / TWI1, 3.3 V | obstacle ranging → map → shield; front zones also feed the brain's LCn |
| USB camera: ELP OV7725 120° or Waveshare OV9281 (global shutter) | Orange Pi USB-C | the brain's eyes; picked after a bench comparison |
| 3D-printed mounts (flow sensor, Orange Pi cover + battery deck, GPS mast) | airframe | all-up weight estimated 300–350 g |

Before flight: a bench test of the two VL53L7CX (cage net and black-box detection, box tops 1–15 cm below the
sensor, zones at the 60° edge that may see the props, map time per frame on the Orange Pi).

## Decisions pending

- **Paper framing:** from "connectome controller avoids obstacles" to "connectome controller + range shield, optimised
  with the shield in the loop"; the connectome question stays with C0/C2/C4 and the OOD tests.

## Paper

Target ICUAS 2027 (full paper due 2027-02-05), framed as a systems paper. Next gates: G1 (11-01) simulation controls,
G2 (11-22) companion GUIDED hover + processor-in-the-loop, G3 (12-20) route B on the real drone.
