# Glossary

Terms, stage names and configuration names used across the reports, the lab notebook and the code.

## Fly neuroscience (MiniFly cell types)

| Term | Meaning in this project |
|---|---|
| MiniFly | ≈ 1,000-neuron hand-wired stand-in connectome from upstream FlyDrones; cell-type names and pathways follow published fly flight circuits (not real fly data) |
| LIF | leaky integrate-and-fire spiking neuron model, event-driven (`flydrones.brain.lif`) |
| R1-R6, T4a–d | photoreceptors and direction-selective motion cells, fed by the retina's flow grids |
| HS / VS | horizontal / vertical lobula-plate tangential cells (optomotor yaw and pitch) |
| LPi | lobula-plate intrinsic inhibitory cells (opponent motion) |
| LPLC2 / LC4 | looming detectors |
| PVLP | looming integrators driving DNp03; PVLP_inh implements the left/right competition |
| LAL_inh | steering inhibition onto DNg02 |
| haltere | body-rotation (gyro yaw rate) input |
| **LCb** | added in v2: "blank" cells, sudden loss of texture in the central field (plain walls) |
| **LCn** | added in v3: "near-side" cells, lateral flow asymmetry → turn away; from S5 also front ToF proximity |
| **LCv** | added in v4: ventral cells, a surface rising under the drone |
| DNg02 | descending neurons for wing-stroke amplitude → throttle and yaw |
| DNp03 | descending neurons for evasive saccades → saccade + brake |
| DNp01 | giant fibre → escape |
| **MDN** | added in v4: "moonwalker" descending neurons → retreat (backing out) |
| KC / MBON / PPL1 / PAM | Kenyon cells, mushroom-body output neuron, punishment / reward dopamine neurons (stage E) |
| efference copy | a copy of the own motor command that suppresses self-generated sensory cues (C1: blank cue after turns) |

## Software components

| Term | Meaning |
|---|---|
| EcpsRetina | camera → per-eye 6×8 grids: flow, looming, blank, near, ventral |
| VentralCue | surface height under the drone, T = baro − ToF |
| EcpsDecoder | DN rates → flight command; saccade, caution, retreat, efference-copy state machine |
| EcpsPilot | the 20 Hz tick, watchdogs, fence turn-back, struct/shield hook |
| SafetyGovernor | upstream command limits: clamps, slew, ceiling, geofence, battery, brain watchdog |
| StructGuard | S1–S3 structural rules (`ecps_struct.py`) |
| RangeShield | velocity-level control-barrier-function filter on ToF ranges (S4) |
| LocalMap | S7 local 3D occupancy map from the two 8×8 ToF boards (`ecps_map.py`, pending push) |
| MavlinkTwin / `mavlink_twin.py` | ECPS MAVLink backend: route B, companion heartbeat, stream requests, simulated ToF |
| fast twin | `rl/twin.py`, the kinematic simulator used for screening and CMA-ES |
| gates / anchors | `rl.gates`: sensory gate, smoke tests and bit-identical result hashes across machines |

## Flight control and navigation

| Term | Meaning |
|---|---|
| route B | the companion sends height above the *floor* as ExtNav z (`VISION_POSITION_ESTIMATE`); fixes the low-box height corruption |
| ExtNav | ArduPilot external navigation source (`EK3_SRC1_POSZ 6`, `VISO_TYPE 1`) |
| EKF3 | ArduPilot's state estimator; here fused optical flow + ToF + ExtNav z + IMU + compass |
| GUIDED | ArduPilot mode that accepts velocity targets from the companion |
| MSP | MultiWii Serial Protocol; how the 3901-L0X talks to the FC |
| CRSF | ELRS receiver protocol |
| companion failsafe | FC treats the companion (sysid 254) as a GCS; heartbeat lost 2 s → land |
| SITL | ArduPilot software-in-the-loop |
| lock-step / RTF | Gazebo and SITL advance together; real-time factor; runs need RTF ≥ 0.95 |
| sim clock | `ECPS_SIMCLOCK`: Python time driven by Gazebo frame stamps, so runs are valid at any RTF |
| PIL | processor-in-the-loop: the control stack on the Orange Pi with recorded inputs |
| CBF | control barrier function; here the half-plane constraint v·uᵢ ≤ α(dᵢ − d_safe) |

## Worlds and metrics

| Term | Meaning |
|---|---|
| cage20 / cage10 | Gazebo models of the 20 ft / 10 ft course cages (net strands, mat, boxes, lamps) |
| lowbox_s0–49 | 50 test worlds with low boxes (0.2–0.75 m) and a 1.1 m box stack, 1.0–2.3 m from take-off |
| empty | cage without obstacles; every saccade is a false alarm |
| contact / runs with contact | any collision / number of runs with at least one |
| near miss | approach closer than the near-miss radius without contact |
| clearance | minimum distance to obstacles (5 % quantile across runs in some tables) |
| coverage | share of cage cells visited in a patrol |
| shield ticks | ticks in which the shield changed the command (clamp / retreat / unstick) |
| OOD modes | out-of-distribution tests: `notex`, `lowlight`, `halfcam`, `wind`, `tof` faults, `ghost` (invisible obstacles) |

## Stages (chronological)

| Stage | Content | Report |
|---|---|---|
| S4–S5, G1–G4 | twin layers, Gazebo airframe, camera, cages, MiniFly flying by the camera | 1 |
| S2 | custom MiniFly v1 → v4 | 1 |
| S7d / S7e / S7b | optical-flow navigation, route B, companion faults | 2 |
| S8 (E1–E3) | ablation, 1,260 flights | 3 |
| A0–A4, B1–B3 | CMA-ES in the GPS cage, control arms | 4 |
| H0–H3 | hardware alignment: Gazebo gate, twin calibration, re-optimisation, acceptance | 5 |
| C0 / C1 / C2 / C4 | brain off and no looming / efference copy / shuffled wiring / bypass vs intact | 5, 6 |
| D | out-of-distribution tests | 6 |
| E | mushroom-body plasticity (negative) | 8 |
| S1–S3 | structural rules without a forward sensor | 7 |
| S4 / S5 | range shield / ToF into the brain | 7 |
| S6 / S7 | cheaper sensor layouts / 8×8 ToF + local 3D map | 7 §8 |
| N1–N6 | route review: larger samples, N3 CMA with the shield, N4 stress tests, N5 pre-registered acceptance, N6 bench + real drone | 7 |
| R0–R6, G1–G4 | real-drone test stages and milestones | [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md) |

## Configuration names (`minifly/*.yaml`, shield conditions)

| Name | Meaning |
|---|---|
| v0 … v4, v1_1 … v3_2 | MiniFly generations ([ARCHITECTURE.md](ARCHITECTURE.md) §3) |
| v4_eff | v4 + C1 blank-cue efference copy; default since 10-13 |
| es2, es3 | CMA-ES decoders from the GPS cage (report 4) |
| h2_s1 … s8 | CMA on v4 with hardware caps (intact arm); h2_s3 accepted as conservative variant |
| c4_s1 … s8 | bypass arm (no LIF network) |
| s1_noback, s2_mem, s2_memw, s3_rng, s23, s12 | structural-rule variants |
| sh5u | shield with 5 single-zone ToF (±25° front pair + right/rear/left) and unstick |
| shfu | shield with an 8×8 front array + right/rear/left |
| s5 | sh5u + ToF into the brain (LCn) |
| f3b, m2, w2, w1map | S6/S7 layouts: 3 × VL53L1X; front + rear VL53L5CX; front + rear VL53L7CX without map; front only + map |
| **w2map** | **final: front + rear VL53L7CX + local 3D map + shield** |
