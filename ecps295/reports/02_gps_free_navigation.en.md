# Report 2 — Flying without GPS: optical flow, the low-box problem and route B

> Branch `ecps295-sim` · experiments S7d / S7e / S7b, 2026-10-03 … 10-05 · ROG, Gazebo 8.15 + SITL 4.7 (patched)
> 中文版：[02_gps_free_navigation.zh.md](02_gps_free_navigation.zh.md)

## Abstract

The real drone has no GPS. It navigates on a Matek 3901-L0X (PMW3901 optical flow + VL53L0X ToF, both facing down). In
the twin, flying low over 0.46–0.6 m boxes **pulled the flight controller's height estimate down by 0.4–1.0 m**. The
FC climbed, crossed the 1.3 m hard fence and landed itself. Tuning the EKF terrain parameters did not help. We fixed
it from two sides:
**route B**, where the companion computer sends height above the *floor* to ArduPilot as an external-navigation
height, and **MiniFly v4**, a ventral pathway (LCv → MDN) that backs away from a surface rising under the drone.
Route B removes all FC landings and keeps the height error at 0.10–0.15 m. With v4 + route B there were no
contacts in 5 random low-box cages. Fault injection (S7b) then exposed three unprotected failure modes, and each got
a dedicated safety mechanism.

## 1. Setup

- `NAV=flow`: EKF3 on optical flow + ToF (`sitl/fhb_delta.parm`), compass declination fixed (`COMPASS_AUTODEC 0`)
  because without GPS the EKF aligned yaw 11° off.
- Take-off without GPS: EKF origin set, lift-off in ALT_HOLD with an RC override from sysid 255, switch to GUIDED at
  0.1 m, climb until ToF or EKF reaches the target (the EKF lagged the ToF by up to 0.8 m).
- All scores use simulator truth (`SIM_STATE`); the EKF estimate is logged next to it.
- Five random low-box cages `ecps295_lowbox_s0..4`: 6 boxes, 0.2–0.75 m tall or a 1.1 m stack, 1.0–2.3 m from
  take-off.

## 2. S7d — what breaks

120 s v3 patrols, 20 ft cage, 5 seeds:

| Configuration | Runs with contact | Path m | EKF xy error max (median / max) | Runs > 1.3 m | Runs on the floor |
|---|---|---|---|---|---|
| GPS | 1 | 25.2 | 0.05 / 0.05 | 0 | 0 |
| flow, ToF primary height | 1 | 12.6 | 0.42 / 0.77 | 4 | 4 |
| flow, baro only | 0 | 18.7 | 0.67 / 4.80 | 2 | 2 |
| flow, baro + EKF terrain tuning | 3 | 18.7 | 0.61 / 1.56 | 2 | 1 |

**Mechanism.** When a box passes under the drone, the ToF range drops (0.83 → 0.40 m). With the ToF as primary height
this pulls the EKF down directly. With baro only, the box still gets in through the flow fusion's terrain state, even
though the baro height itself stays correct. The controller "corrects" by climbing.

## 3. S7e — two fixes

**Ventral cue.** Surface height *T = baro height − ToF range*. The drone's own climb or sink moves both terms
together, so *T* changes only when the surface underneath changes. A fast path catches steps ≥ 0.10 m faster than
0.6 m/s. Offline on the S7d logs, it detected 90 % of box crossings, with a median latency of 0.2 s.

**MiniFly v4.** v3 + 16 LCv cells per side → 2 MDN per side ("moonwalker" descending neurons, which drive backward
walking in *Drosophila*). The decoder's `retreat` backs the drone out along its path until MDN is quiet for 1 s, then
saccades. An earlier version fed LCv into PVLP like LCb did, and the drone saccaded in place over the box for 5 s.

**Route B (companion height).** At 20 Hz, the companion sends `VISION_POSITION_ESTIMATE` z = height above the floor.
This is the tilt-corrected ToF while nothing is under the drone, and baro minus a frozen floor baseline while
something is. On the FC side this needs `EK3_SRC1_POSZ 6` and `VISO_TYPE 1`. The EKF terrain state then treats the
box as terrain, which is what it is.

![Route B and v4](figures/fig5_s7e_route_b.png)

*Figure 2. S7e, 5 random low-box cages per group, no GPS. Route B takes FC landings from 4/5 to 0/5 and the median
height error from ~1 m to 0.1 m. v4 adds a second layer: fewer contacts and a third of the time over boxes.*

| Brain | FC height | Contacts | Near misses | Path m | Over boxes s | Max true height m | FC landings |
|---|---|---|---|---|---|---|---|
| v3 | flow EKF | 5 | 7 | 7.9 | 9.2 | 1.47 | 4 |
| v4 | flow EKF | 1 | 1 | 7.8 | 3.9 | 1.45 | 4 |
| v3 | route B | 1 | 2 | 26.4 | 24.2 | 1.02 | 0 |
| v4 | route B | **0** | 4 | 23.6 | 8.3 | 1.00 | **0** |

A tempting shortcut failed: a `height_guard` in EcpsPilot that commands descent above 1.0 m. ArduPilot executes the
velocity command in its own corrupted height frame, so the guard could not stop the climb.

## 4. S7b — companion faults

Route B makes the companion part of the height loop, so its failures need handling. The first round (fault at 30 s)
showed:

- **Height messages stop and later resume** → ArduPilot falls back to baro and returns to ExtNav within 2 cm. No
  issue.
- **Height messages stop for good while the brain keeps flying** → 4/5 FC landings at up to 2.17 m. On baro, a box
  pulled the EKF from 0.87 m to −0.04 m in 2 s.
- **Companion hangs** → GUIDED times out and the drone hovers on baro until the battery runs out.
- **v4 retreat ran 4 s along a box edge and reversed into an obstacle.** Nothing looks backwards.

The fixes were a height-source watchdog in EcpsPilot (hover at 0.5 s, land at 3 s), `retreat.max_s = 2.0`, and
`companion_fs.parm` (the companion's sysid 254 counts as a GCS; `FS_GCS_ENABLE 5` lands 2 s after its heartbeat
stops). We tested each with and without, in 10 cages (experiment E3 of S8):

![Fault injection](figures/fig4_e3_fault_injection.png)

*Figure 3. Fault at t = 30 s, 10 low-box cages each, paired exact McNemar.*

## 5. Conclusions

1. With a flow + ToF sensor, low obstacles under the drone corrupt the flight controller's height. Better EKF tuning
   does not fix this; the fix is to give the FC a height above the floor.
2. Route B is what makes GPS-free flight possible: in S8 its removal drops mission success from 82 % to 10 % (Report 3).
3. The companion then becomes safety-critical. The two watchdogs and the FC failsafe make every tested companion fault
   end in a hover or a landing.
4. Under nominal route B, v4's ventral pathway mostly shortens the time over boxes. Its value is as a behavioural
   backup for when the height estimate fails.

## 6. Limitations

SITL baro noise is ±0.2 m uniform, with no prop wash or thermal drift, so the "baro − floor baseline" branch of route
B will be worse on the real drone. The 3901-L0X output rate and latency and the VL53L0X edge mixing still need bench
measurements. The ExtNav latency on the Orange Pi has not been measured yet.
