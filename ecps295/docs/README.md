# ecps295 documentation

Reference documents for the ECPS 295 project (UCI, Fall 2026): a MiniFly connectome brain flying the 2S course
quadrotor without GPS, with a range-sensing safety shield. Results and statistics are in the
[reports](../reports/README.md); the chronological lab notebook is [../README.md](../README.md).

| Document | Read it for |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | system layers, MiniFly generations v0 → v4, system generations A → E, safety layer, route B, wiring, Orange Pi / flight-controller deployment, MAVLink traffic |
| [HARDWARE.md](HARDWARE.md) | parts list, pin-out, power, mounts, weight budget, bench tests |
| [FLIGHT_CONTROLLER.md](FLIGHT_CONTROLLER.md) | custom firmware, serial ports, EKF3 without GPS, route-B parameters, fences, failsafes |
| [SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) | installing the fast twin and Gazebo + SITL, running screens, CMA-ES and Gazebo matrices, reproducing figures |
| [FLIGHT_TEST_PLAN.md](FLIGHT_TEST_PLAN.md) | bench → PIL → R0–R6 cage stages, safety rules, ground truth, milestones G1–G4 |
| [GLOSSARY.md](GLOSSARY.md) | cell types, components, stage names (S/H/C/D/E/N/R), configuration names |

Status as of 2026-10-15: simulation work through the hardware decision (w2map) is done; the S6/S7 map code and N3
round 2 are still on the experiment machines; nothing has flown on the real drone yet.
