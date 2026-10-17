# Orange Pi test data (2026-10-15 / 16)

Raw data behind the Orange Pi deployment report (`../ECPS295_OrangePi部署与性能测试报告_20261015.{zh,en}.md`).

## `bench/` — on-Pi timing (report §3–6)

- One directory per bench run, named `<date>_<time>_bench_<profile>`. Profiles P1–P9 are the CPU-placement / governor / load / thermal matrix of §4.1.
- Each directory holds `ticks.csv` (per-tick stage times), `system.csv` (CPU frequency, temperature, load) and `run.json` (summary).
- `profile_matrix.out` is the matrix summary. `sitl/` holds the SITL link tests of §6.

## `pil/` — Gazebo processor-in-the-loop (report §10.3)

One JSON line per run.

| File | What it is |
|---|---|
| `pil2.jsonl` | PIL batch with raw camera frames. |
| `pil3.jsonl` | PIL batch with JPEG camera frames, 2026-10-16 18:50–20:08. |
| `g7_sc2_tf10_local.jsonl` | The same scenes with the brain on ROG (local Gazebo, sim-clock pacing), the paired baseline. |

Fields:
- `pil_in_control_ticks` / `pil_status_n`: ticks the Pi flew vs. status messages received.
- `pil_rtt_ms_*`: network round trip, p50 / p95 / p99 / max.
- `pil_compute_ms_*`: Pi tick time, p50 / p95 / p99 / max.
- `pil_cam_kB_per_frame`: camera frame size.

A run counts as **link-healthy** when in-control ticks / status messages > 0.78 and RTT p99 < 200 ms. Only link-healthy runs enter the behaviour comparison; the others are failsafe tests (home WiFi stalls).
