# Gazebo ground-console session (2026-10-21)

Fly the Gazebo drone from the ECPS Console app.
- **ROG:** Gazebo + SITL. Instance 5 runs the `*_monitor` world, whose airframe has the chase camera. `run_g4.py --mode console` plays the RC pilot: it takes off, then hands over with PIL go = 1. 10 s after each disarm it takes off again.
- **Pi:** the real companion + console link from `/root/pil_hover`, plus a temporary mavlink-router: SITL serial2 → companion 14563 + GCS 14550.
- **Mac:** the app uses its default ports (ECL 14700, MAVLink 14550).

The chase camera (480 px JPEG, 15 Hz) reaches the Pi as PIL kind 5 and shows in the console's **colour** camera view.

    gzc.sh start [world] [minutes]   # default cage20 60
    gzc.sh status
    gzc.sh stop                      # run_g4 lands, instance torn down, Pi router/governor restored

Files outside this tree:
- `pil_link_session_chase.diff`: the Pi test copy vs `/opt` (CTL aux bit 0 = session, kind 5 chase → `cam.last_bgr`). It belongs to the console line (feat/console); `/opt` is not changed.
- `gz_g4_mine_gzc.diff`: ROG launcher changes (the monitor airframe per instance, the chase stream, `RUN_TIMEOUT`, line-buffered `exp.log`).
