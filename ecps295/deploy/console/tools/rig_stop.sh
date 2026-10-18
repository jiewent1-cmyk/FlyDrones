#!/bin/sh
# Stop the rig started by rig_start.sh: SIGTERM to its PID, the rig stops SITL, the router and the companion (and with
# --deployed starts mavlink-router.service again and puts the CPU governor back).
cd "$(dirname "$0")"
[ -f rig.pid ] && kill -TERM "$(cat rig.pid)" 2>/dev/null && echo "stopping rig $(cat rig.pid)"
for i in $(seq 1 25); do kill -0 "$(cat rig.pid 2>/dev/null)" 2>/dev/null || break; sleep 1; done
rm -f rig.pid
tail -4 rig.log
