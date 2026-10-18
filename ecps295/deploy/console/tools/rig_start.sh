#!/bin/sh
# Start tools/sitl_rig.py on the Pi in the background (PID in rig.pid); stop it with rig_stop.sh (never pkill -f over ssh).
#   rig_start.sh [seconds] [sitl_rig.py options, e.g. --deployed]
cd "$(dirname "$0")"
S=${1:-1800}
[ $# -gt 0 ] && shift
nohup setsid python3 sitl_rig.py --work "$(pwd)" --seconds "$S" "$@" > rig.log 2>&1 < /dev/null &
echo $! > rig.pid
echo "rig pid $(cat rig.pid)"
