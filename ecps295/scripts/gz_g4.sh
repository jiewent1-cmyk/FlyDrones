# usage: gz_g4.sh <world.sdf> <tag> "<run_g4.py args>" [--gui]   (add --live to the args to open monitor.py)
#   NAV=gps (default)  GPS + baro EKF (course_base.parm)
#   NAV=flow           Matek 3901-L0X as on the real drone: optical flow + ToF height, no GPS (sitl/fhb_delta.parm)
#   EXTRA_PARM=a.parm,b.parm  appended last (e.g. a SIM_FLOW_RATE variant)
# Gazebo (headless, or GUI on the ROG desktop with --gui) + camera bridge + ArduPilot SITL + run_g4.py
WORLD=$1; TAG=$2; ARGS=$3; GUI=$4
E=~/sim/FlyDrones/ecps295
G=$E/gazebo
# watching (--live): use the *_monitor world (adds the chase camera for monitor.py, RTF ~0.96);
# plain worlds have no view cameras so experiment runs keep RTF 1.0
case "$ARGS" in *--live*) MW=$(basename $WORLD .sdf)_monitor.sdf; [ -f "$G/worlds/$MW" ] && WORLD=$MW ;; esac
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/sim/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$G/models:$G/worlds:$HOME/sim/ardupilot_gazebo/models:$HOME/sim/ardupilot_gazebo/worlds
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
# a previous run that has not finished tearing down holds SITL tcp 5760 / plugin udp 9002 / tick stream 5799 and
# leaves a second Gazebo server publishing the same camera topic (seen: "bind failed on port 5760", 97 frames/s)
ports_busy() { ss -ltnu 2>/dev/null | grep -qE ":(5760|9002|5799) "; }
pkill -x arducopter 2>/dev/null; pkill -f "^gz sim -s" 2>/dev/null; pkill -f "^/usr/bin/python3 .*cam_bridge.py" 2>/dev/null
for i in $(seq 1 30); do ports_busy || break; sleep 0.5; done
ports_busy && { echo "ports 5760/9002/5799 still busy, giving up" >&2; exit 1; }
D=~/sim/runs/$TAG; rm -rf $D; mkdir -p $D; cd $D
if [ "$GUI" = "--gui" ]; then
  export DISPLAY=${DISPLAY:-:1}
  gz sim -r -v2 $WORLD --gui-config $G/gui_ecps295.config > gz.log 2>&1 &
else
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  gz sim -s -r --headless-rendering -v2 $WORLD > gz.log 2>&1 &
fi
GZ=$!
sleep 10
rm -f /dev/shm/ecps295_cam
/usr/bin/python3 $G/cam_bridge.py > bridge.log 2>&1 &
BR=$!
DP=~/sim/ardupilot/Tools/autotest/default_params
PARMS="$DP/copter.parm,$E/sitl/course_base.parm,$G/gz_ecps295.parm"
NAV=${NAV:-gps}
[ "$NAV" = flow ] && PARMS="$PARMS,$E/sitl/fhb_delta.parm"
[ -n "$EXTRA_PARM" ] && PARMS="$PARMS,$EXTRA_PARM"
ARGS="$ARGS --nav $NAV"
echo "NAV=$NAV params: $PARMS" > params.log
~/sim/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home 33.6430,-117.8420,20,0 \
  --defaults "$PARMS" > sitl.out 2>&1 &
SP=$!
sleep 3
LAYOUT=$G/worlds/$(basename $WORLD .sdf).layout.json
[ -f "$LAYOUT" ] && ARGS="$ARGS --layout $LAYOUT"
# sample Gazebo real-time factor while flying (TechRoute §6A: RTF < 0.95 invalidates a run)
WN=$(basename $WORLD .sdf)
# sim and wall clock from one stats message ("<sim s> <real s>"); the whole-run RTF from two of these is the number to
# trust: the 40-sample window below lasts ~8 s and a single lockstep hiccup moved its mean by 0.1-0.4
clocks() { timeout 5 gz topic -e -t /world/$WN/stats -n 1 2>/dev/null | awk '/^(sim_time|real_time) \{/ {b=$1} /sec:/ && b {v[b]+=($1=="nsec:")?$2/1e9:$2} /^\}/ {b=""} END {printf "%.3f %.3f", v["sim_time"], v["real_time"]}'; }
(sleep 40; clocks > clock0.txt) &
(sleep 60; gz topic -e -t /world/$WN/stats -n 40 2>/dev/null | grep real_time_factor | awk '{print $2}' | sort -g \
  | awk '{v[NR]=$1; s+=$1} END {if (NR) printf "RTF mean %.3f median %.3f min %.3f over %d samples\n", s/NR, v[int((NR+1)/2)], v[1], NR}' > rtf.log) &
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
PYTHONPATH=$E MPLBACKEND=Agg timeout 900 python $E/run_g4.py $ARGS 2>&1 | grep -v "EOF on TCP" > exp.log
clocks > clock1.txt
read s0 r0 < clock0.txt 2>/dev/null; read s1 r1 < clock1.txt 2>/dev/null
[ -n "$r1" ] && [ -n "$r0" ] && awk -v a=$s0 -v b=$s1 -v c=$r0 -v d=$r1 'BEGIN {if (d > c) printf "RTF run %.3f over %.0f s\n", (b-a)/(d-c), d-c}' >> rtf.log
wait_gone() { for i in $(seq 1 20); do kill -0 "$@" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$@" 2>/dev/null; }
kill $SP $BR 2>/dev/null; wait_gone $SP $BR
if [ "$GUI" != "--gui" ]; then kill $GZ 2>/dev/null; wait_gone $GZ; pkill -f "^gz sim -s" 2>/dev/null; fi
cat exp.log rtf.log 2>/dev/null
