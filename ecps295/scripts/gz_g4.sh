# usage: gz_g4.sh <world.sdf> <tag> "<run_g4.py args>" [--gui]   (add --live to the args to open monitor.py)
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
~/sim/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home 33.6430,-117.8420,20,0 \
  --defaults "$DP/copter.parm,$E/sitl/course_base.parm,$G/gz_ecps295.parm" > sitl.out 2>&1 &
SP=$!
sleep 3
LAYOUT=$G/worlds/$(basename $WORLD .sdf).layout.json
[ -f "$LAYOUT" ] && ARGS="$ARGS --layout $LAYOUT"
# sample Gazebo real-time factor while flying (TechRoute §6A: RTF < 0.95 invalidates a run)
WN=$(basename $WORLD .sdf)
(sleep 60; gz topic -e -t /world/$WN/stats -n 40 2>/dev/null | grep real_time_factor | awk '{print $2}' | sort -g \
  | awk '{v[NR]=$1; s+=$1} END {if (NR) printf "RTF mean %.3f median %.3f min %.3f over %d samples\n", s/NR, v[int((NR+1)/2)], v[1], NR}' > rtf.log) &
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
PYTHONPATH=$E MPLBACKEND=Agg timeout 900 python $E/run_g4.py $ARGS 2>&1 | grep -v "EOF on TCP" > exp.log
kill $SP $BR; sleep 1
if [ "$GUI" != "--gui" ]; then kill $GZ; sleep 2; fi
cat exp.log rtf.log 2>/dev/null
