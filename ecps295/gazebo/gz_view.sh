# Watch ecps295_quad in the Gazebo GUI with its camera feed, flying a short demo (run on the ROG desktop).
# usage: bash gz_view.sh [world.sdf] [--no-demo]
#   world defaults to ecps295_camtest.sdf; --no-demo starts Gazebo + SITL and leaves the flying to you
#   (connect MAVProxy / QGC to tcp:127.0.0.1:5760, or run demo_flight.py yourself)
G=$(cd "$(dirname "$0")" && pwd)
WORLD=${1:-ecps295_camtest.sdf}
export DISPLAY=${DISPLAY:-:1}
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/sim/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$G/models:$G/worlds:$HOME/sim/ardupilot_gazebo/models:$HOME/sim/ardupilot_gazebo/worlds
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
D=~/sim/runs/gz_view; mkdir -p $D; cd $D

gz sim -r -v2 "$WORLD" --gui-config "$G/gui_ecps295.config" > gz.log 2>&1 &
GZ=$!
sleep 8
P=$G/..
DP=~/sim/ardupilot/Tools/autotest/default_params
~/sim/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home 33.6430,-117.8420,20,0 \
  --defaults "$DP/copter.parm,$P/sitl/course_base.parm,$G/gz_ecps295.parm" > sitl.out 2>&1 &
SP=$!
if [ "$2" != "--no-demo" ]; then
  sleep 3
  source ~/miniconda3/etc/profile.d/conda.sh && conda activate flydrones
  PYTHONPATH=$P python "$G/demo_flight.py"
  kill $SP
  echo "demo finished; close the Gazebo window to exit"
fi
wait $GZ
kill $SP 2>/dev/null
