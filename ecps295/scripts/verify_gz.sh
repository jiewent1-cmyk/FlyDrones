# Headless check: Gazebo Harmonic iris_runway + ArduPilot SITL (JSON) + GUIDED takeoff/hover/land
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/sim/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/sim/ardupilot_gazebo/models:$HOME/sim/ardupilot_gazebo/worlds
# render on the RTX 3070 (hybrid laptop); the NVIDIA EGL vendor file avoids harmless Mesa dri2 warnings
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
D=~/sim/runs/gz_verify; rm -rf $D; mkdir -p $D; cd $D
gz sim -s -r --headless-rendering -v3 iris_runway.sdf > gz.log 2>&1 &
GZ=$!
sleep 12
DP=~/sim/ardupilot/Tools/autotest/default_params
~/sim/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home 33.6430,-117.8420,20,0 \
  --defaults $DP/copter.parm,$DP/gazebo-iris.parm > sitl.out 2>&1 &
SP=$!
sleep 3
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
timeout 300 python ~/sim/FlyDrones/ecps295/sitl_smoke.py --alt 5 --hover 20 2>&1 | grep -v "EOF on TCP" > smoke.log
gz topic -e -t /world/iris_runway/stats -n 3 2>/dev/null | grep -E "real_time_factor" > rtf.log
kill $SP; sleep 1; kill $GZ; sleep 2
cat smoke.log rtf.log
