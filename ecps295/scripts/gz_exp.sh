# usage: gz_exp.sh <world.sdf> <tag> "<python script + args>" [extra .parm...]
# Headless Gazebo (GPU) + ArduPilot SITL over JSON + one experiment script; outputs in ~/sim/runs/<tag>/
WORLD=$1; TAG=$2; CMD=$3; shift 3
G=~/sim/FlyDrones/ecps295/gazebo
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/sim/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$G/models:$G/worlds:$HOME/sim/ardupilot_gazebo/models:$HOME/sim/ardupilot_gazebo/worlds
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
D=~/sim/runs/$TAG; rm -rf $D; mkdir -p $D; cd $D
gz sim -s -r --headless-rendering -v3 $WORLD > gz.log 2>&1 &
GZ=$!
sleep 10
DEF=~/sim/ardupilot/Tools/autotest/default_params/copter.parm
for f in "$@"; do DEF="$DEF,$f"; done
~/sim/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I0 --home 33.6430,-117.8420,20,0 --defaults "$DEF" > sitl.out 2>&1 &
SP=$!
sleep 3
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
PYTHONPATH=~/sim/FlyDrones/ecps295 timeout 900 python $CMD 2>&1 | grep -v "EOF on TCP" > exp.log
WN=$(basename $WORLD .sdf)
gz topic -e -t /world/$WN/stats -n 5 2>/dev/null | grep real_time_factor | awk "{s+=\$2;n++} END {if(n) printf \"RTF mean %.3f over %d samples\n\", s/n, n}" > rtf.log
kill $SP; sleep 1; kill $GZ; sleep 2
cat exp.log rtf.log
