#!/bin/bash
# usage: gz_inst.sh <instance I> <world.sdf> <tag> "<run_g4.py args>"
# One isolated G4 run (Gazebo server + camera bridge + ArduPilot SITL + run_g4.py) so many can run side by side:
#   SITL -I I (MAVLink tcp 5760+10I, JSON physics udp 9002+10I), a per-instance model copy whose plugin listens on
#   9002+10I, GZ_PARTITION per instance (topics), camera shared memory /dev/shm/ecps_inst<I>_cam. Kills only its own
#   processes. NAV=gps|flow, EXTRA_PARM=a.parm,b.parm as gz_g4.sh.
I=$1; WORLD=$2; TAG=$3; ARGS=$4
S=~/sim; E=${ECPS_E:-$S/ecps295}; G=$E/gazebo  # ECPS_E: another ecps295 tree (e.g. ecps295_v4)
source $S/miniforge/etc/profile.d/conda.sh
export GZ_VERSION=harmonic GZ_PARTITION=ecps_inst$I GZ_IP=127.0.0.1
INST=/tmp/ecps_inst$I; rm -rf $INST; mkdir -p $INST/models
cp -r $G/models/ecps295_quad $INST/models/
sed -i "s#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>$((9002 + 10 * I))</fdm_port_in>#" $INST/models/ecps295_quad/model.sdf
export GZ_SIM_SYSTEM_PLUGIN_PATH=$S/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$INST/models:$G/models:$G/worlds:$S/ardupilot_gazebo/models:$S/ardupilot_gazebo/worlds
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
D=$S/runs/$TAG; rm -rf $D; mkdir -p $D; cd $D
conda activate gz
LD_PRELOAD=/lib/x86_64-linux-gnu/libsystemd.so.0 gz sim -s -r --headless-rendering -v2 $WORLD > gz.log 2>&1 &  # conda libsystemd needs glibc > 2.35
GZ=$!
sleep 10
SHM=/dev/shm/ecps_inst${I}_cam; rm -f $SHM
python $G/cam_bridge.py --stream /ecps295/camera:$SHM:192x144:gray > bridge.log 2>&1 &
BR=$!
DP=$S/ardupilot/Tools/autotest/default_params
PARMS="$DP/copter.parm,$E/sitl/course_base.parm,$G/gz_ecps295.parm"
NAV=${NAV:-gps}
[ "$NAV" = flow ] && PARMS="$PARMS,$E/sitl/fhb_delta.parm"
[ -n "$EXTRA_PARM" ] && PARMS="$PARMS,$EXTRA_PARM"
echo "I=$I NAV=$NAV params: $PARMS" > params.log
$S/ardupilot/build/sitl/bin/arducopter --model JSON --speedup 1 -I$I --home 33.6430,-117.8420,20,0 --defaults "$PARMS" > sitl.out 2>&1 &
SP=$!
sleep 3
LAYOUT=$G/worlds/$(basename $WORLD .sdf).layout.json
[ -f "$LAYOUT" ] && ARGS="$ARGS --layout $LAYOUT"
WN=$(basename $WORLD .sdf)
clocks() { timeout 5 gz topic -e -t /world/$WN/stats -n 1 2>/dev/null | awk '/^(sim_time|real_time) \{/ {b=$1} /sec:/ && b {v[b]+=($1=="nsec:")?$2/1e9:$2} /^\}/ {b=""} END {printf "%.3f %.3f", v["sim_time"], v["real_time"]}'; }
(sleep 40; clocks > clock0.txt) &
conda activate fly
SHM_ARG=""; grep -q -- '"--shm"' $E/run_g4.py && SHM_ARG="--shm $SHM"  # newer run_g4 (v4 tree) takes the shm path as an argument
ECPS_SIMCLOCK=$SHM ECPS_SHM=$SHM PYTHONPATH=$E:$S/src MPLBACKEND=Agg OMP_NUM_THREADS=1 timeout 1800 python $E/run_g4.py $ARGS --nav $NAV --url tcp:127.0.0.1:$((5760 + 10 * I)) --port 0 $SHM_ARG 2>&1 | grep -v "EOF on TCP" > exp.log
conda activate gz
clocks > clock1.txt
read s0 r0 < clock0.txt 2>/dev/null; read s1 r1 < clock1.txt 2>/dev/null
[ -n "$r1" ] && [ -n "$r0" ] && awk -v a=$s0 -v b=$s1 -v c=$r0 -v d=$r1 'BEGIN {if (d > c) printf "RTF run %.3f over %.0f s\n", (b-a)/(d-c), d-c}' >> rtf.log
wait_gone() { for i in $(seq 1 20); do kill -0 "$@" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$@" 2>/dev/null; }
kill $SP $BR $GZ 2>/dev/null; wait_gone $SP $BR $GZ
pkill -P $GZ 2>/dev/null
rm -f $SHM; rm -rf $INST
cat rtf.log 2>/dev/null
