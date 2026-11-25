#!/bin/bash
# e7b: e7 again with SIM_FLOW_RND 0.25 (5x the baseline). One Gazebo instance: the GPU is shared with another job
# (a second instance segfaults in OGRE for lack of GPU memory).
cd ~/sim/abl/gz
until grep -q "r6 batch finished" ~/sim/abl/chain.log; do sleep 30; done
while pgrep -x arducopter > /dev/null; do sleep 5; done
E=~/sim/wt/rtl/ecps295
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow ECPS_QUIET_OFF=1
export EXTRA_PARM=$E/sitl/variants/G_extnav_height.parm,$E/sitl/variants/companion_fs.parm,$E/sitl/variants/H_terrain_fast.parm,$PWD/fl_25.parm
sed "s/^e7__/e7b__/" e7_matrix.txt > e7b_matrix.txt
echo "$(date +%T) gz e7b start" >> ~/sim/abl/chain.log
bash trk/run_matrix_par_trk.sh e7b_matrix.txt ~/sim/runs/matrix_e7b.log 1
echo "$(date +%T) gz e7b finished" >> ~/sim/abl/chain.log
