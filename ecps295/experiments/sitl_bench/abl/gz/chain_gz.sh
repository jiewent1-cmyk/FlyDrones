#!/bin/bash
# Gazebo experiments after the SITL ablation queue has finished (2 instances; 3 drop the real-time factor below 0.95)
cd ~/sim/abl/gz
until grep -q "r3d batch finished" ~/sim/abl/chain.log; do sleep 60; done
while pgrep -x arducopter > /dev/null; do sleep 5; done
E=~/sim/wt/rtl/ecps295
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow ECPS_QUIET_OFF=1
export EXTRA_PARM=$E/sitl/variants/G_extnav_height.parm,$E/sitl/variants/companion_fs.parm,$E/sitl/variants/H_terrain_fast.parm
mkdir -p ~/sim/runs
for n in e8a e6 e7; do
  echo "$(date +%T) gz $n start" >> ~/sim/abl/chain.log
  bash $E/scripts/trk/run_matrix_par_trk.sh ${n}_matrix.txt ~/sim/runs/matrix_$n.log 2 > $n.out 2>&1
  echo "$(date +%T) gz $n finished" >> ~/sim/abl/chain.log
done
