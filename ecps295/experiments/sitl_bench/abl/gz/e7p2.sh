#!/bin/bash
# second pilot: SIM_FLOW_RND 0.15 / 0.25 (0.5 gave 1.6 m EKF error and the tracker lost the hold), landmark off + on
cd ~/sim/abl/gz
until grep -q "r6 smoke finished" ~/sim/abl/chain.log; do sleep 20; done
while pgrep -x arducopter > /dev/null; do sleep 5; done
E=~/sim/wt/rtl/ecps295
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow ECPS_QUIET_OFF=1
B=$E/sitl/variants/G_extnav_height.parm,$E/sitl/variants/companion_fs.parm,$E/sitl/variants/H_terrain_fast.parm
for v in 15 25; do
  printf "SIM_FLOW_RND 0.$v\n" > fl_$v.parm
  grep "^e7__cage20_k0_" e7_matrix.txt | sed "s/^e7__/e7q${v}__/" > q_$v.txt
  EXTRA_PARM=$B,$PWD/fl_$v.parm bash trk/run_matrix_par_trk.sh q_$v.txt ~/sim/runs/matrix_e7q$v.log 2
done
echo "$(date +%T) e7 pilot2 finished" >> ~/sim/abl/chain.log
