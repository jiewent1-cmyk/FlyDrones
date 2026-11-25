#!/bin/bash
# pilot for e7b: which flow error gives EKF drift clearly above the landmark error (0.11 m)?  one run per variant, landmark off
cd ~/sim/abl/gz
E=~/sim/wt/rtl/ecps295
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow ECPS_QUIET_OFF=1
B=$E/sitl/variants/G_extnav_height.parm,$E/sitl/variants/companion_fs.parm,$E/sitl/variants/H_terrain_fast.parm
printf "SIM_FLOW_RND 0.5\n" > fl_a.parm
printf "SIM_FLOW_RND 0.5\nFLOW_FXSCALER 150\nFLOW_FYSCALER -150\n" > fl_b.parm
printf "SIM_FLOW_RND 1.5\n" > fl_c.parm
printf "SIM_FLOW_RND 0.5\nFLOW_FXSCALER 150\nFLOW_FYSCALER -150\nSIM_FLOW_DELAY 2\n" > fl_d.parm
L=$(grep "^e7__cage20_k0_off " e7_matrix.txt)
for pair in "a b" "c d"; do
  k=0
  for v in $pair; do
    echo "${L/e7__cage20_k0_off/e7p__$v}" > p_$v.txt
    ( sleep $((k * 20)); INST=$k EXTRA_PARM=$B,$PWD/fl_$v.parm bash trk/run_matrix_trk.sh p_$v.txt ~/sim/runs/matrix_e7p_$v.log ) &
    k=$((k + 1))
  done
  wait
done
echo "$(date +%T) e7 pilot finished" >> ~/sim/abl/chain.log
