#!/bin/bash
# ROG queue 6 (2026-10-15 evening): simulated-time pacing (ECPS_SIMCLOCK=1 + max_flight fix) reference vs tf10, sets A and B
set -e
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
lines() {
  if [ "$2" = A ]; then for k in $(seq 0 49); do echo "$1__lowbox_s${k}_k${k} lowbox_s${k} s7_w2map@intact ${k} $A"; done; for k in $(seq 0 7); do echo "$1__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done
  else for k in $(seq 0 49); do echo "$1__lowbox_s${k}_k$((k+100)) lowbox_s${k} s7_w2map@intact $((k+100)) $A"; done; for k in $(seq 8 15); do echo "$1__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; fi; }
for st in A B; do
  lines g7_sc2 $st > n3a/q6r$st.txt; lines g7_sc2_tf10 $st > n3a/q6t$st.txt
  echo "launch ref $st $(date)"; NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q6r$st.txt ~/sim/runs/matrix_q6r$st.log 3
  echo "launch tf10 $st $(date)"; NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_par_mine.sh n3a/q6t$st.txt ~/sim/runs/matrix_q6t$st.log 3
done
echo "q6 done $(date)"
