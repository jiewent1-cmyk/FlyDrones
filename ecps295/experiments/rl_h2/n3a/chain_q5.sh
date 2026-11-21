#!/bin/bash
# ROG queue 5 (2026-10-15); waits for queue 4 ("q4 done"), then applies patch_simclock.py (run_g4 paced by simulated time when
# ECPS_SIMCLOCK=1; default off) and runs the reference s7_w2map with ECPS_SIMCLOCK=1 on set A (g7_w2map_sc, 58 runs, first one
# is checked). Why: run_g4 paced the brain and the run length by the wall clock while Gazebo runs at ~0.8 x real time, so every
# Gazebo run so far flew ~96 s of simulated time with the brain ticking 25 times per simulated second.
set -e
cd ~/sim/rl_h2
until grep -q "^q4 done" ~/sim/runs/chain_q4.out 2>/dev/null; do sleep 60; done
cd ecps295
md5sum run_g4.py
python3 ../n3a/patch_simclock.py
python3 -m py_compile run_g4.py
md5sum run_g4.py
cd ..
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "g7_w2map_sc__lowbox_s${k}_k${k} lowbox_s${k} s7_w2map@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "g7_w2map_sc__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; } > n3a/q5_matrix.txt
echo "launch q5 $(wc -l < n3a/q5_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q5_matrix.txt ~/sim/runs/matrix_q5.log 3
echo "q5 done $(date)"
