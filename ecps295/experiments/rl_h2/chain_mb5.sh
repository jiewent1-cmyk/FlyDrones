#!/bin/bash
# 2026-10-13: after the mb_v4 matrix, the s5 arm (cone ToF) on the same new pairs: lowbox_s0-49 at brain seed k+100, cage20 k8-15
set -e
until grep -q "^DONE" ~/sim/runs/matrix_mb.log 2>/dev/null; do sleep 60; done
cd ~/sim/rl_h2
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "mb_s5__lowbox_s${k}_k$((k + 100)) lowbox_s${k} s5@intact $((k + 100)) $A"; done
  for k in $(seq 8 15); do echo "mb_s5__cage20_k${k} cage20 s5@intact ${k} $A"; done; } > mb5_matrix.txt
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh mb5_matrix.txt ~/sim/runs/matrix_mb5.log 3
