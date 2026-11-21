#!/bin/bash
# S7b 2026-10-14 22:15 PT: replicate the map shield on the second brain-seed set (lowbox_s0-49 at brain seed k+100, cage20 k8-15),
# paired with mb_v4__* / mb_s5__*. Rows interleaved g7_w2map / g7_w1map (two boards + map vs front board + map + backing corridor).
set -e
cd ~/sim/rl_h2
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do for c in w2map w1map; do echo "g7_${c}__lowbox_s${k}_k$((k + 100)) lowbox_s${k} s7_${c}@intact $((k + 100)) $A"; done; done
  for k in $(seq 8 15); do for c in w2map w1map; do echo "g7_${c}__cage20_k${k} cage20 s7_${c}@intact ${k} $A"; done; done; } > s7b_matrix.txt
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh s7b_matrix.txt ~/sim/runs/matrix_s7b.log 3
