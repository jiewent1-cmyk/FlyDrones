#!/bin/bash
# S7c 2026-10-14 22:35 PT (hardware fixed to w2map): speed stress of the map shield in Gazebo - s7_w2map with the v4 decoder at
# cruise / max_forward 0.65 (instead of 0.5) on lowbox_s0-49 (brain seed k) + cage20 k0-7, paired with g7_w2map. Reference for
# N3 candidates that raise cruise. Runs after matrix_s7b; run_matrix_par_mine.sh pauses 00:55-07:00 PT (quiet hours).
set -e
until grep -q "^DONE" ~/sim/runs/matrix_s7b.log 2>/dev/null; do sleep 60; done
cd ~/sim/rl_h2
A="--mode patrol --seconds 120 --cruise 0.65 --max-forward 0.65 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "g7_w2map_c065__lowbox_s${k}_k${k} lowbox_s${k} s7_w2map@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "g7_w2map_c065__cage20_k${k} cage20 s7_w2map@intact ${k} $A"; done; } > s7c_matrix.txt
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh s7c_matrix.txt ~/sim/runs/matrix_s7c.log 3
