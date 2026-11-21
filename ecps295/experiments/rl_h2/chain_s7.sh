#!/bin/bash
# S7 2026-10-14: Gazebo check of the map shield (two 8x8 boards -> local 3D map -> shield) vs the same boards without the
# map, after the S6 matrix. Rows: g7_w2 (s6_w2), g7_w2map (s7_w2map), g7_w1map (s7_w1map); lowbox_s0-49 + cage20 x 8.
set -e
until grep -q "^DONE" ~/sim/runs/matrix_s6.log 2>/dev/null; do sleep 60; done
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh s7_matrix.txt ~/sim/runs/matrix_s7.log 3
