#!/bin/bash
# S6 2026-10-14: Gazebo check of the best cheaper sensor layouts from the twin screen (rl.shield_screen s6_*), after the
# mb5 matrix (s5, new brain seeds). ~/sim/rl_h2/s6_matrix.txt is written once the twin screen has finished.
set -e
until grep -q "^DONE" ~/sim/runs/matrix_mb5.log 2>/dev/null && [ -s ~/sim/rl_h2/s6_matrix.txt ]; do sleep 60; done
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh s6_matrix.txt ~/sim/runs/matrix_s6.log 3
