#!/bin/bash
# N1b 2026-10-13: sh5u with the full-cone ToF emulation (ECPS_TOF_CONE3D=1), lowbox_s0-49 + cage20 k0-7
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh sh3d_matrix.txt ~/sim/runs/matrix_msh3.log 3
