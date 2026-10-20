#!/bin/bash
# 2026-10-18 hover precision: run_g4 --mode hovertest, g7_sc2_tf10 FC params. usage: run.sh <matrix> <log> <vel|pos> [inst]
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 ECPS_HOLD=$3 INST=${4:-5}
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_mine.sh $1 $2
