#!/bin/bash
# 2026-10-19 hover sweep: run.sh plus an optional extra parm file and yaw rate. usage: run2.sh <matrix> <log> <vel|pos> <inst> <parm|-> <yaw dps>
cd ~/sim/rl_h2
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 ECPS_HOLD=$3 INST=$4 ECPS_YAW_DPS=$6
X=; [ "$5" != - ] && X=,$5
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm$X bash run_matrix_mine.sh $1 $2
