#!/bin/bash
D=/home/jacky/sim/val6
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
FIN="EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1"
ORIG="EK3_RNG_M_NSE=0.5,EK3_HGT_I_GATE=500,SURFTRAK_MODE=1,FENCE_ALT_MAX=2.5,FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1"
SQ="SQ --alt 2.0 --side 2.0 --laps 1 --hold 2.0 --no-loiter-test --thr-auto 1500 --settle 20 --seconds 120"
bash $D/run_one.sh v6_probe_sq_f30_r1 30 10 $D/mtf01_sitl_rng.parm 30 "$FIN,RNGFND1_MIN=0.30" $SQ > /dev/null 2>&1 &
bash $D/run_one.sh v6_probe_sq_o30_r1 31 10 $D/mtf01_sitl_rng.parm 150 "$ORIG,RNGFND1_MIN=0.30" $SQ > /dev/null 2>&1 &
bash $D/run_one.sh v6_probe_h_f01_r1 32 10 $D/mtf01_sitl_rng.parm 270 "$FIN,RNGFND1_MIN=0.01" $SQ --fault 2.5:SIM_SONAR_POS_Z=0.53 --fault 3.0:SIM_SONAR_POS_Z=0.81 --fault 3.5:SIM_SONAR_POS_Z=1.04 --fault 4.0:SIM_SONAR_POS_Z=1.29 --fault 4.5:SIM_SONAR_POS_Z=1.53 --fault 5.0:SIM_SONAR_POS_Z=1.77 --fault 5.5:SIM_SONAR_POS_Z=1.93 > /dev/null 2>&1 &
bash $D/run_one.sh v6_probe_g_f20_r1 33 10 $D/mtf01_sitl_rng.parm 90 "$FIN,RNGFND1_MIN=0.20" $SQ --fault 6.0:SIM_SONAR_POS_Z=1.9 --fault 9.0:SIM_SONAR_POS_Z=-0.05 > /dev/null 2>&1 &
wait
