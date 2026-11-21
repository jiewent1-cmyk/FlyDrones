#!/bin/bash
# part 1 rate sweep: constant send rate 10/20/50/100 Hz, FC armed on the ground so the
# dataflash OF message is logged.  Two instances (40,41) -> two runs in parallel, two rounds.
D=/home/jacky/sim/val3
cd $D
OV=FLOW_TYPE=5,RNGFND1_TYPE=10
COMMON="--sysid 200 --dist 2.0 --duration 25 --flow --flow-x 0.5 --flow-y -0.25 --quality 100 --arm-ground"

(bash run_feed.sh p1_hz10 40 $OV $COMMON --hz 10 > runs_E.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_hz20 41 $OV $COMMON --hz 20 > runs_F.log 2>&1 &)
sleep 60

(bash run_feed.sh p1_hz50 40 $OV $COMMON --hz 50 > runs_G.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_hz100 41 $OV $COMMON --hz 100 > runs_H.log 2>&1 &)
sleep 60
echo launched
