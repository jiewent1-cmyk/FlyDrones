#!/bin/bash
# part 1 supplementary: FC armed on the ground -> dataflash OF log + input rate sweep
D=/home/jacky/sim/val3
cd $D
OV=FLOW_TYPE=5,RNGFND1_TYPE=10
HS="10:8,20:8,50:8,100:8"
(bash run_feed.sh p1_arm_of 40 $OV --sysid 200 --dist 2.0 --duration 34 \
   --flow --flow-schedule "0.5:-0.25:34" --quality 100 --hz-schedule "$HS" \
   --arm-ground > runs_C.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_arm_of1 41 $OV --sysid 200 --dist 2.0 --duration 34 \
   --flow --flow-schedule "0.5:-0.25:34" --quality 0 --hz-schedule "$HS" \
   --arm-ground > runs_D.log 2>&1 &)
sleep 60
echo launched
