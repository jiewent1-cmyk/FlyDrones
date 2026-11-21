#!/bin/bash
# part 1 main battery (task V3), instances 40 and 41
D=/home/jacky/sim/val3
cd $D
OV=FLOW_TYPE=5,RNGFND1_TYPE=10
DS="0.5:4,1.0:4,2.0:4,7.9:4,9.0:4,2.0:8,2.0:8"
QS="100:20,0:8,100:14"
(bash run_feed.sh p1_main200 40 $OV --sysid 200 --dist-schedule "$DS" --duration 42 \
   --flow --flow-schedule "0.5:-0.25:42" --quality-schedule "$QS" \
   --stop-flow-at 30 --stop-dist-at 34 > runs_A.log 2>&1 &)
sleep 1
(bash run_feed.sh p1_main1 41 $OV --sysid 1 --dist-schedule "$DS" --duration 42 \
   --flow --flow-schedule "0.5:-0.25:42" --quality-schedule "$QS" \
   --stop-flow-at 30 --stop-dist-at 34 > runs_B.log 2>&1 &)
sleep 70
echo launched
