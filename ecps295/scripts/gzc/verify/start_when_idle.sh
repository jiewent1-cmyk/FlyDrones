#!/bin/bash
# start a batch once ROG is idle: 1-min load < 4 for 3 checks in a row (another session's training drops RTF and arming fails)
Q=$1; LOG=$2; n=0
echo "waiting for idle $(date)" >> $LOG
while [ $n -lt 3 ]; do l=$(cut -d" " -f1 /proc/loadavg); if awk "BEGIN{exit !($l < 4)}"; then n=$((n+1)); else n=0; fi; sleep 60; done
echo "idle, load $(cut -d" " -f1-3 /proc/loadavg) $(date)" >> $LOG
exec bash ~/sim/gzc2/verify/batch.sh $Q $LOG
