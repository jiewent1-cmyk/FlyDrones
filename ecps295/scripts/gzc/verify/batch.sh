#!/bin/bash
# Planner verification batch (2026-10-22): lines "<world> <plan|noplan> <plan.json> <tag>" in a queue file, 3 workers on
# instances 5-7, each takes the next line (flock). No session starts in the ROG quiet hours (00:55-07:00 PT).
#   batch.sh <queue> <log>
Q=$1; LOG=$2; G=~/sim/gzc/verify
quiet() { local m=$((10#$(TZ=America/Los_Angeles date +%H) * 60 + 10#$(TZ=America/Los_Angeles date +%M))); [ $m -ge 55 ] && [ $m -lt 420 ]; }
take() { flock $Q.lock bash -c "l=\$(head -1 $Q); [ -n \"\$l\" ] && sed -i 1d $Q; echo \"\$l\""; }
echo "start $(date)" >> $LOG
for I in 5 6 7; do
  ( sleep $(( (I - 5) * 25 ))
    while true; do
      while quiet; do sleep 120; done
      L=$(take); [ -z "$L" ] && break
      read -r W C P T <<< "$L"
      echo "$(date +%T) i$I start $T" >> $LOG
      bash $G/run_session.sh $W $C $I $P $T >> $LOG 2>&1
    done ) &
done
wait
echo "ALLDONE $(date)" >> $LOG
