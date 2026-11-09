#!/bin/bash
# Planner verification batch (2026-10-22): lines "<world> <plan|noplan> <plan.json> <tag>" in a queue file, WORKERS (default 5 6) on
# instances 5-7, each takes the next line (flock).
#   batch.sh <queue> <log>
Q=$1; LOG=$2; G=~/sim/gzc2/verify
take() { flock $Q.lock bash -c "l=\$(head -1 $Q); [ -n \"\$l\" ] && sed -i 1d $Q; echo \"\$l\""; }
echo "start $(date)" >> $LOG
for I in ${WORKERS:-5 6}; do  # 10-22: 3 instances ran at RTF ~0.6 (heartbeat gaps, skewed timing); 2 by default
  ( sleep $(( (I - 5) * 25 ))
    while true; do
      L=$(take); [ -z "$L" ] && break
      read -r W C P T <<< "$L"
      echo "$(date +%T) i$I start $T" >> $LOG
      bash $G/run_session.sh $W $C $I $P $T >> $LOG 2>&1
    done ) &
done
wait
echo "ALLDONE $(date)" >> $LOG
