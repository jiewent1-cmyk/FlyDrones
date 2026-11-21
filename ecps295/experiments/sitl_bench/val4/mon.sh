#!/bin/bash
while true; do
  n=$(grep -c " done " /home/jacky/sim/val2/batch_v2.log 2>/dev/null)
  s=$(grep -c start /home/jacky/sim/val2/batch_v2.log 2>/dev/null)
  echo "progress: done=$n started=$s/204"
  grep -iE "Traceback" /home/jacky/sim/val2/batch_v2.log | head -3
  if ! pgrep -f "batch.py jobs_v2" > /dev/null; then echo "BATCH FINISHED done=$n"; break; fi
  sleep 60
done
