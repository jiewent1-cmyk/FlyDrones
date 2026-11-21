#!/bin/bash
# wait until batch log $1 has as many 'done' lines as 'start' lines and no batch.py runs; then exit
L=$1
while true; do
  s=$(grep -c " start " $L 2>/dev/null); d=$(grep -c " done " $L 2>/dev/null)
  if [ "$s" -gt 0 ] && [ "$s" -eq "$d" ] && ! pgrep -f "python3 batch.py" > /dev/null; then echo "batch $L finished: $d runs"; exit 0; fi
  sleep 10
done
