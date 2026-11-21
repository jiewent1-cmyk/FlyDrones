#!/bin/bash
# re-run analyze2.py on every finished non-base run (after analyzer fixes)
cd /home/jacky/sim/opt_flow2
for d in runs/*; do case $d in runs/base_*) continue;; esac
  grep -q "script exit" $d/run.log 2>/dev/null && [ ! -f $d/sitl.pid -o -n "$(grep -l . $d/metrics.json 2>/dev/null)" ] && /home/jacky/miniconda3/envs/flydrones/bin/python analyze2.py $d > $d/metrics.json 2> $d/analyze.err &
  while [ $(jobs -r | wc -l) -ge 8 ]; do sleep 0.2; done
done; wait
