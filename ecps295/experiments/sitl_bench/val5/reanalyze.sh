#!/bin/bash
# re-run abs_analyze.py on every run dir (adds msgs_all); SITL logs are still in runs/*/logs
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
for d in /home/jacky/sim/val5/runs/*/; do
  [ -f "$d/cmd.txt" ] || continue
  $PY /home/jacky/sim/val5/abs_analyze.py "$d" > "$d/metrics.json" 2> "$d/analyze.err"
done
python3 /home/jacky/sim/val5/tables_v5.py > /dev/null
python3 /home/jacky/sim/val5/fails.py
