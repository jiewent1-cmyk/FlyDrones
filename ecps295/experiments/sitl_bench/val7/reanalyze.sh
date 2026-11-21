#!/bin/bash
# reanalyze.sh -- 用当前 analyze7.py 重新生成所有运行目录的 metrics.json
D=/home/jacky/sim/val7
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
n=0
for rd in $D/runs/*/; do
  [ -f "$rd/logs/00000001.BIN" ] || continue
  [ -f "$rd/monitor.csv" ] || continue
  $PY $D/analyze7.py "$rd" > "$rd/metrics.json" 2> "$rd/analyze.err" && n=$((n+1)) &
  if [ $((n % 4)) -eq 0 ]; then wait; fi
done
wait
echo "reanalyzed $n runs"
