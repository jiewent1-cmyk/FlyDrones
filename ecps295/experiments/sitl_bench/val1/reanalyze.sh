#!/bin/bash
# reanalyze.sh [tag-prefix ...]: re-run analyze2.py on the matching run dirs
cd /home/jacky/sim/val1
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
for d in runs/*/; do
    t=$(basename $d)
    if [ $# -gt 0 ]; then
        ok=0
        for p in "$@"; do case $t in $p*) ok=1;; esac; done
        [ $ok -eq 0 ] && continue
    fi
    [ -d "$d/logs" ] || continue
    $PY analyze2.py $d > $d/metrics.json 2> $d/analyze.err
done
echo reanalyze done
