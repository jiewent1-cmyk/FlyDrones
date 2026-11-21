#!/bin/bash
# waits for the val8 loop batch, runs a 4-job smoke test, and only then starts the full ablation batch
cd ~/sim/abl
PY=~/miniconda3/envs/flydrones/bin/python
while pgrep -f "val8/batch.py" > /dev/null; do sleep 15; done
echo "$(date +%T) val8 batch finished" >> chain.log
NINST=4 $PY batch.py jobs_smoke.txt > batch_smoke.log 2>&1
ok=0
for t in $(cut -d"|" -f1 jobs_smoke.txt); do
  c=$($PY -c "import json,sys;print(json.load(open(sys.argv[1])).get(\"cls\",\"?\"))" runs/$t/safety.json 2>/dev/null)
  echo "$(date +%T) smoke $t -> $c" >> chain.log
  case "$c" in OK|ABORT|DEGRADED|UNSAFE) ok=$((ok+1));; esac
done
if [ $ok -lt 4 ]; then echo "$(date +%T) SMOKE FAILED ($ok/4) - full batch NOT started" >> chain.log; exit 1; fi
echo "$(date +%T) smoke passed, starting full batch (3186 jobs, 8 instances)" >> chain.log
NINST=8 $PY batch.py jobs_abl.txt > batch_abl.log 2>&1
echo "$(date +%T) full batch finished" >> chain.log
