#!/bin/bash
# usage: run_fly.sh <tag> <inst 40|41> <overlay K=V,...|-> [val3_fly.py args...]
# Part 2 (task V3): one SITL on my instance number, one MAVLink link that carries the
# mission and (optionally) the simulated MTF-01 sensor feed.
D=/home/jacky/sim/val3
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; ovl=$3; shift 3
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd
: > $rd/overlay.parm
if [ "$ovl" != "-" ]; then echo "$ovl" | tr ',' '\n' | tr '=' ' ' > $rd/overlay.parm; fi
for p in $(pgrep -f "arducopter .* -I$inst( |\$)"); do kill $p 2>/dev/null; done
sleep 1
cd $rd
$AP/build/sitl/bin/arducopter --model + --speedup 1 -I$inst \
    --defaults $AP/Tools/autotest/default_params/copter.parm,$D/mtf01_sitl_rng.parm,$rd/overlay.parm \
    --home 33.6430,-117.8420,20,30 > sitl.log 2>&1 < /dev/null &
pid=$!
echo $pid > sitl.pid
echo "tag=$tag inst=$inst port=$port overlay=$ovl args=$*" > cmd.txt
sleep 12
timeout 300 $PY -u $D/val3_fly.py --url tcp:127.0.0.1:$port --out $rd/run.json --truth-log $rd/truth.csv "$@" > run.log 2>&1
echo "script exit $?" >> run.log
sleep 6
kill $pid 2>/dev/null; sleep 2; kill -9 $pid 2>/dev/null
$PY $D/analyze_mtf.py $rd > metrics.json 2> metrics.err
echo "analyze exit $?" >> run.log
$PY $D/analyze_of.py $rd > of.json 2> of.err
