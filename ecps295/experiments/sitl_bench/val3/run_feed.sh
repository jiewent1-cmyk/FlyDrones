#!/bin/bash
# usage: run_feed.sh <tag> <inst 40|41> <overlay K=V,...|-> [mtf_feed.py args...]
# Own copy of /home/jacky/sim/opt_flow2/run_one.sh for task V3: starts one SITL on my
# instance number, runs mtf_feed.py against it, then kills the SITL and analyses the
# dataflash log.  Only touches instance $inst.
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
timeout 180 $PY -u $D/mtf_feed.py --url tcp:127.0.0.1:$port --out $rd/feed.json "$@" > feed.log 2>&1
echo "feeder exit $?" >> feed.log
sleep 3
kill $pid 2>/dev/null; sleep 2; kill -9 $pid 2>/dev/null
$PY $D/analyze_of.py $rd > of.json 2> of.err
echo "analyze exit $?" >> feed.log
