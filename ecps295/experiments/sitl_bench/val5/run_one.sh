#!/bin/bash
# usage: run_one.sh <tag> <instance 10..13> <speedup> <base parm> <home yaw deg> <overlay K=V,K=V|-> [abs_sim.py args...]
# Copy of /home/jacky/sim/opt_flow2/run_one.sh for abs_fix: own dir runs/<tag>/, only touches instance $inst.
D=/home/jacky/sim/val5
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; spd=$3; parm=$4; yaw=$5; ovl=$6; shift 6
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd/terrain; cd $rd
: > overlay.parm
if [ "$ovl" != "-" ]; then echo "$ovl" | tr ',' '\n' | tr '=' ' ' > overlay.parm; fi
for p in $(pgrep -f "arducopter .* -I$inst( |\$)"); do kill $p 2>/dev/null; done
sleep 1
$AP/build/sitl/bin/arducopter --model + --speedup $spd -I$inst --defaults $AP/Tools/autotest/default_params/copter.parm,$parm,$rd/overlay.parm \
    --home 33.6430,-117.8420,20,$yaw > sitl.log 2>&1 < /dev/null &
pid=$!
echo $pid > sitl.pid
echo "tag=$tag inst=$inst speedup=$spd parm=$parm yaw=$yaw ovl=$ovl args=$*" > cmd.txt
sleep 3
timeout $((1800 / spd + 200)) $PY -u $D/abs_sim.py "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/abs_analyze.py $rd > metrics.json 2> analyze.err
