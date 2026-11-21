#!/bin/bash
# usage: run_one.sh <tag> <instance 10..13> <speedup> <parm file> <home yaw> [opt_sim.py args...]
# Own directory runs/<tag>/. Only touches instance $inst (5760 + 10*inst).
D=/home/jacky/sim/opt_flow
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; spd=$3; parm=$4; yaw=$5; shift 5
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd/logs; cd $rd
ln -sfn $D/terrain terrain
# clear a leftover of *this* instance only
for p in $(pgrep -f "arducopter .* -I$inst( |\$)"); do kill $p 2>/dev/null; done
sleep 1
$AP/build/sitl/bin/arducopter --model + --speedup $spd -I$inst --defaults $AP/Tools/autotest/default_params/copter.parm,$parm \
    --home 33.6430,-117.8420,20,$yaw > sitl.log 2>&1 < /dev/null &
pid=$!
echo $pid > sitl.pid
echo "tag=$tag inst=$inst speedup=$spd parm=$parm yaw=$yaw args=$*" > cmd.txt
sleep 3
timeout $((1200 / spd + 200)) $PY -u $D/opt_sim.py "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/analyze_opt.py $rd > metrics.json 2> analyze.err
