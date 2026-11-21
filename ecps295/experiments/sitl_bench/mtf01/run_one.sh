#!/bin/bash
# usage: run_one.sh <tag> <instance 3..6> <speedup> <parm file> [mtf_sim.py args...]
# Each run gets its own directory runs/<tag>/ (eeprom.bin, logs/, sitl.log, run.log). Only touches its own instance.
D=/home/jacky/sim/mtf01
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; spd=$3; parm=$4; shift 4
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd; cd $rd
ln -s $D/terrain terrain
# clear a leftover of *this* instance only
for p in $(pgrep -f "arducopter .* -I$inst( |$)"); do kill $p 2>/dev/null; done
sleep 1
$AP/build/sitl/bin/arducopter --model + --speedup $spd -I$inst --defaults $AP/Tools/autotest/default_params/copter.parm,$parm \
    --home 33.6430,-117.8420,20,0 > sitl.log 2>&1 < /dev/null &
pid=$!
echo $pid > sitl.pid
echo "tag=$tag inst=$inst speedup=$spd parm=$parm args=$*" > cmd.txt
sleep 3
timeout $((600 / spd + 120)) $PY -u $D/mtf_sim.py "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/analyze.py $rd > metrics.json 2> analyze.err
