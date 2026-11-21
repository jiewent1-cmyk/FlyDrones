#!/bin/bash
# usage: run_one.sh <tag> <instance 30..33> <speedup> <base parm> <home yaw deg> <overlay K=V,K=V|-> [opt2_sim.py args...]
# Copy of /home/jacky/sim/opt_flow2/run_one.sh for /home/jacky/sim/abl, instances 30..33 (ports 6060..6090).
D=/home/jacky/sim/abl
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
hgpid=
if [ "$(cat $D/hg/$tag.json 2>/dev/null)" != "\"off\"" ]; then
  $PY -u $D/hg_loop.py --url tcp:127.0.0.1:$((port + 2)) --cfg "$(cat $D/hg/$tag.json)" > hg.log 2>&1 &
  hgpid=$!
fi
SIM=$D/opt2_sim7.py; AN=$D/analyze7.py
timeout $((1500 / spd + 150)) $PY -u $SIM "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
[ -n "$hgpid" ] && kill $hgpid 2>/dev/null; kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $AN $rd > metrics.json 2> analyze.err
$PY $D/safety.py $rd > safety.json 2> safety.err
rm -f $rd/eeprom.bin
