#!/bin/bash
# usage: run_one.sh <tag> <instance 20..23> <speedup> <parm file> <home lat,lon,alt,hdg> [gf_sim.py args...]
# Copied from ~/sim/mtf01/run_one.sh; own dir, own instances, home (start point/heading) per run.
D=/home/jacky/sim/gps_flow
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; spd=$3; parm=$4; home=$5; shift 5
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd; cd $rd
ln -s /home/jacky/sim/mtf01/terrain terrain
# clear a leftover of *this* instance only (pattern needs the binary path, so it never matches this script)
for p in $(pgrep -f "^$AP/build/sitl/bin/arducopter .* -I$inst( |$)"); do kill $p 2>/dev/null; done
sleep 1
$AP/build/sitl/bin/arducopter --model + --speedup $spd -I$inst --defaults $AP/Tools/autotest/default_params/copter.parm,$parm \
    --home $home > sitl.log 2>&1 < /dev/null &
pid=$!
echo $pid > sitl.pid
echo "tag=$tag inst=$inst speedup=$spd parm=$parm home=$home args=$*" > cmd.txt
cp $parm run.parm
sleep 3
timeout $((600 / spd + 120)) $PY -u $D/gf_sim.py "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/gf_analyze.py $rd > metrics.json 2> analyze.err
