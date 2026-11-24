#!/bin/bash
# usage: run_one.sh <tag> <instance> <speedup> <parm file> <home lat,lon,alt,hdg> [gh_sim.py args...]
D=/home/jacky/sim/gps_hover
AP=/home/jacky/sim/ardupilot
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; inst=$2; spd=$3; parm=$4; home=$5; shift 5
port=$((5760 + 10 * inst))
rd=$D/runs/$tag
rm -rf $rd; mkdir -p $rd; cd $rd
ln -s /home/jacky/sim/mtf01/terrain terrain
$AP/build/sitl/bin/arducopter --model + --speedup $spd -I$inst --defaults $AP/Tools/autotest/default_params/copter.parm,$parm \
    --home $home > sitl.log 2>&1 < /dev/null &
pid=$!
echo "tag=$tag inst=$inst speedup=$spd parm=$parm home=$home args=$*" > cmd.txt
cp $parm run.parm
sleep 3
timeout 1600 $PY -u $D/gh_sim.py "$@" --speed $spd --url tcp:127.0.0.1:$port > run.log 2>&1
echo "script exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/gh_analyze.py $rd > metrics.json 2> analyze.err
gzip -1 logs/*.BIN 2>/dev/null
rm -f eeprom.bin
