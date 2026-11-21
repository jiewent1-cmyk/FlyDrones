#!/bin/bash
# usage: run_rehearsal.sh <tag> [auto_mission.py args...]
cd ~/sim/field1119
tag=$1; shift
[ -f sitl.pid ] && kill $(cat sitl.pid) 2>/dev/null
for p in $(pgrep -f "arducopter --model \+ --speedup 1 -I2"); do kill $p 2>/dev/null; done
sleep 1
rm -f eeprom.bin
AP=~/sim/ardupilot
$AP/build/sitl/bin/arducopter --model + --speedup 1 -I2 --defaults $AP/Tools/autotest/default_params/copter.parm,field_sitl.parm --home 33.6430,-117.8420,20,0 > sitl_$tag.log 2>&1 < /dev/null &
echo $! > sitl.pid
sleep 3
timeout 330 ~/miniconda3/envs/flydrones/bin/python -u auto_mission.py rehearse "$@" --url tcp:127.0.0.1:5780 > run_$tag.log 2>&1
echo "rehearsal exit $?" >> run_$tag.log
kill $(cat sitl.pid) 2>/dev/null
