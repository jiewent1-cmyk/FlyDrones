#!/bin/bash
# copy of field1119/run_rehearsal.sh, unchanged except: instance -I6 (port 5820), run dir runs/<tag>/, analysis
D=/home/jacky/sim/mtf01; AP=/home/jacky/sim/ardupilot; PY=/home/jacky/miniconda3/envs/flydrones/bin/python
tag=$1; shift
rd=$D/runs/$tag; rm -rf $rd; mkdir -p $rd; cd $rd; ln -s $D/terrain terrain
for p in $(pgrep -f "arducopter .* -I6( |$)"); do kill $p 2>/dev/null; done
sleep 1
$AP/build/sitl/bin/arducopter --model + --speedup 1 -I6 --defaults $AP/Tools/autotest/default_params/copter.parm,$D/orig/field_sitl.parm --home 33.6430,-117.8420,20,0 > sitl.log 2>&1 < /dev/null &
pid=$!; echo $pid > sitl.pid
echo "tag=$tag inst=6 speedup=1 parm=orig/field_sitl.parm args=$*" > cmd.txt
sleep 3
timeout 330 $PY -u $D/orig/auto_mission.py rehearse "$@" --url tcp:127.0.0.1:5820 > run.log 2>&1
echo "rehearsal exit $?" >> run.log
kill $pid 2>/dev/null; sleep 1; kill -9 $pid 2>/dev/null
$PY $D/analyze.py $rd > metrics.json 2> analyze.err
