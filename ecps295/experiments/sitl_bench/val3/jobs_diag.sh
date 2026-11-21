#!/bin/bash
# differential test: opt2_sim (known good) vs my feeder, one SITL each
D=/home/jacky/sim/val3
cd $D
PY=/home/jacky/miniconda3/envs/flydrones/bin/python
# instance 40: opt2_sim against a SITL started the same way as my feeder
AP=/home/jacky/sim/ardupilot
rd=$D/runs/opt2probe
rm -rf $rd; mkdir -p $rd
: > $rd/overlay.parm
echo "FLOW_TYPE 5
RNGFND1_TYPE 10" > $rd/overlay.parm
cd $rd
$AP/build/sitl/bin/arducopter --model + --speedup 1 -I40 \
  --defaults $AP/Tools/autotest/default_params/copter.parm,$D/mtf01_sitl_rng.parm,$rd/overlay.parm \
  --home 33.6430,-117.8420,20,30 > sitl.log 2>&1 < /dev/null &
p1=$!
cd $D
sleep 12
timeout 40 $PY -u /home/jacky/sim/opt_flow2/opt2_sim.py D3 --url tcp:127.0.0.1:6160 --no-loiter-test --no-auto --settle 12 --speed 1 > runs/opt2_probe.log 2>&1
echo "opt2 exit $?" >> runs/opt2_probe.log
kill $p1 2>/dev/null; sleep 1; kill -9 $p1 2>/dev/null
echo OPT2DONE
