#!/bin/bash
# Gazebo ground-console session (2026-10-21): Gazebo + SITL on ROG (INST 5, *_monitor world with the chase camera,
# run_g4 --mode console = scripted pilot), the companion + console link on the Pi (/root/pil_hover/gzc_start.sh).
# The Mac app connects to the Pi with its default ports (ECL 14700, MAVLink 14550).
#   gzc.sh start [world] [minutes]     default cage20, 60 min
#   gzc.sh stop                         lands (run_g4 ends), stops both sides
#   gzc.sh status
PI=root@192.168.86.28; R=ROG-laptop
cmd=${1:-status}; W=${2:-cage20}; MIN=${3:-60}
case $cmd in
start)
  ssh $R "pgrep -f '[r]un_g4.py --mode console' >/dev/null" && { echo "a console session already runs on ROG"; exit 1; }
  ssh $PI "pgrep -f '[e]cps_companion' >/dev/null" && { echo "a companion already runs on the Pi (another test?)"; exit 1; }
  ssh $PI /root/pil_hover/gzc_start.sh || exit 1
  TAG=gzc_$(date +%m%d_%H%M)_$W; S=$((MIN * 60))
  ssh $R "cd ~/sim/rl_h2; M=~/sim/rl_h2/ecps295/minifly; V=~/sim/rl_h2/ecps295/sitl/variants; (setsid nohup env INST=5 RUN_TIMEOUT=$((S + 600)) ECPS_ARM=intact ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 ECPS_PIL=192.168.86.20:14600 ECPS_PIL_CHASE=/dev/shm/ecps295_i5_chase SITL_EXTRA=--serial2=udpclient:192.168.86.20:14561 NAV=flow EXTRA_PARM=\$V/G_extnav_height.parm,\$V/companion_fs.parm,\$V/H_terrain_fast.parm bash gz_g4_mine.sh ecps295_${W}_monitor.sdf $TAG \"--mode console --seconds $S --cruise 0.5 --max-forward 0.5 --fence-turn --seed 0 --config \$M/s7_w2map.yaml --decoder ecps --pilot ecps --retina ecps --out run\" > ~/sim/runs/$TAG.out 2>&1 < /dev/null &)"
  echo "started $TAG ($MIN min). ROG log: ~/sim/runs/$TAG/exp.log" ;;
stop)
  ssh $R 'for p in $(pgrep -f "^python [^ ]*run_g4.py --mode console"); do kill -INT $p; done; echo "run_g4 interrupted (lands, then the instance tears down)"'
  for i in $(seq 1 24); do ssh $R "pgrep -f '[r]un_g4.py --mode console' >/dev/null" || break; sleep 5; done
  ssh $PI /root/pil_hover/gzc_stop.sh ;;
status)
  ssh $R 'p=$(pgrep -f "[r]un_g4.py --mode console"); echo "ROG run_g4: ${p:-none}"; d=$(ls -td ~/sim/runs/gzc_* 2>/dev/null | grep -v "\.out$" | head -1); [ -n "$d" ] && { echo "$d"; grep -E "console:|airborne|Error|error" $d/exp.log | tail -6; }'
  ssh $PI 'cd /root/pil_hover; [ -f gzc.pids ] && echo "Pi session: $(cat gzc.pids | tr "\n" " ")" || echo "Pi session: none"; f=$(ls -t logs/gzc/companion_*.out 2>/dev/null | head -1); [ -n "$f" ] && tail -4 $f' ;;
esac
