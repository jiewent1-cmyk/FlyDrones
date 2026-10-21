#!/bin/bash
# PIL hover-hold check (2026-10-20 night): the Orange Pi companion from /root/pil_hover (= /opt/ecps295 + feat/hover
# mavlink_twin.py, ECPS_ROOT) against one Gazebo instance on ROG (INST 5). Per world: pil4p = ECPS_HOLD=pos (yaw 30,
# brake 0.1), then pil4v = vel. Pairs with Gazebo-local hb2 / hp0 (same worlds, seeds). Respects the ROG quiet hours.
PI="root@192.168.86.20"; R=ROG-laptop; LOG=$1; shift
quiet() { local m=$((10#$(TZ=America/Los_Angeles date +%H) * 60 + 10#$(TZ=America/Los_Angeles date +%M))); [ $m -ge 55 ] && [ $m -lt 420 ]; }
for spec in "$@"; do   # spec = <world>:<seed>:<pos|vel>
  IFS=: read -r w k hold <<< "$spec"; tag=pil4${hold:0:1}__${w}_k${k}
  while quiet; do sleep 120; done
  for i in $(seq 1 36); do ssh -o ConnectTimeout=20 $PI "pgrep -f '[e]cps_companion' > /dev/null" || break; sleep 5; done
  for i in 1 2 3; do ssh -o ConnectTimeout=20 $PI "cd /root/pil_hover && (setsid nohup env ECPS_ROOT=/root/pil_hover ECPS_HOLD=$hold ECPS_HOLD_YAW_DPS=30 bash -c 'for i in 1 2 3 4; do /opt/ecps295/venv/bin/python companion/ecps_companion.py --mode guided --pil 14600 --url udpin:0.0.0.0:14561 --seed $k --set safety.max_flight_s=420 --seconds 900 --affinity 6,7 --tag $tag --log-dir /root/pil_hover/logs/pil --quiet; [ \$? -eq 1 ] || break; done' > /root/pil_hover/logs/pil/$tag.out 2>&1 < /dev/null &)" && ssh -o ConnectTimeout=20 $PI "sleep 3; pgrep -f '[-]-tag $tag' > /dev/null" && break; sleep 10; done
  ssh -o ConnectTimeout=20 $R "cd ~/sim/rl_h2 && echo '$tag $w s7_w2map@intact $k --mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn' > hv/$tag.txt && V=~/sim/rl_h2/ecps295/sitl/variants && (setsid nohup env INST=5 ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 ECPS_PIL=192.168.86.20:14600 SITL_EXTRA=--serial2=udpclient:192.168.86.20:14561 NAV=flow EXTRA_PARM=\$V/G_extnav_height.parm,\$V/companion_fs.parm,\$V/H_terrain_fast.parm bash run_matrix_mine.sh hv/$tag.txt ~/sim/runs/matrix_pil4.log > /dev/null 2>&1 < /dev/null &)"
  echo "$(date +%T) started $tag" >> $LOG
  for i in $(seq 1 120); do sleep 10; ssh -o ConnectTimeout=20 $R "pgrep -f '[r]un_matrix_mine.sh hv/$tag.txt' > /dev/null" || break; done
  for i in $(seq 1 30); do ssh -o ConnectTimeout=20 $PI "pgrep -f '[-]-tag $tag' > /dev/null" || break; sleep 5; done
  echo "$(date +%T) done $tag $(ssh $R "grep '^$tag ' ~/sim/runs/matrix_pil4.log | cut -c1-200")" >> $LOG
done
echo "ALLDONE $(date +%T)" >> $LOG
