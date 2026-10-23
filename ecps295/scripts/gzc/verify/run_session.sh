#!/bin/bash
# One planner-verification session on ROG (2026-10-22): Gazebo + SITL (gz_g4_mine.sh INST n, run_g4 --mode console =
# scripted pilot), the companion from ~/sim/gzc (same code as the Pi test copy, run locally: PIL over 127.0.0.1) with
# the console link, and verify_client.py playing the plan. FC heartbeat timeout 4 s: SITL beats per simulated second. Truth: ~/sim/runs/<tag>/run.csv; client log: client.json.
#   run_session.sh <world> <plan|noplan> <inst 5-7> <plan.json> <tag>
W=$1; COND=$2; I=$3; PLAN=$4; TAG=$5
G=~/sim/gzc; R=~/sim/rl_h2; V=$R/ecps295/sitl/variants; M=$R/ecps295/minifly
PILP=$((14600 + 10 * I)); SITLP=$((14561 + 10 * I)); ECLP=$((14700 + 10 * I)); TCP=$((5760 + 10 * I))
FENCE=$(python3 -c "import json; print(json.load(open('$PLAN'))['fence_m'])")
mkdir -p $G/logs/$TAG
# 10-22: a companion left over from an earlier session on this instance kept the ports and flew the next session
# (the new one failed with EADDRINUSE): end any companion of this instance first, then wait for the ports
for p in $(pgrep -f "ecps_companion.py .*--console $ECLP "); do kill $p; done
for i in $(seq 1 20); do ss -lun | grep -qE ":($PILP|$SITLP|$ECLP) " || break; sleep 1; done
ss -lun | grep -qE ":($PILP|$SITLP|$ECLP) " && { echo "$TAG ports $PILP/$SITLP/$ECLP still busy, skipped"; exit 1; }
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
# companion (restarted while it exits 1 = no FC heartbeat yet; a session end exits 0)
( cd $G; while true; do ECPS_FC_HB_TIMEOUT_S=4 ECPS_ROOT=$G ECPS_HOLD=pos ECPS_HOLD_YAW_DPS=30 python -u companion/ecps_companion.py --mode guided \
    --pil $PILP --url udpin:127.0.0.1:$SITLP --console $ECLP --hw config/hw_$COND.yaml --set safety.max_flight_s=3600 \
    --set safety.geofence_radius_m=$FENCE --seconds 0 --tag $TAG --log-dir $G/logs/$TAG; rc=$?; [ $rc -eq 1 ] || break; sleep 2; done ) \
  > $G/logs/$TAG/companion.out 2>&1 &
CP=$!
# Gazebo + SITL + scripted pilot
( cd $R; INST=$I RUN_TIMEOUT=3600 ECPS_ARM=intact ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 ECPS_PIL=127.0.0.1:$PILP \
    SITL_EXTRA=--serial2=udpclient:127.0.0.1:$SITLP NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm \
    bash gz_g4_mine.sh ecps295_$W.sdf $TAG "--mode console --seconds 3000 --cruise 0.5 --max-forward 0.5 --fence-turn --seed $I --config $M/s7_w2map.yaml --decoder ecps --pilot ecps --retina ecps --out run" ) \
  > $G/logs/$TAG/gz.out 2>&1 &
GP=$!
sleep 15
python -u $G/verify/verify_client.py --port $ECLP --plan $PLAN --tag $TAG --out $G/logs/$TAG/client.json --timeout 2700 > $G/logs/$TAG/client.out 2>&1
# end of the session: run_g4 lands and tears the instance down, the companion exits with go -> 0
for p in $(pgrep -f "^python [^ ]*run_g4.py .*--url tcp:127.0.0.1:$TCP "); do kill -INT $p; done  # python only: SIGINT to timeout too would interrupt the landing
for i in $(seq 1 60); do kill -0 $GP 2>/dev/null || break; sleep 2; done
for i in $(seq 1 15); do kill -0 $CP 2>/dev/null || break; sleep 2; done
pkill -P $CP 2>/dev/null; kill $CP 2>/dev/null  # the python first: once the subshell is gone it is no longer its child
for p in $(pgrep -f "ecps_companion.py .*--tag $TAG "); do kill $p; done
sleep 2; pgrep -f "ecps_companion.py .*--tag $TAG " > /dev/null && echo "$TAG companion still running after the session"
cp $G/logs/$TAG/client.json ~/sim/runs/$TAG/ 2>/dev/null
echo "$TAG $(tail -1 $G/logs/$TAG/client.out)"
