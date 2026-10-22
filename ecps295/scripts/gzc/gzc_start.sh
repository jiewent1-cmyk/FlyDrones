#!/bin/bash
# Gazebo ground-console session, Pi side (2026-10-21, hover session): the companion from /root/pil_hover (PIL camera +
# ToF from ROG, console link on 14700) and a temporary mavlink-router: SITL on ROG (--serial2=udpclient:<pi>:14561) ->
# companion UDP 14563 + ground station UDP 14550 (server), so the Mac app connects with its default ports.
# mavlink-router.service holds 14550: stopped here, started again by gzc_stop.sh (which also restores the governor).
#   gzc_start.sh            stop with gzc_stop.sh (never pkill -f over ssh)
cd /root/pil_hover
[ -f gzc.pids ] && { echo "already running (gzc.pids)"; exit 1; }
cat /sys/devices/system/cpu/cpufreq/policy6/scaling_governor > gzc.gov
echo performance > /sys/devices/system/cpu/cpufreq/policy6/scaling_governor
systemctl is-active --quiet mavlink-router && { systemctl stop mavlink-router; echo 1 > gzc.router; }
cat > gzc_router.conf <<R
[General]
TcpServerPort = 0
ReportStats = false
[UdpEndpoint sitl]
Mode = Server
Address = 0.0.0.0
Port = 14561
[UdpEndpoint companion]
Mode = Normal
Address = 127.0.0.1
Port = 14563
[UdpEndpoint gcs]
Mode = Server
Address = 0.0.0.0
Port = 14550
R
setsid nohup taskset -c 4 mavlink-routerd -c gzc_router.conf > logs/gzc/router.log 2>&1 < /dev/null &
echo $! > gzc.pids
T=$(date +%Y%m%d_%H%M%S)
setsid nohup env ECPS_ROOT=/root/pil_hover ECPS_HOLD=pos ECPS_HOLD_YAW_DPS=30 bash -c "while true; do nice -n -10 /opt/ecps295/venv/bin/python -u companion/ecps_companion.py --mode guided --pil 14600 --url udpin:127.0.0.1:14563 --set safety.max_flight_s=1800 --set safety.geofence_radius_m=${GZC_FENCE:-2.7} --seconds 0 --affinity 6,7 --tag gzc_$T --log-dir /root/pil_hover/logs/gzc; rc=\$?; echo \"companion exit \$rc \$(date +%T)\"; [ \$rc -eq 1 ] || [ -f gzc.restart ] || break; sleep 2; done" > logs/gzc/companion_$T.out 2>&1 < /dev/null &
echo $! >> gzc.pids
echo "started router + companion ($(tr "\n" " " < gzc.pids)), log logs/gzc/companion_$T.out"
