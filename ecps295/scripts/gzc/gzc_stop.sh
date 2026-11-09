#!/bin/bash
# Stop the Gazebo console session (gzc_start.sh): companion loop + router, then mavlink-router.service and the governor.
cd /root/pil_hover
if [ -f gzc.pids ]; then
  for p in $(cat gzc.pids); do kill -- -$p 2>/dev/null || kill $p 2>/dev/null; done
  sleep 2; rm -f gzc.pids
fi
[ -f gzc.router ] && { systemctl start mavlink-router; rm -f gzc.router; }
[ -f gzc.gov ] && { cat gzc.gov > /sys/devices/system/cpu/cpufreq/policy6/scaling_governor; rm -f gzc.gov; }
echo "stopped; mavlink-router $(systemctl is-active mavlink-router), governor $(cat /sys/devices/system/cpu/cpufreq/policy6/scaling_governor)"
