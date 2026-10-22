#!/bin/bash
# From the Mac: copy runtime tree (ROG Gazebo tree = what runs on the drone), twin tree (Mac mini s1b) and the
# companion package to the Pi, then run install_pi.sh there. Usage: ECPS_GCS_IP=<static-Mac-IP> deploy_to_pi.sh <rog_tree_dir> <twin_tree_dir> [host]
set -euo pipefail
ROGT=$1; TWIN=$2; H=${3:-root@192.168.86.20}; HERE=$(cd "$(dirname "$0")/.." && pwd)
if [ -z "${ECPS_GCS_IP:-}" ]; then echo "Set ECPS_GCS_IP to the control Mac's static LAN address." >&2; exit 2; fi
case "$ECPS_GCS_IP" in (*[!0-9.]*|.*|*..*|*.) echo "ECPS_GCS_IP must be a fixed IPv4 address." >&2; exit 2 ;; esac
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$ECPS_GCS_IP" || { echo "ECPS_GCS_IP must be a valid IPv4 address." >&2; exit 2; }
X="--exclude=__pycache__ --exclude=*.pre_* --exclude=*.orig --exclude=*.bak --exclude=*.ld19"
tar czf - $X --exclude=ecps295/gazebo -C "$ROGT" src ecps295 | ssh $H 'cd /opt/ecps295 && rm -rf src ecps295 && tar xzf -'
tar czf - $X --exclude=ecps295/gazebo -C "$TWIN" src ecps295 | ssh $H 'mkdir -p /opt/ecps295/twin && cd /opt/ecps295/twin && rm -rf src ecps295 && tar xzf -'
tar czf - $X -C "$HERE" companion config systemd scripts docs 2>/dev/null | ssh $H 'cd /opt/ecps295 && tar xzf -'
ssh $H "ECPS_GCS_IP='$ECPS_GCS_IP' bash /opt/ecps295/scripts/install_pi.sh"
