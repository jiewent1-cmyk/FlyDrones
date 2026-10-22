#!/bin/bash
# Run ON the Orange Pi as root after the tree is copied to /opt/ecps295 (see deploy_to_pi.sh on the Mac).
set -euo pipefail
R=/opt/ecps295
if [ -z "${ECPS_GCS_IP:-}" ]; then
    echo "Set ECPS_GCS_IP to the control Mac's static LAN address before installing (dedicated AP recommended)." >&2
    exit 2
fi
case "$ECPS_GCS_IP" in
    (*[!0-9.]*|.*|*..*|*.) echo "ECPS_GCS_IP must be a fixed IPv4 address." >&2; exit 2 ;;
esac
python3 -c 'import ipaddress,sys; ipaddress.IPv4Address(sys.argv[1])' "$ECPS_GCS_IP" || {
    echo "ECPS_GCS_IP must be a valid IPv4 address." >&2
    exit 2
}
if ! command -v nft >/dev/null 2>&1; then
    echo "nftables is required; install the nftables package before deploying the flight service." >&2
    exit 2
fi
mkdir -p /etc/mavlink-router $R/logs
install -d -m 700 $R/config
if [ ! -s $R/config/console.key ]; then
    umask 077
    head -c 32 /dev/urandom > $R/config/console.key
    echo "Generated $R/config/console.key. Copy this file securely to the Mac Keychain before enabling the ground console."
fi
chmod 600 $R/config/console.key
sed "s/ECPS_GCS_IP_REQUIRED/$ECPS_GCS_IP/" $R/config/mavlink-router.conf > /etc/mavlink-router/main.conf
chmod 644 /etc/mavlink-router/main.conf
install -d -m 755 /etc/ecps295
sed "s/ECPS_GCS_IP_REQUIRED/$ECPS_GCS_IP/" $R/config/ecps-security.nft > /etc/ecps295/security.nft
install -m 644 $R/systemd/ecps-security.service /etc/systemd/system/ecps-security.service
systemctl daemon-reload
systemctl enable --now ecps-security.service
install -m 644 $R/systemd/ecps-companion.service /etc/systemd/system/ecps-companion.service
systemctl daemon-reload
systemctl disable --now serial-getty@ttyS0.service 2>/dev/null || true
systemctl enable --now mavlink-router.service
systemctl disable ecps-companion.service 2>/dev/null || true   # installed, NOT started at boot (enable after ground tests)
( cd $R/drivers/vl53l7cx/linux && make -s )
$R/venv/bin/python -c "import os; os.environ['ECPS_TOF_SELFTEST']='1'; import sys; sys.path.insert(0,'$R/companion'); import tof_vl53l7cx"
echo "mavlink-router: $(systemctl is-active mavlink-router)  ecps-companion: $(systemctl is-enabled ecps-companion)"
