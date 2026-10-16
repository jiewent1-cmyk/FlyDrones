#!/bin/bash
# Run ON the Orange Pi as root after the tree is copied to /opt/ecps295 (see deploy_to_pi.sh on the Mac).
set -euo pipefail
R=/opt/ecps295
mkdir -p /etc/mavlink-router $R/logs
install -m 644 $R/config/mavlink-router.conf /etc/mavlink-router/main.conf
install -m 644 $R/systemd/ecps-companion.service /etc/systemd/system/ecps-companion.service
systemctl daemon-reload
systemctl disable --now serial-getty@ttyS0.service 2>/dev/null || true
systemctl enable --now mavlink-router.service
systemctl disable ecps-companion.service 2>/dev/null || true   # installed, NOT started at boot (enable after ground tests)
( cd $R/drivers/vl53l7cx/linux && make -s )
$R/venv/bin/python -c "import os; os.environ['ECPS_TOF_SELFTEST']='1'; import sys; sys.path.insert(0,'$R/companion'); import tof_vl53l7cx"
echo "mavlink-router: $(systemctl is-active mavlink-router)  ecps-companion: $(systemctl is-enabled ecps-companion)"
