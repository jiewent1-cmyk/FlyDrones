#!/usr/bin/env bash
# Run on the Mac: pack the server's hw tree (~/sim/hw/{ecps295,src}) for setup_desktop.sh HW_TAR=...
# Only use it when the H-phase owner says the tree is frozen: it is edited in place, and an anchors.json older than the
# code fails `rl.gates anchor` (seen 2026-10-08). Read-only on the server.
#   bash pack_hw_tree.sh [out.tgz]      then: scp out.tgz desktop:  and on the desktop: HW_TAR=~/out.tgz bash setup_desktop.sh hw
set -euo pipefail
OUT=${1:-hw_tree_$(date +%Y%m%d_%H%M).tgz}
ssh ext-tanjiewen-cloud 'cd ~/sim/hw && tar czf - --exclude=__pycache__ --exclude=rl/out --exclude=rl/results --exclude="._*" ecps295 src' > "$OUT"
ls -la "$OUT"
tar tzf "$OUT" | grep -E "ecps295/(run_g4.py|simclock.py|rl/anchors.json)$"
