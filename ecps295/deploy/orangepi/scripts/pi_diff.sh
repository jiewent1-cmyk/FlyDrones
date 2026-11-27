#!/bin/bash
# Read-only: compare the companion code in this checkout with what runs on the Pi (/opt/ecps295), file by file.
# Changes nothing on the Pi. Prints: SAME / DIFF / ONLY-LOCAL / ONLY-PI per file, then the config keys that differ.
#
#   pi_diff.sh [ssh-host]          default root@100.112.163.133 (NetBird)
#
# Layout on the Pi (deploy_to_pi.sh): /opt/ecps295/{companion,config,scripts} from ecps295/deploy/orangepi,
# /opt/ecps295/ecps295 and /opt/ecps295/src from the runtime tree.
set -euo pipefail
H=${1:-root@100.112.163.133}
REPO=$(cd "$(dirname "$0")/../../../.." && pwd)
DEP="$REPO/ecps295/deploy/orangepi"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

local_sums() {  # $1 = base dir, $2.. = sub dirs; prints "<md5>  <relpath>"
  local base=$1; shift
  (cd "$base" && find "$@" -type f -name '*.py' ! -path '*__pycache__*' -print0 2>/dev/null | sort -z | xargs -0 md5sum 2>/dev/null || true)
}
md5sum --version >/dev/null 2>&1 || md5sum() { for f in "$@"; do printf '%s  %s\n' "$(md5 -q "$f")" "$f"; done; }

{ local_sums "$DEP" companion scripts; local_sums "$REPO" ecps295 src | grep -v ' ecps295/deploy/\| ecps295/experiments/\| ecps295/gazebo/'; } | sort -k2 > "$tmp/local"
ssh -o ConnectTimeout=8 -o BatchMode=yes "$H" 'cd /opt/ecps295 && find companion scripts ecps295 src -type f -name "*.py" ! -path "*__pycache__*" -print0 2>/dev/null | sort -z | xargs -0 md5sum' \
  | grep -v ' ecps295/experiments/\| ecps295/gazebo/' | sort -k2 > "$tmp/pi"

join -1 2 -2 2 -a 1 -a 2 -e - -o 0,1.1,2.1 "$tmp/local" "$tmp/pi" | awk '
  $2 == "-" { print "ONLY-PI    " $1; next }
  $3 == "-" { print "ONLY-LOCAL " $1; next }
  $2 != $3  { print "DIFF       " $1; next }
            { same++ }
  END { print same " files identical" }' | sort

echo "--- companion.yaml: lines that differ (local < > Pi)"
ssh -o BatchMode=yes "$H" 'cat /opt/ecps295/config/companion.yaml' > "$tmp/pi.yaml"
diff <(grep -v '^\s*#' "$DEP/config/companion.yaml" | sed 's/ *#.*//' | grep -v '^\s*$') \
     <(grep -v '^\s*#' "$tmp/pi.yaml" | sed 's/ *#.*//' | grep -v '^\s*$') || true
