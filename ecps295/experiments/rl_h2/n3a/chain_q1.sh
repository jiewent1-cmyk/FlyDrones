#!/bin/bash
# ROG queue 1 (2026-10-15, after the c065 matrix; quiet hours are handled by run_matrix_mine.sh, no start 00:55-07:00 PT):
#   N3 round-2 Gazebo acceptance of CMA candidate n3s1 (seed 1, best validation = gen 4, cruise 0.525) and
#   N4 map lower band g7_w2map_lo45 (s7_w2map + band_lo_m 0.45: low box tops count as obstacles), set A interleaved,
#   then n3s1 set B. All paired with g7_w2map__* (cruise 0.5). Yamls: n3a/gz_export.py; patches: n3a/patch_shdbg.py
#   (debug columns) and patch_bandlo.py (default-preserving) applied before launch. 177 runs, about 3.3 h with 3 instances.
# usage: chain_q1.sh [--print]   (--print only writes / prints the matrix)
set -e
cd ~/sim/rl_h2 2>/dev/null || cd "$(dirname "$0")/.."
mkdir -p n3a
lines() {  # <tag> <variant yaml> <cruise> <A|B>
  local a="--mode patrol --seconds 120 --cruise $3 --max-forward $3 --fence-turn --ext-height"
  if [ "$4" = A ]; then
    for k in $(seq 0 49); do echo "$1__lowbox_s${k}_k${k} lowbox_s${k} $2@intact ${k} $a"; done
    for k in $(seq 0 7); do echo "$1__cage20_k${k} cage20 $2@intact ${k} $a"; done
  else
    for k in $(seq 0 49); do echo "$1__lowbox_s${k}_k$((k + 100)) lowbox_s${k} $2@intact $((k + 100)) $a"; done
    for k in $(seq 8 15); do echo "$1__cage20_k${k} cage20 $2@intact ${k} $a"; done
  fi
}
# 3 replays of the S7b graze (g7_w2map__lowbox_s32_k132: slow side drift while turning in place, not over a box) with the
# shield debug columns, then the paired runs
{ for r in 1 2 3; do echo "dbg${r}_w2map__lowbox_s32_k132 lowbox_s32 s7_w2map@intact 132 --mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"; done
  paste -d '\n' <(lines n3s1 n3s1 0.525 A) <(lines g7_w2map_lo45 s7_w2map_lo45 0.5 A); lines n3s1 n3s1 0.525 B; } > n3a/q1_matrix.txt
if [ "$1" = --print ]; then wc -l < n3a/q1_matrix.txt; exit 0; fi
for y in n3s1 s7_w2map_lo45; do test -f ecps295/minifly/$y.yaml; done
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch q1 $(wc -l < n3a/q1_matrix.txt) runs $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh n3a/q1_matrix.txt ~/sim/runs/matrix_q1.log 3
