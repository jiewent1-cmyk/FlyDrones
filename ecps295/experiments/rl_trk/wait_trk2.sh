# 2026-10-19: start trk2 after 3d hv_sweep (no gz sim running for 2 checks), never after 00:15 PT
cd ~/sim/rl_trk
n=0
while :; do
  h=$(TZ=America/Los_Angeles date +%H%M)
  if [ $((10#$h)) -ge 15 ] && [ $((10#$h)) -lt 1800 ]; then echo "too late/early $h, not started" > wait_trk2.out; exit 0; fi
  if pgrep -f "^gz sim" > /dev/null || pgrep -f "bash par2[.]sh" > /dev/null || ! tail -1 ~/sim/runs/hv_sweep.log 2>/dev/null | grep -q DONE; then n=0; else n=$((n+1)); fi  # 3d: its sweep is done when par2.sh is gone and hv_sweep.log ends with DONE
  [ $n -ge 2 ] && break
  sleep 60
done
echo "start $(date)" > wait_trk2.out
V=~/sim/rl_trk/ecps295/sitl/variants
ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_par_trk.sh trk2_matrix.txt ~/sim/runs/matrix_trk2.log 3
echo "end $(date)" >> wait_trk2.out
