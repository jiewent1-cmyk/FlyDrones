#!/bin/bash
# N1c 2026-10-13: after the msh3 (sh5u, cone ToF) matrix finishes, deploy S5 and run s5 with the same cone emulation
set -e
until grep -q "^DONE" ~/sim/runs/matrix_msh3.log 2>/dev/null; do sleep 60; done
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
cd ~/sim/rl_h2/ecps295
python ~/sim/rl_h2/s5gz/deploy_s5.py
python -m py_compile ecps_pilot.py ecps_retina.py run_g4.py
PYTHONPATH=. python ~/sim/rl_h2/s5gz/check_s5.py | tee ~/sim/runs/check_s5.out
grep -q "s5.yaml tof_on True" ~/sim/runs/check_s5.out
grep -q "sh5u.yaml tof_on False" ~/sim/runs/check_s5.out
cd ~/sim/rl_h2
A="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
{ for k in $(seq 0 49); do echo "ms5_s5__lowbox_s${k}_k${k} lowbox_s${k} s5@intact ${k} $A"; done
  for k in $(seq 0 7); do echo "ms5_s5__cage20_k${k} cage20 s5@intact ${k} $A"; done; } > s5_matrix.txt
V=~/sim/rl_h2/ecps295/sitl/variants
export ECPS_TOF_CONE3D=1
echo "launch $(date)"
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh s5_matrix.txt ~/sim/runs/matrix_ms5.log 3
