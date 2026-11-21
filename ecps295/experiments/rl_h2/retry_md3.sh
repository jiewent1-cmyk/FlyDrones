#!/bin/bash
# wait for the d3 matrix to finish, then rerun every FAILED line once (arming / EKF init failures on notex floors)
cd ~/sim/rl_h2
until grep -q "^DONE" ~/sim/runs/matrix_md3.log 2>/dev/null; do sleep 300; done
grep FAILED ~/sim/runs/matrix_md3.log | cut -d" " -f1 | sort -u > ~/sim/runs/md3_failed.txt
grep -F -w -f ~/sim/runs/md3_failed.txt d3_matrix.txt > d3r_matrix.txt
[ -s d3r_matrix.txt ] || exit 0
V=~/sim/rl_h2/ecps295/sitl/variants
NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm bash run_matrix_par_mine.sh d3r_matrix.txt ~/sim/runs/matrix_md3r.log 3
