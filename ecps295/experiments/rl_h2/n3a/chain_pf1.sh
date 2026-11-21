# after wp2: patrol regression of the fence-turn fix (2026-10-17), same worlds/seeds as g7_sc2_tf10
until grep -q DONE ~/sim/runs/matrix_wp2.log 2>/dev/null; do sleep 60; done
cd ~/sim/rl_h2; V=~/sim/rl_h2/ecps295/sitl/variants
ECPS_TOF_CONE3D=1 ECPS_SIMCLOCK=1 NAV=flow EXTRA_PARM=$V/G_extnav_height.parm,$V/companion_fs.parm,$V/H_terrain_fast.parm bash run_matrix_par_mine.sh n3a/pf1_matrix.txt ~/sim/runs/matrix_pf1.log 3
