# Phase A / B analysis of the slew-fix re-runs (gen_slewcheck.py). Writes ~/sim/runs/slewfix/final/*.md|csv|json
cd ~/sim/runs
P=~/miniconda3/envs/flydrones/bin/python
S=~/sim/FlyDrones/ecps295/ablation_stats.py
O=slewfix/final
mkdir -p $O
E1="^lowbox_s(1?[0-9])_k"
E1C="no_lcb no_lcv no_retreat no_new_cells no_saccade no_efference lad_v3 no_routeB"
E2C="no_lcb no_new_cells no_saccade no_efference"
# A1: same condition, bug (S8 run) vs fixed, paired on the episode
for c in full $E1C; do $P $S ~/sim/runs --ref $c --conds fx_$c --episodes "$E1" --out $O/A1_e1_$c > /dev/null; done
for c in full $E2C; do $P $S ~/sim/runs --ref $c --conds fx_$c --metrics e2 --episodes "^wall_" --out $O/A1_e2_$c > /dev/null; done
# A2: effects under the fixed code (all conditions vs fx_full, Holm across conditions), and the same subset with the bug
fx() { echo "$@" | tr ' ' '\n' | sed 's/^/fx_/' | paste -sd,; }
$P $S ~/sim/runs --ref fx_full --conds $(fx $E1C) --episodes "$E1" --out $O/A2_e1_fixed > /dev/null
$P $S ~/sim/runs --ref full --conds $(echo $E1C | tr ' ' ,) --episodes "$E1" --out $O/A2_e1_bug > /dev/null
$P $S ~/sim/runs --ref fx_full --conds $(fx $E2C) --metrics e2 --episodes "^wall_" --out $O/A2_e2_fixed > /dev/null
$P $S ~/sim/runs --ref full --conds $(echo $E2C | tr ' ' ,) --metrics e2 --episodes "^wall_" --out $O/A2_e2_bug > /dev/null
# B: models vs v4 (fx_full on lowbox / walls, fx_m_v4 in cage20), and vs v3_2
M="v4_oc5 v3_2 es2 es3 wcov2 wcov4 wcov8"
mc() { echo "$@" | tr ' ' '\n' | sed 's/^/fx_m_/' | paste -sd,; }
$P $S ~/sim/runs --ref fx_full --conds $(mc $M) --episodes "^lowbox_s([0-9]|[12][0-9])_k" --out $O/B_low_vs_v4 > /dev/null
$P $S ~/sim/runs --ref fx_m_v3_2 --conds $(mc v4_oc5 es2 es3 wcov2 wcov4 wcov8),fx_full --episodes "^lowbox_s([0-9]|[12][0-9])_k" --out $O/B_low_vs_v3_2 > /dev/null
[ -n "$(ls -d fx_m_es3__cage20_k* 2>/dev/null | head -1)" ] && {
  $P $S ~/sim/runs --ref fx_m_v4 --conds $(mc $M) --episodes "^cage20_k" --out $O/B_cage_vs_v4 > /dev/null
  $P $S ~/sim/runs --ref fx_m_v3_2 --conds $(mc v4 v4_oc5 es2 es3 wcov2 wcov4 wcov8) --episodes "^cage20_k" --out $O/B_cage_vs_v3_2 > /dev/null; }
[ -n "$(ls -d fx_m_es3__wall_* 2>/dev/null | head -1)" ] && {
  $P $S ~/sim/runs --ref fx_full --conds $(mc $M) --metrics e2 --episodes "^wall_" --out $O/B_wall_vs_v4 > /dev/null
  $P $S ~/sim/runs --ref fx_m_v3_2 --conds $(mc v4_oc5 es2 es3 wcov2 wcov4 wcov8),fx_full --metrics e2 --episodes "^wall_" --out $O/B_wall_vs_v3_2 > /dev/null; }
echo analysis done
