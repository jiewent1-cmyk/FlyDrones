# G4 flight-test suite: MiniFly (upstream synthetic, default config) flies ArduPilot SITL in Gazebo by camera.
# Headless, plain worlds (RTF 1.0). Results: ~/sim/runs/suite_<test>/<test>.json, table in ~/sim/runs/suite_table.md
# usage: bash g4_suite.sh [test ...]     (default: all)
TESTS=${@:-"T1_v015 T1_v025 T1_v035 T2_plain T3_offset T4_freeze T5_hover"}
declare -A W=( [T1_v015]=wall_tape [T1_v025]=wall_tape [T1_v035]=wall_tape [T2_plain]=wall_plain
               [T3_offset]=wall_offset [T4_freeze]=wall_tape [T5_hover]=camtest_tape )
declare -A A=( [T1_v015]="--mode approach --seconds 30 --cruise 0.3 --max-forward 0.3"
               [T1_v025]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T1_v035]="--mode approach --seconds 30 --cruise 0.7 --max-forward 0.7"
               [T2_plain]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T3_offset]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T4_freeze]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5 --freeze-cam-at 3"
               [T5_hover]="--mode hover --seconds 90" )
for t in $TESTS; do
  echo "$(date +%T) $t on ecps295_${W[$t]}"
  bash ~/sim/gz_g4.sh ecps295_${W[$t]}.sdf suite_$t "${A[$t]} --out $t" > /dev/null 2>&1
done
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
python ~/sim/FlyDrones/ecps295/suite_table.py ~/sim/runs/suite_T* > ~/sim/runs/suite_table.md
cat ~/sim/runs/suite_table.md
