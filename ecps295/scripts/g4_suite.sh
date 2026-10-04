# G4 flight-test suite: MiniFly flies ArduPilot SITL in Gazebo by camera. Headless, plain worlds (RTF 1.0).
# usage: [VARIANT=v2] [DECODER=ecps] [PILOT=ecps] [RETINA=ecps] bash g4_suite.sh [test ...]      (default: all tests, variant v0 / upstream)
# results: ~/sim/runs/suite_<variant>_<test>/<test>.json, table: ~/sim/runs/suite_<variant>_table.md
VARIANT=${VARIANT:-v0}; DECODER=${DECODER:-upstream}; PILOT=${PILOT:-upstream}; RETINA=${RETINA:-upstream}
CFG=~/sim/FlyDrones/ecps295/minifly/$VARIANT.yaml
TESTS=${@:-"T0_probe T1_v015 T1_v025 T1_v035 T2_plain T3_offset T3b_offset_plain T4_freeze T5_hover T6_cage20"}
declare -A W=( [T0_probe]=camtest_tape [T1_v015]=wall_tape [T1_v025]=wall_tape [T1_v035]=wall_tape
               [T2_plain]=wall_plain [T3_offset]=wall_offset [T3b_offset_plain]=wall_offset_plain [T4_freeze]=wall_tape [T5_hover]=camtest_tape
               [T6_cage20]=cage20 [T7_patrol]=cage20 )
declare -A A=( [T0_probe]="--mode probe"
               [T1_v015]="--mode approach --seconds 30 --cruise 0.3 --max-forward 0.3"
               [T1_v025]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T1_v035]="--mode approach --seconds 30 --cruise 0.7 --max-forward 0.7"
               [T2_plain]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T3_offset]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T3b_offset_plain]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5"
               [T4_freeze]="--mode approach --seconds 30 --cruise 0.5 --max-forward 0.5 --freeze-cam-at 3"
               [T5_hover]="--mode hover --seconds 60"
               [T6_cage20]="--mode approach --seconds 40 --cruise 0.5 --max-forward 0.5"
               [T7_patrol]="--mode patrol --seconds 120 --cruise 0.5 --max-forward 0.5 --fence-turn" )
for t in $TESTS; do
  echo "$(date +%T) $VARIANT $t on ecps295_${W[$t]}"
  bash ~/sim/gz_g4.sh ecps295_${W[$t]}.sdf suite_${VARIANT}_$t "${A[$t]} --config $CFG --decoder $DECODER --pilot $PILOT --retina $RETINA --out $t" > /dev/null 2>&1
done
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
python ~/sim/FlyDrones/ecps295/suite_table.py ~/sim/runs/suite_${VARIANT}_T* > ~/sim/runs/suite_${VARIANT}_table.md
cat ~/sim/runs/suite_${VARIANT}_table.md
