# offset-wall turn direction: v2 vs v2_1, 3 brain seeds each; then v2_1 regression subset
M=~/sim/FlyDrones/ecps295/minifly
for v in v2 v2_1; do for seed in 0 1 2; do
  for w in wall_offset_plain wall_offset; do
    tag=dir_${v}_${w}_s$seed
    bash ~/sim/gz_g4.sh ecps295_$w.sdf $tag "--mode approach --seconds 20 --cruise 0.5 --max-forward 0.5 --seed $seed --config $M/$v.yaml --decoder ecps --pilot ecps --retina ecps --out run" > /dev/null 2>&1
    echo "$(date +%T) $tag $(/usr/bin/python3 -c "import json; j=json.load(open(\"$HOME/sim/runs/$tag/run.json\")); print(\"contact\", j[\"contact\"], \"min_clr\", j[\"min_clearance_m\"], \"saccades\", [(e[1], \"L\" if e[2] < 0 else \"R\") for e in j[\"escape_log\"]])")"
  done
done; done
VARIANT=v2_1 DECODER=ecps PILOT=ecps RETINA=ecps bash ~/sim/g4_suite.sh T2_plain T1_v025 T5_hover T6_cage20 > /dev/null 2>&1
echo DONE
