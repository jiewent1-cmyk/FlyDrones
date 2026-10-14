#!/bin/bash
# stage E screen: train 8 lifetimes then frozen-g test, per setting. usage: e_screen.sh <procs> <nice> tag:json ...
P=$1; N=$2; shift 2
for spec in "$@"; do
  tag=${spec%%:*}; mb=${spec#*:}
  nice -n $N python -m rl.mb_life --tag $tag --lives 0-7 --procs 8 --mb "$mb" >> rl/out/mb/$tag.log 2>&1
  nice -n $N python -m rl.mb_life --tag $tag --lives 0-7 --test --procs $P >> rl/out/mb/$tag.log 2>&1
done
echo E_SCREEN_DONE >> rl/out/mb/screen.done
