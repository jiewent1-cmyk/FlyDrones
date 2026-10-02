# usage: run_sitl.sh <tag> [extra .parm files...]
TAG=$1; shift
source ~/miniconda3/etc/profile.d/conda.sh; conda activate ardupilot
D=~/sim/runs/$TAG; rm -rf $D; mkdir -p $D; cd $D
# SITL 4.7 only resolves --model <frame>:<file>.json relative to cwd: copy it in
if [[ "$MODEL" == *:*.json ]]; then J=${MODEL#*:}; cp "$J" .; MODEL="${MODEL%%:*}:$(basename "$J")"; fi
DEF=~/sim/ardupilot/Tools/autotest/default_params/copter.parm
for f in "$@"; do DEF="$DEF,$f"; done
exec ~/sim/ardupilot/build/sitl/bin/arducopter --model ${MODEL:-quad} --speedup 1 -I${INST:-0} --home ${HOME_LOC:-33.6430,-117.8420,20,0} --defaults "$DEF" > sitl.out 2>&1
