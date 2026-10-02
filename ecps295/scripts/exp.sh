# usage: [MODEL=..] [INST=n] exp.sh <tag> "<python script + args>" [extra .parm...]
# SITL instance n listens on tcp 5760+10n; the script gets --url appended. Log in runs/<tag>/exp.log
TAG=$1; CMD=$2; shift 2
INST=${INST:-0}
bash ~/sim/run_sitl.sh $TAG "$@" &
SP=$!
sleep 3
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
cd ~/sim/runs/$TAG && PYTHONPATH=~/sim/FlyDrones/ecps295 timeout 900 python $CMD --url tcp:127.0.0.1:$((5760+10*INST)) 2>&1 | grep -v "EOF on TCP" > exp.log
kill $SP 2>/dev/null; sleep 1
cat exp.log
