# usage: smoke.sh <tag> <hover_s> [extra .parm...]
TAG=$1; HOV=$2; shift 2
bash ~/sim/run_sitl.sh $TAG "$@" &
SP=$!
sleep 3
source ~/miniconda3/etc/profile.d/conda.sh; conda activate flydrones
timeout $((HOV+200)) python ~/sim/FlyDrones/ecps295/sitl_smoke.py --hover $HOV 2>&1 | grep -v "EOF on TCP" | head -80
kill $SP 2>/dev/null; sleep 1; pkill -x arducopter
