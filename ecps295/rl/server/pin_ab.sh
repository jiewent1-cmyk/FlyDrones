#!/bin/bash
# NUMA pinning A/B, run after the validation batch: same world / flight, unpinned vs pinned.
cd ~/sim
while ! grep -q "all done" gz_val.log; do sleep 30; done
sleep 30
A="--mode patrol --seconds 60 --cruise 0.5 --max-forward 0.5 --fence-turn --decoder ecps --pilot ecps --retina ecps --out run --config $HOME/sim/ecps295/minifly/v3_2.yaml"
group() {  # name script cores n
  local name=$1 script=$2 cores=$3 n=$4
  echo "== $name  load before: $(cut -d" " -f1 /proc/loadavg)"
  local t0=$(date +%s)
  for i in $(seq 0 $((n - 1))); do PIN_CORES=$cores bash $script $i ecps295_cage20.sdf ab_${name}_i$i "$A --seed $i" > /dev/null 2>&1 & sleep 2; done
  sleep 90; echo "   load during: $(cut -d" " -f1 /proc/loadavg)"
  wait
  echo "   wall $(( $(date +%s) - t0 )) s"
  for i in $(seq 0 $((n - 1))); do echo "   i$i $(grep -h "RTF run" runs/ab_${name}_i$i/rtf.log 2>/dev/null) $(grep -o "CPUS=[^ ]*" runs/ab_${name}_i$i/params.log 2>/dev/null)"; done
}
group unpinned4 gz_inst.sh 0 4
group pin6x4 gz_inst_pin.sh 6 4
group pin4x4 gz_inst_pin.sh 4 4
group unpinned1 gz_inst.sh 0 1
group pin6x1 gz_inst_pin.sh 6 1
echo AB_DONE
