# usage: run_ablation.sh <dir with gen_ablation.py output> [instances=2] [batch ...]
# Runs every batch (batches.tsv: name, NAV, EXTRA_PARM with $V = ecps295/sitl/variants) with run_matrix_par.sh, then
# re-runs every run that is missing or has a whole-run RTF < 0.95 on a single instance (up to 2 more tries).
# Logs: <dir>/<batch>.log; progress: grep -c . <dir>/*.log*
A=$1; N=${2:-2}; shift $(( $# < 2 ? $# : 2 ))
V=$HOME/sim/FlyDrones/ecps295/sitl/variants
R=$HOME/sim/runs
want="$*"
while IFS=$'\t' read -r b nav extra; do
  [ -n "$want" ] && [[ " $want " != *" $b "* ]] && continue
  extra=$(echo "$extra" | sed "s#\$V#$V#g")
  [ -f $A/$b.log ] && grep -q DONE $A/$b.log && { echo "$b already done"; continue; }
  echo "$(date +%T) batch $b: $(grep -c . $A/$b.txt) runs, NAV=$nav EXTRA_PARM=$extra, $N instances"
  NAV=$nav EXTRA_PARM=$extra bash ~/sim/run_matrix_par.sh $A/$b.txt $A/$b.log $N
  for try in 1 2; do
    : > $A/$b.redo.txt
    while read -r tag rest; do
      ok=$(awk '/^RTF run/ {print ($3 >= 0.95)}' $R/$tag/rtf.log 2>/dev/null)
      [ -f $R/$tag/run.json ] && [ "$ok" = 1 ] || echo "$tag $rest" >> $A/$b.redo.txt
    done < $A/$b.txt
    n=$(grep -c . $A/$b.redo.txt); [ "$n" = 0 ] && break
    echo "$(date +%T) batch $b: re-running $n invalid runs on one instance (try $try)"
    NAV=$nav EXTRA_PARM=$extra INST=0 bash ~/sim/run_matrix.sh $A/$b.redo.txt $A/$b.redo$try.log
  done
done < $A/batches.tsv
echo "$(date +%T) all batches done"
