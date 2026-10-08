# usage: run_resume.sh <dir with gen_*.py output> [instances=2]
# Like run_ablation.sh, but resumable after an interruption (power cut, reboot): for every batch in batches.tsv it
# only runs the lines whose run is missing or invalid (no run.json or whole-run RTF < 0.95), first on N parallel
# instances, then the remaining invalid ones on a single instance (up to 2 more tries). Safe to start again any time.
A=$1; N=${2:-2}
V=$HOME/sim/FlyDrones/ecps295/sitl/variants
R=$HOME/sim/runs
todo() {  # $1 batch file -> lines still to run
  while read -r tag rest; do
    [ -z "$tag" ] && continue
    ok=$(awk '/^RTF run/ {print ($3 >= 0.95)}' $R/$tag/rtf.log 2>/dev/null)
    [ -f $R/$tag/run.json ] && [ "$ok" = 1 ] || echo "$tag $rest"
  done < "$1"
}
while IFS=$'\t' read -r b nav extra; do
  extra=$(echo "$extra" | sed "s#\$V#$V#g")
  todo $A/$b.txt > $A/$b.todo.txt
  n=$(grep -c . $A/$b.todo.txt)
  [ "$n" = 0 ] && { echo "$(date +%T) batch $b complete"; continue; }
  echo "$(date +%T) batch $b: $n of $(grep -c . $A/$b.txt) runs to do, NAV=$nav, $N instances"
  NAV=$nav EXTRA_PARM=$extra bash ~/sim/run_matrix_par.sh $A/$b.todo.txt $A/$b.resume.log $N
  for try in 1 2; do
    todo $A/$b.txt > $A/$b.redo.txt
    n=$(grep -c . $A/$b.redo.txt); [ "$n" = 0 ] && break
    echo "$(date +%T) batch $b: re-running $n invalid runs on one instance (try $try)"
    NAV=$nav EXTRA_PARM=$extra INST=0 bash ~/sim/run_matrix.sh $A/$b.redo.txt $A/$b.redo$try.log
  done
done < $A/batches.tsv
echo "$(date +%T) all batches done"
