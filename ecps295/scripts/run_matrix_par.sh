# usage: run_matrix_par.sh <matrix file> <log> <instances>   (same line format as run_matrix.sh)
# Splits the lines round-robin over N parallel Gazebo/SITL instances (gz_g4.sh INST=0..N-1), each running its share
# one after another; NAV / EXTRA_PARM come from the environment (one batch per parameter set). Each worker logs to
# <log>.partK while running; the parts are merged into <log> with a final "DONE" when all workers have finished.
M=$1; LOG=$2; N=${3:-3}
Q=$(mktemp -d)
grep -v '^\s*#' "$M" | grep -v '^\s*$' | awk -v n=$N -v q=$Q '{print > (q "/" (NR - 1) % n)}'
for k in $(seq 0 $((N - 1))); do
  [ -f $Q/$k ] || continue
  ( sleep $((k * 20)); INST=$k bash ~/sim/run_matrix.sh $Q/$k $LOG.part$k ) &
done
wait
cat $LOG.part* >> $LOG 2>/dev/null; rm -f $LOG.part*
grep -v '^DONE' $LOG > $LOG.tmp; mv $LOG.tmp $LOG; echo DONE >> $LOG
rm -rf $Q
