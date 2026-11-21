#!/bin/bash
# run after the current batch: chain.sh <wait-log> <out-log> <job files...>
cd /home/jacky/sim/abs_fix
W=$1; O=$2; shift 2
./waitbatch.sh $W > /dev/null
exec python3 batch.py "$@" > $O 2>&1
