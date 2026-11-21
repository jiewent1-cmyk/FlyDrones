#!/bin/bash
# kill_children.sh <batch pid>: kill the run_one.sh children (and their arducopter) of a batch.py
for r in $(pgrep -P $1); do
    echo "kill run_one $r"
    for a in $(pgrep -P $r); do kill -9 $a 2>/dev/null; done
    kill -9 $r 2>/dev/null
done
sleep 3
