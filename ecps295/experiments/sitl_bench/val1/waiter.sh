#!/bin/bash
# wait for the A/B batch (pid given) to exit, then run the C/D/E batch
cd /home/jacky/sim/val1
while kill -0 $1 2>/dev/null; do sleep 3; done
python3 batch.py jobs_C.txt jobs_D.txt jobs_E.txt > batch_CDE.log 2>&1
