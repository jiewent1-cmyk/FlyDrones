#!/bin/bash
# stop abs_fix batch/driver processes and SITL instances 10..13 only
for p in $(pgrep -f "python3 batch.py"); do kill $p; done
for p in $(pgrep -f "abs_sim.py"); do kill $p; done
for i in 10 11 12 13; do for p in $(pgrep -f "arducopter .* -I$i( |\$)"); do kill $p; done; done
sleep 2
pgrep -af "arducopter|abs_sim|batch.py" | grep -v pgrep
exit 0
