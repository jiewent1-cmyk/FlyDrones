#!/bin/bash
# Orange Pi profiling matrix for the ECPS295 companion (bench mode, simulated camera + ToF, s7_w2map).
# Runs on the Pi as root; restores the CPU governor afterwards. Logs: /opt/ecps295/logs/*_bench_P*
set -u
R=/opt/ecps295; PY=$R/venv/bin/python; C=$R/companion/ecps_companion.py
SEC=${SEC:-120}
gov() { for g in /sys/devices/system/cpu/cpufreq/policy*/scaling_governor; do echo "$1" > "$g"; done; }
run() { local tag=$1; shift; echo "== $tag $(date +%T)"; $PY -u $C --mode bench --camera sim --tof sim --quiet --tag "$tag" "$@" | tail -2; sleep 20; }
trap 'gov schedutil; pkill -f "stress_load" 2>/dev/null' EXIT
gov schedutil
run P1_a76            --seconds $SEC --affinity 6,7
run P2_a55            --seconds $SEC --affinity 0-5
run P3_any            --seconds $SEC
run P4_a55one         --seconds $SEC --affinity 0
gov performance
run P5_a76_perf       --seconds $SEC --affinity 6,7
gov schedutil
# background load on the six A55 cores (e.g. logging, router, GCS link, a camera encoder)
for c in 0 1 2 3 4 5; do taskset -c $c $PY -c "import time
while True: sum(i*i for i in range(10000))" stress_load & done
run P6_a76_bgload     --seconds $SEC --affinity 6,7
run P7_any_bgload     --seconds $SEC
pkill -f stress_load
sleep 30
run P8_a76_thermal15  --seconds 900 --affinity 6,7
echo "== done $(date +%T)"
