# ROG thermal log during long simulation batches: one CSV row every 30 s to ~/sim/runs/thermal.csv,
# ALERT lines to ~/sim/runs/thermal_alerts.log when the CPU (Tctl) >= 95 C, the GPU >= 87 C, or the GPU reports
# thermal / power throttling. Stop with: pkill -f thermal_log.sh
OUT=$HOME/sim/runs/thermal.csv
ALERT=$HOME/sim/runs/thermal_alerts.log
hw() { for h in /sys/class/hwmon/hwmon*; do [ "$(cat $h/name)" = "$1" ] && cat $h/$2 2>/dev/null && return; done; echo 0; }
[ -f $OUT ] || echo "time,cpu_tctl_c,gpu_c,igpu_c,nvme_max_c,cpu_fan_rpm,gpu_fan_rpm,cpu_mhz_avg,load1,gpu_util,gpu_w,gpu_throttle" > $OUT
while true; do
  t=$(date +%F_%T)
  cpu=$(( $(hw k10temp temp1_input) / 1000 ))
  igpu=$(( $(hw amdgpu temp1_input) / 1000 ))
  nv=0; for h in /sys/class/hwmon/hwmon*; do [ "$(cat $h/name)" = nvme ] && for f in $h/temp*_input; do v=$(( $(cat $f) / 1000 )); [ $v -gt $nv ] && nv=$v; done; done
  cf=$(hw asus fan1_input); gf=$(hw asus fan2_input)
  mhz=$(awk '/MHz/ {s+=$4; n++} END {printf "%d", s/n}' /proc/cpuinfo)
  ld=$(cut -d" " -f1 /proc/loadavg)
  IFS=, read -r g gu gw gt <<< "$(nvidia-smi --query-gpu=temperature.gpu,utilization.gpu,power.draw,clocks_event_reasons.active --format=csv,noheader,nounits | tr -d ' ')"
  echo "$t,$cpu,$g,$igpu,$nv,$cf,$gf,$mhz,$ld,$gu,$gw,$gt" >> $OUT
  # 0x20 SW thermal, 0x40 HW thermal, 0x80 HW power brake (nvidia-smi clocks_event_reasons bits)
  thr=$(( ${gt:-0} & 0xE0 ))
  if [ "$cpu" -ge 95 ] || [ "${g:-0}" -ge 87 ] || [ "$thr" -ne 0 ]; then
    echo "$t ALERT cpu ${cpu}C gpu ${g}C gpu_throttle $gt cpu_mhz $mhz" >> $ALERT
  fi
  sleep 30
done
