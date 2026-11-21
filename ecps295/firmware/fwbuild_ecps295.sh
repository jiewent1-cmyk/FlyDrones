#!/bin/bash
# ECPS295 custom ArduCopter 4.7.0 for MatekF405-TE: MSP flow + MSP rangefinder + VISO + EKF3 optflow/extnav
set -euo pipefail
LOG=~/sim/fw_ecps295/build.log
mkdir -p ~/opt ~/sim/fw_ecps295
exec > >(tee -a "$LOG") 2>&1
echo "== $(date) start"
TC=~/opt/gcc-arm-none-eabi-10-2020-q4-major
if [ ! -x $TC/bin/arm-none-eabi-gcc ]; then
  cd ~/opt
  curl -fL -o gcc-arm.tar.bz2 https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
  tar xjf gcc-arm.tar.bz2 && rm gcc-arm.tar.bz2
fi
export PATH=$TC/bin:$HOME/miniconda3/envs/ardupilot/bin:$PATH
arm-none-eabi-gcc --version | head -1
WT=~/sim/ap-fw470
if [ ! -d $WT ]; then
  git -C ~/sim/ardupilot worktree add --detach $WT Copter-4.7.0
fi
cd $WT
git submodule update --init --recursive --depth 1 2>&1 | tail -3 || git submodule update --init --recursive
cat > ~/sim/fw_ecps295/extra_hwdef.dat <<'H'
# ECPS295 (2026-11-10): enable the 7 features the stock 1 MB build leaves out
undef AP_OPTICALFLOW_ENABLED
define AP_OPTICALFLOW_ENABLED 1
undef HAL_MSP_SENSORS_ENABLED
define HAL_MSP_SENSORS_ENABLED 1
undef HAL_MSP_OPTICALFLOW_ENABLED
define HAL_MSP_OPTICALFLOW_ENABLED 1
undef HAL_MSP_RANGEFINDER_ENABLED
define HAL_MSP_RANGEFINDER_ENABLED 1
undef EK3_FEATURE_OPTFLOW_FUSION
define EK3_FEATURE_OPTFLOW_FUSION 1
undef HAL_VISUALODOM_ENABLED
define HAL_VISUALODOM_ENABLED 1
undef EK3_FEATURE_EXTERNAL_NAV
define EK3_FEATURE_EXTERNAL_NAV 1
H
${EXTRA_DISABLES:+cat ~/sim/fw_ecps295/extra_disables.dat >> ~/sim/fw_ecps295/extra_hwdef.dat}
./waf configure --board MatekF405-TE --extra-hwdef=$HOME/sim/fw_ecps295/extra_hwdef.dat 2>&1 | tail -5
./waf copter 2>&1 | tail -25
ls -la build/MatekF405-TE/bin/
echo "== $(date) done"
