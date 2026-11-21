#!/bin/bash
set -euo pipefail
export PATH=$HOME/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$HOME/miniconda3/envs/ardupilot/bin:$PATH
cd ~/sim/ap-fw470
git status --short | head -5
git describe --tags --always
./waf configure --board MatekF405-TE --extra-hwdef=$HOME/sim/fw_ecps295_mtf/extra_hwdef.dat 2>&1 | tail -5
./waf copter 2>&1 | tail -25
ls -la build/MatekF405-TE/bin/
arm-none-eabi-size build/MatekF405-TE/bin/arducopter
python3 Tools/scripts/extract_features.py build/MatekF405-TE/bin/arducopter > ~/sim/fw_ecps295_mtf/features.txt 2>&1 || echo "extract_features failed"
cp build/MatekF405-TE/bin/arducopter.apj ~/sim/fw_ecps295_mtf/ECPS295_arducopter_4.7.0_MatekF405-TE_mtf01_20261119.apj
cp build/MatekF405-TE/bin/arducopter.bin ~/sim/fw_ecps295_mtf/ECPS295_arducopter_4.7.0_MatekF405-TE_mtf01_20261119.bin
echo "== done $(date)"
