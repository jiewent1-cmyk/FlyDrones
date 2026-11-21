#!/bin/bash
# 建议一节的单独实验：判据 B 持续性(b_hold) 与 斜率补偿(slope) 的效果
cd /home/jacky/sim/val7
BASE="fc=0.5,tau=30,E=0.7,T=1.0,R=0.6"
echo "=== 误报（第1步 220 次飞行）==="
for bh in 1 2 3; do
  python offline_eval.py fp --runs runs_s1.txt --set "$BASE,b_hold=$bh" --jobs 4
done
for ts in 10 30; do
  python offline_eval.py fp --runs runs_s1.txt --set "$BASE,slope=1,tau_s=$ts" --jobs 4
done
echo "=== 检出（第2步 55 次飞行）==="
for bh in 1 2 3; do
  python offline_eval.py detgrid --runs runs_s2.txt --set "$BASE,b_hold=$bh" --jobs 4
done
for ts in 10 30; do
  python offline_eval.py detgrid --runs runs_s2.txt --set "$BASE,slope=1,tau_s=$ts" --jobs 4
done
