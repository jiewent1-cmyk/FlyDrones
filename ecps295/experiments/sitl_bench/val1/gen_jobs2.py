#!/usr/bin/env python3
"""gen_jobs2.py: re-run of the 4 runs lost in the batch conflict + D1b/D2b (companion fully dead)."""
FINAL = ("EK3_RNG_M_NSE=0.1,EK3_HGT_I_GATE=200,SURFTRAK_MODE=0,FENCE_ALT_MAX=2.5,"
         "FENCE_RADIUS=4,FENCE_MARGIN=0.3,SIM_SONAR_RND=0.02,SIM_WIND_T=1")
HARSH = ("SIM_WIND_SPD=1.5,SIM_WIND_TURB=1.0,SIM_FLOW_RND=0.15,FLOW_FXSCALER=50,"
         "FLOW_FYSCALER=50,SIM_SONAR_RND=0.05")
P = 'mtf01_sitl_rng.parm'
lines = []
lines.append('|'.join(['a_hov1_r5', P, '10', '210', FINAL,
                        'HOV --alt 1.0 --hold 120 --no-loiter-test --settle 25 --seconds 260']))
lines.append('|'.join(['a_hov2_r5', P, '10', '210', FINAL,
                        'HOV --alt 2.0 --hold 120 --no-loiter-test --settle 25 --seconds 260']))
lines.append('|'.join(['a_sq_r5', P, '10', '210', FINAL,
                        'SQ --alt 2.0 --side 2.0 --laps 3 --no-loiter-test --settle 25 --seconds 260']))
lines.append('|'.join(['b_d3_r1', P, '10', '30', FINAL + ',' + HARSH + ',SIM_WIND_DIR=0',
                        'D3 --alt 2.0 --side 2.0 --settle 25 --seconds 200']))
HOV2 = 'HOV --alt 2.0 --hold 120 --no-loiter-test --settle 25 --seconds 260'
for i, y in enumerate([30, 150, 270, 90, 210], 1):
    lines.append('|'.join(['d1b_hbfull_r%d' % i, P, '10', str(y), FINAL,
                            HOV2 + ' --vis-cut 40 --hb-cut-full']))
    lines.append('|'.join(['d2b_rcfull_r%d' % i, P, '10', str(y), FINAL,
                            HOV2 + ' --vis-cut 40 --hb-cut-full --fault 40:SIM_RC_FAIL=1']))
open('/home/jacky/sim/val1/jobs_fix_d1b.txt', 'w').write('\n'.join(lines) + '\n')
print(len(lines), 'jobs')
