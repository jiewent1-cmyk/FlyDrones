import re
p = '/home/jacky/sim/opt_flow2/opt2_sim.py'
for i, line in enumerate(open(p, errors='ignore'), 1):
    if 'SIM_STATE' in line or 'sim_state' in line or '108' in line or '153' in line or 'ATTITUDE' in line or 'LOCAL_POSITION' in line:
        print('%4d: %s' % (i, line.rstrip()[:160]))
