import glob
import os
pats = glob.glob('/home/jacky/miniconda3/envs/flydrones/lib/python*/site-packages/pymavlink/message_definitions/v1.0/common.xml')
print(pats)
for p in pats:
    for i, line in enumerate(open(p, errors='ignore'), 1):
        if 'SIM_STATE' in line or 'SIMSTATE' in line:
            print(p, i, line.strip())
