import os
p = '/home/jacky/miniconda3/envs/flydrones/lib/python3.11/site-packages/pymavlink/message_definitions/v1.0/common.xml'
print(os.path.exists(p))
if os.path.exists(p):
    for i, line in enumerate(open(p, errors='ignore'), 1):
        if 'SIM_STATE' in line or 'SIMSTATE' in line:
            print(i, line.strip())
import pymavlink
from pymavlink.dialects.v20 import ardupilotmega as m
print('MAVLINK_MSG_ID_SIM_STATE =', m.MAVLINK_MSG_ID_SIM_STATE)
print('MAVLINK_MSG_ID_SIMSTATE =', m.MAVLINK_MSG_ID_SIMSTATE)
