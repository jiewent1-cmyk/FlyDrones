import re
targets = [
 ('/home/jacky/sim/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp', ['accept_packet','msg.sysid','GCS_SYSID_ENFORCE','OPTICAL_FLOW','RANGEFINDER','DISTANCE_SENSOR']),
 ('/home/jacky/sim/ardupilot/libraries/GCS_MAVLink/GCS.cpp', ['OPTICAL_FLOW','SENSOR_OPTICAL_FLOW','healthy()','RANGEFINDER']),
]
for p, pats in targets:
    print('===', p)
    for i, line in enumerate(open(p, errors='ignore'), 1):
        for pat in pats:
            if pat in line:
                print('%5d: %s' % (i, line.rstrip()[:160]))
                break
