from pymavlink.dialects.v20 import ardupilotmega as m
for name, mid in (('SIM_STATE', 108), ('SIMSTATE', 164), ('OPTICAL_FLOW', 100), ('DISTANCE_SENSOR', 132)):
    print(name, m.mavlink_map[mid].fieldnames)
