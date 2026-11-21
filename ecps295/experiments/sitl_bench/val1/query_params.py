#!/usr/bin/env python3
"""query_params.py <port> <PARAM> ... : read MAVLink params from a running SITL."""
import sys
import time

from pymavlink import mavutil

port = int(sys.argv[1])
names = sys.argv[2:]
m = mavutil.mavlink_connection('tcp:127.0.0.1:%d' % port, source_system=253, source_component=190)
if m.wait_heartbeat(timeout=15) is None:
    sys.exit('no heartbeat on port %d' % port)
got = {}
for n in names:
    for _ in range(3):
        m.mav.param_request_read_send(1, 1, n.encode(), -1)
        t = time.time()
        while time.time() - t < 1.5:
            x = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if x and x.param_id == n:
                got[n] = x.param_value
                break
    print('%-22s %s' % (n, got.get(n, 'NOT FOUND')))
