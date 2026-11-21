import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], source_system=254)
m.wait_heartbeat(timeout=30)
for mid in (108, 164):
    m.mav.command_long_send(1, 1, 511, 0, mid, 1e5, 0, 0, 0, 0, 0)
got = set(); t = time.time()
while time.time() - t < 15 and len(got) < 2:
    x = m.recv_match(type=['SIM_STATE', 'SIMSTATE'], blocking=True, timeout=1)
    if x and x.get_type() not in got:
        got.add(x.get_type()); print(x.to_dict())
