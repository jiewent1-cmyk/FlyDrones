import math
import sys
sys.path.insert(0, '/home/jacky/sim/val3')
from mtf_feed import euler_to_dcm, sitl_flow

# level: dcm[2][2] must be 1 (rotmat.c.z in AP_OpticalFlow_SITL.cpp:67)
d = euler_to_dcm(0.0, 0.0, 0.0)
print('level dcm[2][2] =', round(d[2][2], 6), ' dcm =', [[round(x, 3) for x in r] for r in d])
d = euler_to_dcm(math.radians(10), math.radians(20), math.radians(30))
print('rpy 10/20/30 dcm[2][2] =', round(d[2][2], 6), 'expected', round(math.cos(math.radians(10)) * math.cos(math.radians(20)), 6))
# north-east velocity at yaw 0 -> body x = north, body y = east
fx, fy = sitl_flow((1.0, 0.0, 0.0), 0.0, 0.0, 0.0, (0.0, 0.0, 0.0), 2.0)
print('moving north 1 m/s at 2 m: flow =', round(fx, 5), round(fy, 5), '(expect x=0, y=+0.5 rad/s)')
fx, fy = sitl_flow((0.0, 1.0, 0.0), 0.0, 0.0, 0.0, (0.0, 0.0, 0.0), 2.0)
print('moving east 1 m/s at 2 m: flow =', round(fx, 5), round(fy, 5), '(expect x=-0.5, y=0)')
