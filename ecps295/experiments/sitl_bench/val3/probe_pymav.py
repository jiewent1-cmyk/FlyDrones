import inspect
import pymavlink
print("pymavlink:", pymavlink.__file__)
from pymavlink.dialects.v20 import common as c
print(inspect.getsource(c.MAVLink.distance_sensor_send)[:700])
print("=====")
print(inspect.getsource(c.MAVLink.optical_flow_send)[:800])
