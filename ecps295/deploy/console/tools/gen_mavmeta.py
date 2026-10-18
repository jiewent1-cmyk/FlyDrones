"""Emit Swift MAVLink metadata (wire layout + CRC_EXTRA) for the messages the console decodes, from pymavlink."""
import struct, sys
from pymavlink.dialects.v20 import ardupilotmega as m

NAMES = ["HEARTBEAT", "SYS_STATUS", "SYSTEM_TIME", "SET_MODE", "SCALED_PRESSURE", "ATTITUDE", "LOCAL_POSITION_NED",
         "GLOBAL_POSITION_INT", "RC_CHANNELS", "VFR_HUD", "COMMAND_LONG", "COMMAND_ACK", "OPTICAL_FLOW",
         "VISION_POSITION_ESTIMATE", "SIM_STATE", "DISTANCE_SENSOR", "BATTERY_STATUS", "RANGEFINDER",
         "EKF_STATUS_REPORT", "VIBRATION", "STATUSTEXT", "POWER_STATUS", "POSITION_TARGET_LOCAL_NED", "MEMINFO"]
SZ = {"float": 4, "double": 8, "char": 1, "int8_t": 1, "uint8_t": 1, "uint8_t_mavlink_version": 1, "int16_t": 2,
      "uint16_t": 2, "int32_t": 4, "uint32_t": 4, "int64_t": 8, "uint64_t": 8}
TY = {"float": "f32", "double": "f64", "char": "char", "int8_t": "i8", "uint8_t": "u8", "uint8_t_mavlink_version": "u8",
      "int16_t": "i16", "uint16_t": "u16", "int32_t": "i32", "uint32_t": "u32", "int64_t": "i64", "uint64_t": "u64"}
out = []
for name in NAMES:
    cls = getattr(m, "MAVLink_" + name.lower() + "_message")
    ids = cls.id
    order = cls.ordered_fieldnames  # wire order
    types = dict(zip(cls.fieldnames, cls.fieldtypes))
    alen = dict(zip(cls.fieldnames, cls.array_lengths)) if hasattr(cls, "array_lengths") else {}
    # array_lengths in pymavlink are in wire order (ordered_fieldnames)
    alen = dict(zip(order, cls.array_lengths))
    off = 0
    fields = []
    for f in order:
        t = types[f]
        n = alen.get(f, 0) or 0
        size = SZ[t] * max(1, n)
        fields.append((f, TY[t], off, n))
        off += size
    assert off == struct.calcsize(cls.unpacker.format) if hasattr(cls, "unpacker") else True, name
    out.append((ids, name, cls.crc_extra, off, fields))
print("// Generated from pymavlink 2.4.50 (dialect ardupilotmega) by tools/gen_mavmeta.py - do not edit by hand.")
print("// wire layout per message: (name, type, byte offset, array length); payload length includes extensions.")
print("let mavMessages: [UInt32: MavMessageSpec] = [")
for ids, name, crc, ln, fields in out:
    fs = ", ".join(f'MavField("{f}", .{t}, {o}, {n})' for f, t, o, n in fields)
    print(f'    {ids}: MavMessageSpec(id: {ids}, name: "{name}", crcExtra: {crc}, length: {ln}, fields: [{fs}]),')
print("]")
