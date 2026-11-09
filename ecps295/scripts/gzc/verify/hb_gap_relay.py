"""UDP relay between SITL and the companion that can hide the FC's HEARTBEAT (2026-11-04, resume_e2e.py): while the flag
file exists, HEARTBEAT frames from SITL are dropped and everything else passes, the same gap a slow simulation causes.

usage: hb_gap_relay.py <listen port for SITL> <companion port> <flag file>
"""

import os
import select
import socket
import sys

listen, comp, flag = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
fc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fc.bind(("127.0.0.1", listen))
cp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
fc_addr = None


def without_heartbeat(d: bytes) -> bytes:
    out, i = bytearray(), 0
    while i < len(d):
        if d[i] == 0xFD and i + 10 <= len(d):
            n = 12 + d[i + 1] + (13 if d[i + 2] & 1 else 0)
            mid = int.from_bytes(d[i + 7:i + 10], "little")
        elif d[i] == 0xFE and i + 6 <= len(d):
            n, mid = 8 + d[i + 1], d[i + 5]
        else:
            return d  # not a clean frame boundary: pass the datagram as it is
        if mid != 0:
            out += d[i:i + n]
        i += n
    return bytes(out)


while True:
    r, _, _ = select.select([fc, cp], [], [], 1.0)
    if fc in r:
        d, fc_addr = fc.recvfrom(65535)
        if os.path.exists(flag):
            d = without_heartbeat(d)
        if d:
            cp.sendto(d, ("127.0.0.1", comp))
    if cp in r:
        d, _ = cp.recvfrom(65535)
        if fc_addr is not None:
            fc.sendto(d, fc_addr)
