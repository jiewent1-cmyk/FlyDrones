"""Obstacle layouts exported by gazebo/make_quad.py (<world>.layout.json) as flydrones Rooms."""

from __future__ import annotations

import json

from flydrones.drones.sim import Box, Room


def room_from_layout(path: str) -> Room:
    """Gazebo ENU obstacles -> flydrones Room in the telemetry frame (x = north, y = east, as LOCAL_POSITION_NED)."""
    lay = json.load(open(path))
    boxes = []
    for b in lay["boxes"]:
        (e, n, z), (se, sn, sz) = b["center_enu"], b["size_enu"]
        boxes.append(Box((n - sn / 2, e - se / 2, z - sz / 2), (n + sn / 2, e + se / 2, z + sz / 2), 0.2, b["name"]))
    return Room(size_x=lay["floor_m"], size_y=lay["floor_m"], height=3.0, boxes=boxes)
