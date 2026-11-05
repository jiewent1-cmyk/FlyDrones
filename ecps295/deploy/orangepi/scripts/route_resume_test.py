"""Route resume after a brief control loss (console_link.Operator.on_control, 2026-11-04), no hardware.

usage: python route_resume_test.py    (exit 0 = all pass)
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "companion"))
from console_link import Operator  # noqa: E402

PTS = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


class Pilot:
    wp_idx, wp_done_t, wp_radius = 0, None, 0.35

    def __init__(self):
        self.goals = []

    def set_waypoints(self, pts):
        self.goals.append(list(pts))
        self.wp_idx = 0

    def clear_waypoints(self):
        pass

    def __getattr__(self, name):  # anything else the operator asks of the pilot's layers
        raise AttributeError(name)


class Drone:
    GUIDED = "GUIDED"
    mode, arm = "GUIDED", True

    def fc_mode(self):
        return self.mode

    def armed(self):
        return self.arm


def flying(handover="HOLD", resume_s=0.3, n=None):
    d, p = Drone(), Pilot()
    op = Operator({"route_resume_s": resume_s}, 2.7, p, d)
    op.handover = handover
    op.events = []
    op.on_event = lambda src, text: op.events.append(text)
    op.on_control(True)
    assert op.set_route(PTS) == ""
    op.on_control(True)
    assert op.mode == "GOTO"
    if n is not None:
        op.pilot.wp_idx = n
    return op, d


def resumed(op):
    return any("route resumed" in e for e in op.events)


def test_brief_gap_resumes_at_current_waypoint():
    op, _ = flying(n=1)
    op.on_control(False)
    op.on_control(False)
    op.on_control(True)
    assert op.mode == "GOTO" and resumed(op)
    assert op.route == PTS[1:] and op.pilot.goals[-1] == PTS[1:], (op.route, op.pilot.goals)


def test_long_gap_is_a_hand_over():
    op, _ = flying()
    op.on_control(False)
    time.sleep(0.35)
    op.on_control(True)
    assert op.mode == "HOLD" and not op.route and not resumed(op)


def test_zero_window_never_resumes():
    op, _ = flying(resume_s=0.0)
    op.on_control(False)
    time.sleep(0.01)
    op.on_control(True)
    assert op.mode == "HOLD" and not resumed(op)


def test_pilot_mode_switch_is_a_hand_over():
    op, d = flying()
    d.mode = "LOITER"
    op.on_control(False)
    d.mode = "GUIDED"
    op.on_control(False)
    op.on_control(True)
    assert op.mode == "HOLD" and not resumed(op)


def test_mode_switch_later_in_the_gap_is_a_hand_over():
    op, d = flying()
    op.on_control(False)
    d.mode = "LOITER"
    op.on_control(False)
    d.mode = "GUIDED"
    op.on_control(True)
    assert op.mode == "HOLD" and not resumed(op)


def test_disarm_is_a_hand_over():
    op, d = flying()
    op.on_control(False)
    d.arm = False
    op.on_control(False)
    d.arm = True
    op.on_control(True)
    assert op.mode == "HOLD" and not resumed(op)


def test_emergency_stop_is_a_hand_over():
    op, _ = flying()
    op.on_control(False, False)
    op.on_control(False, True)
    op.on_control(True, True)
    assert op.mode == "HOLD" and not resumed(op)


def test_operator_hold_during_the_gap_stays():
    op, _ = flying()
    op.on_control(False)
    op.set_mode("HOLD", "console")
    op.on_control(False)
    op.on_control(True)
    assert not resumed(op) and not op.route


def test_new_route_during_the_gap_is_not_a_resume():
    op, _ = flying()
    op.on_control(False)
    assert op.set_route([(0.5, 0.5)]) == ""
    op.on_control(False)
    op.on_control(True)
    assert not resumed(op)


def test_loss_in_hold_is_a_hand_over():
    op, _ = flying()
    op.set_mode("HOLD", "console")
    op.handover = "PATROL"
    op.on_control(False)
    op.on_control(True)
    assert op.mode == "PATROL" and not resumed(op)


def test_second_loss_resumes_again():
    op, _ = flying()
    for _ in range(2):
        op.on_control(False)
        op.on_control(True)
    assert op.mode == "GOTO" and sum("route resumed" in e for e in op.events) == 2


class Planner:
    cfg = {"replan_s": 1.0}

    def result(self):
        return None

    def request(self, *a, **k):
        pass


def test_planned_route_keeps_its_waypoint():
    d, p = Drone(), Pilot()
    op = Operator({"route_resume_s": 1.0}, 2.7, p, d)
    op.planner = Planner()
    op.events = []
    op.on_event = lambda src, text: op.events.append(text)
    op.on_control(True)
    assert op.set_route(PTS) == ""
    op.on_control(True)
    op.route_i, op.plan_path, op.plan_goal = 1, [(0.5, 0.5)], (9.0, 9.0)
    op.on_control(False)
    op.on_control(True)
    assert op.mode == "GOTO" and resumed(op) and op.route == PTS and op.route_i == 1
    assert op.plan_path == [] and op.plan_goal is None


if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    bad = 0
    for name, fn in tests:
        try:
            fn()
            print("PASS", name)
        except Exception as e:
            bad += 1
            print("FAIL", name, type(e).__name__, e)
    print(f"{len(tests) - bad}/{len(tests)} passed")
    sys.exit(1 if bad else 0)
