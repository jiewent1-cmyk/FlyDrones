#!/usr/bin/env python3
"""Gazebo matrices for the final-presentation experiments (2026-11-21). Line: <tag> <world> <variant> <seed> <run_g4 args>.
e6  sensor configuration: front ToF + map memory (s7_w1map, what the drone flies now) vs front + rear ToF (s7_w2map),
    paired by world and seed, patrol + marker approach.
e7  landmark position fix off / on, 4 min hold in front of the marker (drift over time).
e8a companion hangs (--mute-at): time until the FC's GCS failsafe lands."""
W = ["cage10", "cage20"] + ["lowbox_s%d" % i for i in range(12)]
P = "--mode patrol --seconds %d --cruise 0.5 --max-forward 0.5 --fence-turn --ext-height"
T = " --track @E@/gazebo/worlds/ecps295_%s_mk.marker.json --track-at 6 --track-hold %d"
e6 = [f"e6__{w}_k{k}__{c} {w}_mk s7_{c}@intact {k} " + P % 130 + T % (w, 60) for k in (0, 1, 2) for w in W for c in ("w2map", "w1map")]
e7 = [f"e7__{w}_k{k}_{o} {w}_mk s7_w2map@intact {k} " + P % 300 + T % (w, 240) + (" --landmark {}" if o == "on" else "")
      for k in (0, 1) for w in W for o in ("off", "on")]
e8 = [f"e8a__{w}_k{k}_t{t} {w}_mk s7_w2map@intact {k} " + P % 120 + f" --mute-at {t}"
      for w, k in [("cage20", i) for i in range(5)] + [("lowbox_s%d" % i, 0) for i in range(5)] for t in (30, 60, 90)]
for n, m in (("e6", e6), ("e7", e7), ("e8a", e8)):
    open(f"{n}_matrix.txt", "w").write("\n".join(m) + "\n")
    print(n, len(m))
