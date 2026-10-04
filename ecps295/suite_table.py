"""Markdown table of run_g4.py summaries for the G4 flight-test suite."""

import glob
import json
import os
import re
import sys

cols = [
    ("test", None),
    ("world", None),
    ("contact", "contact"),
    ("min clr m", "min_clearance_m"),
    ("brake@ m", "brake_onset_clearance_m"),
    ("escape@ m", "escape_onset_clearance_m"),
    ("final clr m", "final_clearance_m"),
    ("alt min-max m", None),
    ("escapes", "escapes"),
    ("safety", "safety_events"),
    ("RTF median", None),
    ("tick p95 ms", None),
]
print("| " + " | ".join(c for c, _ in cols) + " |")
print("|" + "---|" * len(cols))
for d in sorted(sys.argv[1:]):
    m = re.match(r"suite_(?:(.+?)_)?(T\d.*)$", os.path.basename(d))  # suite_<variant>_<test> or suite_<test>
    test = m.group(2) if m else os.path.basename(d)
    js = glob.glob(os.path.join(d, f"{test}.json"))
    if not js:
        print(f"| {test} | (no result) |")
        continue
    j = json.load(open(js[0]))
    rtf = (
        (open(os.path.join(d, "rtf.log")).read().split("median") + ["? (run ended before sampling)"])[1].split()[0]
        if os.path.exists(os.path.join(d, "rtf.log"))
        else "?"
    )
    world = open(os.path.join(d, "gz.log")).read().split("world/", 1)[-1].split("/", 1)[0] if False else ""
    vals = []
    for name, key in cols:
        if name == "test":
            vals.append(test)
        elif name == "world":
            vals.append(j.get("world", ""))
        elif name == "alt min-max m":
            vals.append(f"{j['alt_min']:.2f}-{j['alt_max']:.2f}")
        elif name == "RTF median":
            vals.append(rtf)
        elif name == "tick p95 ms":
            vals.append(str(j["tick_interval_ms_p50_p95_max"][1]))
        else:
            v = j.get(key)
            if key == "safety_events":
                from collections import Counter

                v = ", ".join(f"{k}x{n}" if n > 1 else k for k, n in Counter(v).items()) or "-"
            vals.append("yes" if v is True else "no" if v is False else ("-" if v is None else str(v)))
    print("| " + " | ".join(vals) + " |")
