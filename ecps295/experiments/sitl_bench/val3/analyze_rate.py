#!/usr/bin/env python3
"""Compare feeder send rate with the rate the FC logged (dataflash OF)."""
import json
import os
import sys

for tag in sys.argv[1:]:
    rd = os.path.join('runs', tag)
    feed = json.load(open(os.path.join(rd, 'feed.json')))
    of = json.load(open(os.path.join(rd, 'of.json')))
    ts = feed['sent_ts']['flow']
    span = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    sent_hz = (len(ts) - 1) / span if span > 0 else -1
    dspan = of.get('of_span_s') or 0.0
    rcv_hz = (of.get('of_count', 0) - 1) / dspan if dspan > 0 else -1
    print('%-12s sent=%6d span=%6.2f sent_hz=%7.2f | OF=%6d span=%6.2f fc_hz=%7.2f qual=%s flowX=%s flowY=%s' % (
        tag, len(ts), span, sent_hz, of.get('of_count', 0), dspan, rcv_hz,
        of.get('of_qual_mean'), of.get('of_flowX_mean'), of.get('of_flowY_mean')))
    per_s = dict(of.get('of_per_s') or [])
    vals = list(per_s.values())
    if vals:
        print('   OF per-s min=%d max=%d mean=%.1f n_sec=%d' % (
            min(vals), max(vals), sum(vals) / len(vals), len(vals)))
