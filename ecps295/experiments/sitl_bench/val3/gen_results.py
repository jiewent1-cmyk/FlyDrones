#!/usr/bin/env python3
"""Build results.csv from the run directories (one row per run; only measured values)."""
import csv
import json
import os

BASE = '/home/jacky/sim/val3/runs'
COLS = ['run', 'part', 'instance', 'port', 'sysid', 'overlay', 'feeder_args', 'sent_dist_msgs', 'sent_flow_msgs',
        'fc_distance_sensor_rx', 'fc_optical_flow_rx', 'fc_rangefinder_rx', 'dist_sent_m', 'dist_fc_cm',
        'flow_sent_rate_xy', 'flow_fc_rate_xy', 'flow_fc_legacy_xy', 'quality_sent', 'quality_fc',
        'dist_stop_at_s', 'dist_silence_s', 'flow_stop_at_s', 'flow_silence_s',
        'of_count', 'of_hz', 'of_qual', 'of_flowX_mean', 'of_flowY_mean', 'of_bodyX_mean', 'of_bodyY_mean',
        'mission', 'armed', 'flew', 'mission_complete', 'reached', 'alt_max_m', 'radius_max_m',
        'hgt_est_rms_m', 'hgt_est_maxabs_m', 'pos_est_err_max_m', 'land_offset_m', 'auto_seconds',
        'truth_rows', 'status', 'note']


def g(d, k, default=''):
    return d.get(k, default) if d else default


rows = []
for tag in sorted(os.listdir(BASE)):
    p = os.path.join(BASE, tag)
    if not os.path.isdir(p):
        continue
    cmd = {}
    c = os.path.join(p, 'cmd.txt')
    if os.path.exists(c):
        for tok in open(c, errors='ignore').read().split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                cmd[k] = v
    args = cmd.get('args', '')
    part = 'part2' if tag.startswith('p2_') else 'part1'
    ovl = ''
    o = os.path.join(p, 'overlay.parm')
    if os.path.exists(o):
        ovl = ','.join(x.split()[0] + '=' + x.split()[1] for x in open(o).read().split('\n') if x.strip())
    a = {}
    aj = os.path.join(p, 'analysis.json')
    if os.path.exists(aj):
        a = json.load(open(aj))
    f = {}
    fj = os.path.join(p, 'of.json')
    if os.path.exists(fj):
        try:
            f = json.load(open(fj))
        except Exception:
            f = {}
    m = {}
    mj = os.path.join(p, 'metrics.json')
    if os.path.exists(mj):
        try:
            m = json.load(open(mj))
        except Exception:
            m = {}
    s = m.get('script') or {}
    auto = s.get('auto') or {}
    rx = g(a, 'rx_counts', {}) or {}
    dp = g(a, 'dist_phases', []) or []
    fp = g(a, 'flow_phases', []) or []
    qp = g(a, 'quality_phases', []) or []
    dist_sent = '/'.join(str(x) for ph in dp for x in ph.get('sent_m', []))
    dist_fc = '/'.join(str(x) for ph in dp for x in ph.get('fc_cm', []))
    flow_sent = '/'.join(','.join(str(v) for v in ph.get('sent_x_y', [])) for ph in fp)
    flow_fc = '/'.join(','.join(str(v) for v in (ph.get('fc_rate_x'), ph.get('fc_rate_y'))) for ph in fp)
    flow_leg = '/'.join(','.join(str(v) for v in (ph.get('fc_legacy_x'), ph.get('fc_legacy_y'))) for ph in fp)
    q_sent = '/'.join(str(x) for ph in qp for x in ph.get('sent_q', []))
    q_fc = '/'.join(str(x) for ph in qp for x in ph.get('fc_q', []))
    st = 'no feed.log'
    note = ''
    fl = os.path.join(p, 'feed.log')
    rl = os.path.join(p, 'run.log')
    if os.path.exists(fl):
        txt = open(fl, errors='ignore').read()
        if 'feeder exit 0' in txt:
            st = 'ok'
        elif 'feeder exit 1' in txt:
            st = 'feeder error'
        else:
            st = 'unknown'
        if st == 'feeder error':
            note = 'pymavlink rc_channels_override_send 缺 chan7_raw/chan8_raw'
    if os.path.exists(rl):
        txt = open(rl, errors='ignore').read()
        if 'script exit 0' in txt:
            st = 'ok'
        elif 'script exit' in txt:
            st = 'script error'
        if 'Traceback' in txt:
            st = 'script error'
            errs = [ln for ln in txt.split('\n') if 'Error' in ln]
            note = note or (errs[-1][:120] if errs else 'Traceback in run.log')
    truth_rows = ''
    tf = os.path.join(p, 'truth.csv')
    if os.path.exists(tf):
        truth_rows = sum(1 for _ in open(tf)) - 1
    mission = ''
    if args:
        for mm in ('HOV', 'SQ'):
            if mm in args.split():
                mission = mm
    rows.append([
        tag, part, cmd.get('inst', ''), cmd.get('port', ''), g(a, 'sysid', ''), ovl, args,
        g(a, 'sent', {}).get('dist', s.get('sent', {}).get('dist', '')),
        g(a, 'sent', {}).get('flow', s.get('sent', {}).get('flow', '')),
        rx.get('DISTANCE_SENSOR', 0), rx.get('OPTICAL_FLOW', 0), rx.get('RANGEFINDER', 0),
        dist_sent, dist_fc, flow_sent, flow_fc, flow_leg, q_sent, q_fc,
        g(a, 'dist_stop_at', ''), g(a, 'dist_silence_after_s', ''), g(a, 'flow_stop_at', ''), g(a, 'flow_silence_after_s', ''),
        g(f, 'of_count', ''), g(f, 'of_hz', ''),
        ('%s-%s' % (g(f, 'of_qual_min', ''), g(f, 'of_qual_max', ''))) if f else '',
        g(f, 'of_flowX_mean', ''), g(f, 'of_flowY_mean', ''), g(f, 'of_bodyX_mean', ''), g(f, 'of_bodyY_mean', ''),
        mission, s.get('auto_armed', ''), auto.get('flew', ''), m.get('mission_complete', ''),
        g(m, 'reached', ''), g(m, 'alt_max', ''), g(m, 'radius_max', ''),
        g(m, 'hgt_est_rms', ''), g(m, 'hgt_est_maxabs', ''), g(m, 'pos_est_err_max', ''),
        g(m, 'land_offset', ''), g(m, 'auto_seconds', ''),
        truth_rows, st, note])

with open('/home/jacky/sim/val3/results.csv', 'w', newline='') as fh:
    w = csv.writer(fh)
    w.writerow(COLS)
    w.writerows(rows)
print('rows:', len(rows))
