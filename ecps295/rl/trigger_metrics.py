"""Trigger-level metrics (RL roadmap B1). Definitions fixed on 2026-10-06 before any result was seen.

Inputs are per-tick arrays at the 20 Hz control rate: time t [s], position north/east [m] (truth), clearance [m] (prop tip
to nearest obstacle reaching the drone's height, as run_g4.py), escape flag (decoder saccade / escape active).

  threat tick      clearance < 0.6 m, approaching faster than 0.05 m/s, and TTC = clearance / approach < 2.0 s
  threat episode   threat ticks merged when gaps are < 0.5 s
  hit              an escape onset inside [episode start - 0.25 s, episode end]; latency = onset - episode start
  false alarm      an escape onset with no threat tick within +-1.0 s and clearance > 0.8 m
  chain trigger    an escape onset less than 2.0 s after the previous escape ended
  slow fraction    share of ticks with ground speed < 0.05 m/s (speed from 0.25 s differences)
Works on the fast twin (episode rows) and on Gazebo run.csv (columns t, north, east, clearance, escape).
"""

from __future__ import annotations

import csv

import numpy as np

CLR, APPROACH, TTC, MERGE, PRE, FA_WIN, FA_CLR, CHAIN, SLOW = 0.6, 0.05, 2.0, 0.5, 0.25, 1.0, 0.8, 2.0, 0.05


def trigger_metrics(t, north, east, clearance, escape) -> dict:
    t, n, e = (np.asarray(x, float) for x in (t, north, east))
    clr = np.asarray(clearance, float)
    esc = np.asarray(escape, bool)
    if len(t) < 10:
        return {}
    dt = float(np.median(np.diff(t)))
    lag = max(1, int(round(0.25 / dt)))
    approach = np.zeros_like(clr)
    approach[lag:] = -(clr[lag:] - clr[:-lag]) / (t[lag:] - t[:-lag])
    finite = np.isfinite(clr)
    with np.errstate(divide="ignore", invalid="ignore"):
        ttc = np.where(approach > APPROACH, clr / approach, np.inf)
    threat = finite & (clr < CLR) & (approach > APPROACH) & (ttc < TTC)

    # threat episodes
    eps, start, last = [], None, None
    for i in np.flatnonzero(threat):
        if start is None:
            start = last = i
        elif t[i] - t[last] <= MERGE:
            last = i
        else:
            eps.append((start, last))
            start = last = i
    if start is not None:
        eps.append((start, last))

    onsets = np.flatnonzero(esc[1:] & ~esc[:-1]) + 1
    if esc[0]:
        onsets = np.r_[0, onsets]
    ends = np.flatnonzero(~esc[1:] & esc[:-1]) + 1
    hits, lat = 0, []
    for s, f in eps:
        cand = [o for o in onsets if t[s] - PRE <= t[o] <= t[f]]
        if cand:
            hits += 1
            lat.append(max(0.0, t[cand[0]] - t[s]))
    fa = 0
    for o in onsets:
        win = (t >= t[o] - FA_WIN) & (t <= t[o] + FA_WIN)
        if not threat[win].any() and finite[o] and clr[o] > FA_CLR:
            fa += 1
    chain = 0
    for o in onsets:
        prev_end = ends[ends < o]
        if prev_end.size and t[o] - t[prev_end[-1]] < CHAIN:
            chain += 1
    sp = np.zeros_like(t)
    sp[lag:] = np.hypot(n[lag:] - n[:-lag], e[lag:] - e[:-lag]) / (t[lag:] - t[:-lag])
    minutes = (t[-1] - t[0]) / 60.0
    return {
        "threat_episodes": len(eps),
        "saccade_hits": hits,
        "saccade_recall": round(hits / len(eps), 3) if eps else None,
        "saccade_latency_s": round(float(np.mean(lat)), 3) if lat else None,
        "saccade_onsets": int(onsets.size),
        "false_alarms": fa,
        "false_alarms_per_min": round(fa / minutes, 3) if minutes > 0 else None,
        "chain_triggers": chain,
        "chain_rate": round(chain / onsets.size, 3) if onsets.size else None,
        "slow_fraction": round(float((sp[lag:] < SLOW).mean()), 3),
        "mean_speed": round(float(sp[lag:].mean()), 3),
    }


def from_run_csv(path: str) -> dict:
    rows = list(csv.DictReader(open(path)))
    f = lambda k: [float(r[k]) for r in rows]  # noqa: E731
    return trigger_metrics(f("t"), f("north"), f("east"), f("clearance"), [r["escape"] == "1" for r in rows])
