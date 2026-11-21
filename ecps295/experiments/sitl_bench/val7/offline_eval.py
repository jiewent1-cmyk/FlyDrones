#!/usr/bin/env python3
"""offline_eval.py -- 用记录的逐帧输入离线重算任意一组监视器参数。

输入：runs/<tag>/monitor.csv（逐帧原始输入：t, distance, roll, pitch, press, armed）
输出：
  fp   模式：给定参数（或整个参数网格）在若干次飞行里是否触发、触发时刻、原因
  det  模式：给定注入时刻，算出是否检出、检出延迟、判定时刻真值高度偏离

用法示例：
  offline_eval.py fp --runs runs.txt --grid --out fp.csv
  offline_eval.py fp --runs runs.txt --set fc=1.0,tau=30,E=0.5,T=1.0,R=0.6
  offline_eval.py det --run runs/v7_sq_gN_r1 --inject-t 12.3 --set fc=1.0,tau=30,E=0.5,T=1.0,R=0.6
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from baro_range_monitor import BaroRangeMonitor

GRID = {
    "fc": [0.5, 1.0],
    "tau": [10.0, 30.0],
    "E": [0.3, 0.5, 0.7],
    "T": [0.5, 1.0, 1.5],
    "R": [0.4, 0.6],
}
DEFAULT = {"fc": 1.0, "tau": 30.0, "E": 0.5, "T": 1.0, "R": 0.6}


def parse_set(s):
    p = dict(DEFAULT)
    if s:
        for kv in s.split(","):
            k, v = kv.split("=")
            p[k.strip()] = float(v)
    return p


def grid_sets():
    out = []
    for fc in GRID["fc"]:
        for tau in GRID["tau"]:
            for E in GRID["E"]:
                for T in GRID["T"]:
                    for R in GRID["R"]:
                        out.append({"fc": fc, "tau": tau, "E": E, "T": T, "R": R})
    return out


def load_frames(run_dir):
    path = os.path.join(run_dir, "monitor.csv")
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r["t"])
                d = float(r["distance"]) if r["distance"] not in ("", None) else None
                roll = float(r["roll"])
                pitch = float(r["pitch"])
                press = float(r["press"]) if r["press"] not in ("", None) else None
                armed = r.get("armed", "0") == "1"
            except (ValueError, KeyError):
                continue
            rows.append((t, d, roll, pitch, press, armed))
    return rows


def replay(rows, params):
    """重放一遍，返回逐帧状态和首次触发信息。"""
    mon = BaroRangeMonitor(**params)
    trace = []
    for (t, d, roll, pitch, press, armed) in rows:
        s = mon.step(t, d, roll, pitch, press, armed)
        trace.append(s)
    return trace, mon


def summarize(trace, mon):
    # 注意：监视器在解锁→上锁时复位自己的触发状态，所以要看轨迹里的第一次触发，
    # 不能只看 mon 的最终属性。
    trig = [s for s in trace if s["trigger"]]
    e = [s["e"] for s in trace if s["enabled"]]
    return {
        "n_frames": len(trace),
        "trigger": bool(trig),
        "t_trig": trig[0]["t"] if trig else None,
        "t_trig_A": trig[0]["t_trig_A"] if trig else None,
        "t_trig_B": trig[0]["t_trig_B"] if trig else None,
        "reason": trig[0]["reason"] if trig else None,
        "trig_A": bool(trig and trig[0]["trig_A"]),
        "trig_B": bool(trig and trig[0]["trig_B"]),
        "e_mean": round(sum(e) / len(e), 4) if e else None,
        "e_std": round(math.sqrt(sum((v - sum(e) / len(e)) ** 2 for v in e) / len(e)), 4) if e else None,
        "e_absmax": round(max(abs(v) for v in e), 4) if e else None,
    }


# ------------------------------------------------------------------ 真值高度
def truth_series(run_dir):
    """dataflash SIM2 的真值高度序列 [(t, alt_m)]，相对起飞前的地面基准。"""
    from pymavlink import mavutil
    key = os.path.abspath(run_dir)
    if key in _TRUTH:
        return _TRUTH[key]
    _TRUTH[key] = []
    ld = os.path.join(run_dir, "logs")
    if os.path.isdir(ld):
        for f in sorted(x for x in os.listdir(ld) if x.endswith(".BIN")):
            m = mavutil.mavlink_connection(os.path.join(ld, f))
            sim = []
            while True:
                x = m.recv_match(type="SIM2")
                if x is None:
                    break
                sim.append((x.TimeUS / 1e6, x.PN, x.PE, x.PD))
            if not sim:
                continue
            pd0 = max(s[3] for s in sim)
            _TRUTH[key] = [(tt, -(pd - pd0)) for tt, pn, pe, pd in sim]
            break
    return _TRUTH[key]


_TRUTH = {}


def truth_at(run_dir, t):
    """t（FC 时钟秒）处的真值高度（m）。"""
    ser = truth_series(run_dir)
    if not ser:
        return None
    best = None
    for tt, alt in ser:
        if best is None or abs(tt - t) < abs(best[0] - t):
            best = (tt, alt)
    return round(best[1], 3) if best else None


# ------------------------------------------------------------------ 任务
def job_fp(args):
    run_dir, params = args
    rows = load_frames(run_dir)
    if not rows:
        return {"run": run_dir, "error": "no monitor.csv"}
    trace, mon = replay(rows, params)
    r = summarize(trace, mon)
    r["run"] = run_dir
    mp = os.path.join(run_dir, "metrics.json")
    if os.path.exists(mp):
        try:
            m = json.load(open(mp))
            r["flight_s"] = m.get("flight_s")
            r["alt_max"] = m.get("alt_max")
        except Exception:
            pass
    return r


def job_det(args):
    run_dir, params = args
    rows = load_frames(run_dir)
    out = {"run": run_dir}
    if not rows:
        out["error"] = "no monitor.csv"
        return out
    trace, mon = replay(rows, params)
    sm = summarize(trace, mon)
    out["trigger"] = sm["trigger"]
    out["t_trig"] = sm["t_trig"]
    out["reason"] = sm["reason"]
    out["trig_A"] = sm["trig_A"]
    out["trig_B"] = sm["trig_B"]
    mp = os.path.join(run_dir, "metrics.json")
    inj = None
    if os.path.exists(mp):
        try:
            m = json.load(open(mp))
            if m.get("s0") is not None and m.get("f_t0") is not None:
                inj = m["s0"] + m["f_t0"]
            out["flight_s"] = m.get("flight_s")
            out["alt_max"] = m.get("alt_max")
        except Exception:
            pass
    out["inject_t"] = inj
    if out["t_trig"] is not None and inj is not None:
        out["delay"] = round(out["t_trig"] - inj, 2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["fp", "det", "detgrid"])
    ap.add_argument("--runs", help="每行一个 run 目录的列表文件")
    ap.add_argument("--run", help="单个 run 目录")
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--set", default="")
    ap.add_argument("--out")
    ap.add_argument("--inject-t", type=float, default=None)
    ap.add_argument("--jobs", type=int, default=8)
    a = ap.parse_args()

    runs = []
    if a.runs:
        for line in open(a.runs):
            line = line.strip()
            if line:
                if os.path.isabs(line):
                    runs.append(line)
                elif line.startswith("runs/"):
                    runs.append(os.path.join("/home/jacky/sim/val7", line))
                else:
                    runs.append(os.path.join("/home/jacky/sim/val7/runs", line))
    elif a.run:
        runs.append(a.run)

    if a.mode == "det":
        params = parse_set(a.set)
        rows = load_frames(a.run)
        trace, mon = replay(rows, params)
        r = summarize(trace, mon)
        if a.inject_t is not None and r["t_trig"] is not None:
            r["inject_t"] = a.inject_t
            r["delay_s"] = round(r["t_trig"] - a.inject_t, 3)
            r["truth_at_trig"] = truth_at(a.run, r["t_trig"])
            r["truth_at_inject"] = truth_at(a.run, a.inject_t)
            if r["truth_at_trig"] is not None and r["truth_at_inject"] is not None:
                r["truth_dev_at_trig"] = round(r["truth_at_trig"] - 2.0, 3)
        print(json.dumps(r, indent=1))
        return

    sets = grid_sets() if a.grid else [parse_set(a.set)]

    if a.mode == "detgrid":
        tasks = [(r, p) for p in sets for r in runs]
        with Pool(a.jobs) as pool:
            res = pool.map(job_det, tasks)
        rowsout = []
        i = 0
        for p in sets:
            chunk = res[i:i + len(runs)]
            i += len(runs)
            det = [c for c in chunk if c.get("trigger")]
            dl = [c["delay"] for c in det if c.get("delay") is not None]
            rowsout.append({"params": json.dumps(p, sort_keys=True), "n_runs": len(runs), "n_det": len(det),
                            "delay_med": round(sorted(dl)[len(dl) // 2], 2) if dl else None,
                            "delay_max": round(max(dl), 2) if dl else None,
                            "per_cell": json.dumps({os.path.basename(c["run"]): c.get("delay")
                                                    for c in chunk})})
            print("%s -> 检出 %d/%d  延迟中位 %s 最大 %s" %
                  (json.dumps(p, sort_keys=True), len(det), len(runs), rowsout[-1]["delay_med"],
                   rowsout[-1]["delay_max"]))
        if a.out:
            with open(a.out, "w") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rowsout[0].keys()))
                w.writeheader()
                for r in rowsout:
                    w.writerow(r)
            print("written", a.out)
        return

    # --- fp ---
    tasks = [(r, p) for p in sets for r in runs]
    with Pool(a.jobs) as pool:
        res = pool.map(job_fp, tasks)
    rowsout = []
    i = 0
    for p in sets:
        chunk = res[i:i + len(runs)]
        i += len(runs)
        n_trig = sum(1 for c in chunk if c.get("trigger"))
        n_a = sum(1 for c in chunk if c.get("trig_A"))
        n_b = sum(1 for c in chunk if c.get("trig_B"))
        n_ab = sum(1 for c in chunk if c.get("trig_A") and c.get("trig_B"))
        errs = [c for c in chunk if c.get("error")]
        fs = [c.get("flight_s") or 0 for c in chunk]
        hours = sum(fs) / 3600.0
        rowsout.append({"params": json.dumps(p, sort_keys=True), "n_runs": len(runs), "n_trigger": n_trig,
                        "n_trig_A": n_a, "n_trig_B": n_b, "n_trig_AB": n_ab,
                        "n_error": len(errs), "flight_h": round(hours, 3),
                        "fp_per_h": round(n_trig / hours, 2) if hours else None,
                        "runs": json.dumps([{"run": os.path.basename(c["run"]), "t_trig": c.get("t_trig"),
                                             "reason": c.get("reason")} for c in chunk])})
        print("%s -> %d/%d 触发 (%.2f 飞行小时, %.2f 次/小时)" %
              (json.dumps(p, sort_keys=True), n_trig, len(runs), hours, (n_trig / hours) if hours else 0))
    if a.out:
        with open(a.out, "w") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rowsout[0].keys()))
            w.writeheader()
            for r in rowsout:
                w.writerow(r)
        print("written", a.out)


if __name__ == "__main__":
    main()
