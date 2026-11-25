import csv, glob, json, os, statistics as st
rows = []
for d in sorted(glob.glob(os.path.expanduser("~/sim/runs/e8a__*"))):
    tag = os.path.basename(d)
    try:
        r = list(csv.DictReader(open(d + "/run.csv"))); j = json.load(open(d + "/run.json"))
    except Exception as e:
        print(tag, "FAILED", e); continue
    tm = next((float(x["t"]) for x in r if x["mute"] == "1"), None)
    tl = next((float(x["t"]) for x in r if x["mute"] == "1" and x["fc_mode"] == "9"), None)
    pre = [x for x in r if x["mute"] == "0"]
    post = [x for x in r if tl is not None and float(x["t"]) >= tl]
    alt0 = float(pre[-1]["alt"]) if pre else None
    drift = None
    if post:
        n0, e0 = float(post[0]["north"]), float(post[0]["east"])
        drift = max(((float(x["north"]) - n0) ** 2 + (float(x["east"]) - e0) ** 2) ** .5 for x in post)
    rows.append(dict(tag=tag, mute=tm, dt=None if tl is None or tm is None else tl - tm, alt0=alt0, end_alt=float(r[-1]["alt"]), t_end=float(r[-1]["t"]),
                     drift=drift, contact=j.get("contact_episodes"), minclr=j.get("min_clearance_m"), rtf=j.get("rtf_run") or j.get("rtf"), modes=sorted({x["fc_mode"] for x in r})))
for x in rows: print(x)
dt = [x["dt"] for x in rows if x["dt"] is not None]
print("n", len(rows), "landed", len(dt), "dt min/med/max %.2f %.2f %.2f" % (min(dt), st.median(dt), max(dt)), "contacts", sum(x["contact"] or 0 for x in rows))
print("drift med/max", st.median(x["drift"] for x in rows if x["drift"] is not None), max(x["drift"] for x in rows if x["drift"] is not None))
j = json.load(open(os.path.expanduser("~/sim/runs/" + rows[0]["tag"] + "/run.json"))); print({k: v for k, v in j.items() if not isinstance(v, (dict, list))})
