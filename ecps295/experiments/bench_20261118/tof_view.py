#!/usr/bin/env python3
"""ECPS295 bench tool: live 8x8 view of the front and rear VL53L7CX in a browser (orientation check, S3.4 / S3.7).

Runs on the Orange Pi. The companion must be stopped first (it owns the sensors):

    systemctl stop ecps-companion
    /opt/ecps295/venv/bin/python tof_view.py            # serves http://127.0.0.1:8765 on the Pi
    ssh -N -L 8765:127.0.0.1:8765 root@<pi>             # on the Mac, then open http://localhost:8765

The sensors are read RAW (flip_lr / flip_ud / transpose all off). The page applies the three flags itself, in the
driver's order (transpose, then flip_ud, then flip_lr), starting from the values in companion.yaml, so toggling them
shows at once what the companion would see. Nothing is written to the config.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

sys.path.insert(0, "/opt/ecps295/companion")
import tof_vl53l7cx as T  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--hw", default="/opt/ecps295/config/companion.yaml")
ap.add_argument("--port", type=int, default=8765)
ap.add_argument("--bind", default="127.0.0.1")
a = ap.parse_args()

FLAGS = ("flip_lr", "flip_ud", "transpose")
sensors, flags = {}, {}
for name, c in yaml.safe_load(open(a.hw))["tof"]["sensors"].items():
    c = dict(c)
    c.pop("bearing", None)
    flags[name] = {k: bool(c.pop(k, False)) for k in FLAGS}
    sensors[name] = T.ToFSensor(name, **c).start()  # raw order: no flags passed

rate = {n: [0, time.monotonic(), 0.0] for n in sensors}  # frames at last sample, time, hz
lock = threading.Lock()


def snapshot() -> dict:
    out = {}
    now = time.monotonic()
    with lock:
        for n, s in sensors.items():
            r = rate[n]
            if now - r[1] >= 1.0:
                r[2] = (s.frames - r[0]) / (now - r[1])
                r[0], r[1] = s.frames, now
            f = s.frame
            out[n] = {
                "f": None if f is None else [None if not math.isfinite(v) else round(float(v), 3) for v in f],
                "hz": round(r[2], 1), "errors": s.errors, "temp": s.temp_c, "status": str(s.status),
                "age": round(s.age_s(), 2) if f is not None else None, "bus": s.bus,
            }
    return out


PAGE = r"""<!doctype html><html lang="zh"><head><meta charset="utf-8"><title>ToF 雷达测试</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{margin:0;padding:16px;background:#0e1116;color:#e6e9ee;font:15px -apple-system,BlinkMacSystemFont,sans-serif}
h1{font-size:18px;margin:0 0 4px}.sub{color:#9aa4b2;margin-bottom:14px;font-size:13px}
.wrap{display:flex;gap:28px;flex-wrap:wrap}.card{background:#161b22;border:1px solid #2a323d;border-radius:10px;padding:14px}
.card h2{font-size:17px;margin:0 0 2px}.stat{color:#9aa4b2;font-size:13px;margin-bottom:8px;font-variant-numeric:tabular-nums}
.frame{display:grid;grid-template-columns:28px auto 28px;grid-template-rows:26px auto 26px;align-items:center;justify-items:center}
.lab{color:#9aa4b2;font-size:14px;font-weight:600}
.grid{display:grid;grid-template-columns:repeat(8,58px);grid-template-rows:repeat(8,58px);gap:3px}
.c{display:flex;align-items:center;justify-content:center;border-radius:5px;font-size:15px;font-weight:600;
   font-variant-numeric:tabular-nums;background:#1d232c;color:#566070}
.btns{margin-top:10px;display:flex;gap:8px;flex-wrap:wrap}
button{background:#1d232c;color:#e6e9ee;border:1px solid #3a4452;border-radius:6px;padding:7px 12px;font-size:14px;cursor:pointer}
button.on{background:#1f6feb;border-color:#1f6feb}
pre{background:#0e1116;border:1px solid #2a323d;border-radius:6px;padding:8px 10px;margin:10px 0 0;font-size:13px}
.near{margin-top:8px;font-size:14px}.near b{font-size:17px}
.legend{display:flex;align-items:center;gap:8px;margin:14px 0 0;color:#9aa4b2;font-size:13px}
.bar{width:260px;height:12px;border-radius:6px;background:linear-gradient(90deg,hsl(0,85%,48%),hsl(60,85%,48%),hsl(120,85%,40%),hsl(220,85%,50%))}
.bad{color:#ff7b72}
</style></head><body>
<h1>ToF 雷达测试 · 前 / 后 VL53L7CX 8×8</h1>
<div class="sub">每块的画面 = 站在传感器后面、顺着它看的方向看到的样子（上 = 天，左 = 传感器自己的左）。后 ToF 要面朝机尾看。格内数字是米，灰色 = 没有目标。</div>
<div class="wrap" id="wrap"></div>
<div class="legend">0 m<div class="bar"></div>2 m 以上</div>
<script>
const FL=__FLAGS__, NAMES={front:"前 ToF（机头）",rear:"后 ToF（机尾）"}, MAXM=2.0;
const el=(t,c,x)=>{const e=document.createElement(t);if(c)e.className=c;if(x!=null)e.textContent=x;return e};
const ui={};
for(const n of Object.keys(FL)){
  const card=el("div","card"); card.append(el("h2","",NAMES[n]||n));
  const stat=el("div","stat","…"); card.append(stat);
  const fr=el("div","frame"), grid=el("div","grid"), cells=[];
  for(let i=0;i<64;i++){const c=el("div","c","--");cells.push(c);grid.append(c)}
  fr.append(el("div"),el("div","lab","上"),el("div"),el("div","lab","左"),grid,el("div","lab","右"),el("div"),el("div","lab","下"),el("div"));
  card.append(fr);
  const near=el("div","near"); card.append(near);
  const btns=el("div","btns"), B={};
  for(const k of ["transpose","flip_ud","flip_lr"]){
    const b=el("button","",{transpose:"transpose 转置",flip_ud:"flip_ud 上下翻",flip_lr:"flip_lr 左右翻"}[k]);
    b.onclick=()=>{FL[n][k]=!FL[n][k];paint(n)}; B[k]=b; btns.append(b)}
  card.append(btns); const pre=el("pre"); card.append(pre);
  ui[n]={stat,cells,near,B,pre,last:null}; document.getElementById("wrap").append(card); paint(n);
}
function orient(f,fl){ // same order as tof_vl53l7cx.ToFSensor._orient
  let g=[];for(let r=0;r<8;r++)g.push(f.slice(r*8,r*8+8));
  if(fl.transpose)g=g[0].map((_,c)=>g.map(row=>row[c]));
  if(fl.flip_ud)g=g.slice().reverse();
  if(fl.flip_lr)g=g.map(row=>row.slice().reverse());
  return g.flat();
}
function paint(n){
  const u=ui[n],fl=FL[n];
  for(const k in u.B)u.B[k].className=fl[k]?"on":"";
  u.pre.textContent=`flip_lr: ${fl.flip_lr}\nflip_ud: ${fl.flip_ud}\ntranspose: ${fl.transpose}`;
  if(!u.last||!u.last.f){u.near.textContent="";return}
  const g=orient(u.last.f,fl); let best=null;
  g.forEach((v,i)=>{const c=u.cells[i];
    if(v==null){c.textContent="--";c.style.background="";c.style.color="";return}
    const t=Math.min(v,MAXM)/MAXM; c.textContent=v.toFixed(2);
    c.style.background=`hsl(${Math.round(t*220)},85%,${v<0.3?48:42}%)`; c.style.color="#fff";
    if(best==null||v<g[best])best=i});
  if(best==null){u.near.textContent="视场内没有目标";return}
  const r=Math.floor(best/8),c=best%8;
  const where=(r<3?"上":r>4?"下":"中")+(c<3?"左":c>4?"右":"中");
  u.near.innerHTML=`最近 <b>${g[best].toFixed(2)} m</b>，在画面的 <b>${where}</b>（第 ${r} 行，第 ${c} 列）`;
}
async function tick(){
  try{const d=await (await fetch("/data",{cache:"no-store"})).json();
    for(const n in d){const u=ui[n];if(!u)continue;const s=d[n];u.last=s;
      const bad=s.f==null||s.age>1||s.errors>0;
      u.stat.className="stat"+(bad?" bad":"");
      u.stat.textContent=`总线 ${s.bus} · ${s.hz} Hz · 错误 ${s.errors} · ${s.temp??"-"} °C · ${s.f==null?s.status:"数据 "+s.age+" s 前"}`;
      paint(n)}
  }catch(e){for(const n in ui){ui[n].stat.className="stat bad";ui[n].stat.textContent="连接断开："+e}}
  setTimeout(tick,90);
}
tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path.startswith("/data"):
            body, ctype = json.dumps(snapshot()).encode(), "application/json"
        else:
            body, ctype = PAGE.replace("__FLAGS__", json.dumps(flags)).encode(), "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


srv = ThreadingHTTPServer((a.bind, a.port), H)
print(f"tof_view: http://{a.bind}:{a.port}  sensors {list(sensors)}  config flags {flags}", flush=True)
try:
    srv.serve_forever()
except KeyboardInterrupt:
    pass
finally:
    for s in sensors.values():
        s.stop()
