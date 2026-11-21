import cv2, glob, os, sys, numpy as np, statistics as st
from rl.episode import build_cfg
from ecps_retina import EcpsRetina
from rl.twin import World, TwinCamera
cfg = build_cfg("minifly/v3_2.yaml", 0)
def fracs(frames):
    ret = EcpsRetina.from_config(cfg); out=[]
    for f in frames:
        ret.encode(f); lb = ret.last_blank
        if "frac_L" in lb: out.append(max(lb["frac_L"], lb["frac_R"]))
    return out
fs = sorted(glob.glob("/home/jacky/sim/runs/rec_cage/frames/*.jpg"))[:120]
gz = fracs([cv2.imread(f, cv2.IMREAD_GRAYSCALE) for f in fs])
w = World.from_layout("gazebo/worlds/ecps295_cage20.layout.json")
print(f"gazebo hover        median {st.median(gz):.2f} mean {np.mean(gz):.2f}")
for c, l in [("0","0"),("0.12","0.25"),("0.25","0.25"),("0.4","0.4")]:
    os.environ["TWIN_CLOUDS"], os.environ["TWIN_LAMPS"] = c, l
    cam = TwinCamera()
    fr = fracs([cam.render(w, np.array([0.14, 0, 0.6]), 0.0) for _ in range(60)])
    rng = np.random.default_rng(0); fr2 = []
    for k in range(80):
        r, a, yaw = rng.uniform(0,1.8), rng.uniform(0,6.28), rng.uniform(0,6.28)
        fr2 += fracs([cam.render(w, np.array([r*np.cos(a), r*np.sin(a), 0.6]), yaw)]*2)
    print(f"twin clouds={c} lamps={l}: same pose median {st.median(fr):.2f} | random poses median {st.median(fr2):.2f} share>=.55 {np.mean(np.array(fr2)>=.55):.2f}")
