"""Where does the lateral excursion happen: before the guard trip or during the LAND descent after it?"""
import glob, json, math, re, sys, os
from multiprocessing import Pool
from pymavlink import mavutil
HARD = ["r_gN","r_h","r_k","r_g5","x_gN_noise","x_h_drn","x_h_drp","x_k_st2n","r_oor3","r_c","r_j2"]
def one(rd):
    tag=os.path.basename(rd); met=json.load(open(rd+"/metrics.json")); psi=math.radians(met.get("true_yaw0_deg") or 0)
    hg=open(rd+"/hg.log").read() if os.path.exists(rd+"/hg.log") else ""
    arm=re.search(r"^\s*([\d.]+) FC ARMED",hg,re.M); trip=re.search(r"^\s*([\d.]+) EVENT HEIGHT GUARD",hg,re.M)
    dt=float(trip.group(1))-float(arm.group(1)) if arm and trip else None
    m=mavutil.mavlink_connection(sorted(glob.glob(rd+"/logs/*.BIN"))[-1])
    sim=[]; tarm=None; land=None; ekfD=[]
    while True:
        x=m.recv_match(type=["SIM2","MSG","MODE","XKF1"])
        if x is None: break
        ty=x.get_type(); t=x.TimeUS*1e-6
        if ty=="MSG":
            if "Arming motors" in x.Message and tarm is None: tarm=t
        elif ty=="MODE":
            if x.ModeNum==9 and land is None: land=t
        elif ty=="XKF1":
            if x.C==0: ekfD.append((t,x.PN,x.PE))
        else: sim.append((t,x.PN,x.PE,x.PD,x.VN,x.VE))
    if not sim or tarm is None: return None
    n0,e0,d0=sim[0][1:4]
    def lat(n,e):
        dn,de=n-n0,e-e0; xx=dn*math.cos(psi)+de*math.sin(psi); yy=-dn*math.sin(psi)+de*math.cos(psi)
        return max(-xx,xx-2,-yy,yy-2)
    tt=tarm+dt if dt is not None else land
    air=[s for s in sim if -(s[3]-d0)>0.10]
    if not air: return None
    pre=[lat(s[1],s[2]) for s in air if tt is None or s[0]<=tt]
    post=[(s,lat(s[1],s[2])) for s in air if tt is not None and s[0]>tt]
    out=dict(tag=tag,trip_dt=dt,land_t=(land-tarm) if land else None,lat_pre=max(pre) if pre else None,
             lat_post=max(p[1] for p in post) if post else None, desc_s=(post[-1][0][0]-tt) if post else None,
             vmax_post=max(math.hypot(p[0][4],p[0][5]) for p in post) if post else None,
             alt_at=-(min(sim,key=lambda s:abs(s[0]-tt))[3]-d0) if tt else None)
    return out
if __name__=="__main__":
    arms=sys.argv[1].split(",")
    rds=[rd for a in arms for f in HARD for rd in sorted(glob.glob("runs/ab_%s__%s__r*"%(a,f)))]
    with Pool(12) as p: res=[r for r in p.map(one,rds) if r]
    json.dump(res,open("lat_timing.json","w"))
    import pandas as pd
    d=pd.DataFrame(res); d["arm"]=d.tag.str.extract(r"ab_(.+?)__"); d["fault"]=d.tag.str.extract(r"__(.+?)__r")
    pd.set_option("display.width",250)
    print(d.groupby(["arm","fault"]).agg(n=("tag","size"),lat_pre_med=("lat_pre","median"),lat_pre_max=("lat_pre","max"),lat_post_med=("lat_post","median"),lat_post_max=("lat_post","max"),desc_s=("desc_s","median"),v_post_med=("vmax_post","median"),v_post_max=("vmax_post","max"),alt_at=("alt_at","median")).round(2))
