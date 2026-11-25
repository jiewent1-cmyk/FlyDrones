from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent; OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "all_runs.csv")

# Per-arm outcomes from run-level classification in all_runs.csv.
order=["FULL","NOGUARD","BARE","STOCKGATE","SURF1","FULL_NOALTF","BARE_NOALTF","G_LANDONLY","G_RATE","G_E05","G_E10","BAROPRI"]
fig, ax = plt.subplots(figsize=(10,4.5))
bottom=[0.0]*len(order)
for key,color,label in [("OK","#009E73","OK"),("UNSAFE","#D55E00","Unsafe"),("DEGRADED",OKABE[1],"Degraded"),("ABORT",OKABE[0],"Abort"),("ARMED_GND",OKABE[4],"Armed on ground")]:
    vals=[]
    for arm in order:
        rows=df[df.arm==arm]
        vals.append((rows.cls2==key).sum()/len(rows)*100 if len(rows) else 0)
    ax.bar(range(len(order)), vals, label=label, color=color, bottom=bottom)
    bottom=[a+b for a,b in zip(bottom,vals)]
ax.set(xticks=range(len(order)),xticklabels=order,ylabel="Runs in outcome class (%)",title="Outcome mix by controller arm")
ax.tick_params(axis="x",rotation=55); ax.legend(ncol=4,fontsize=8)
ax.legend(fontsize=8, loc="center left", bbox_to_anchor=(1.01, 0.5))
fig.tight_layout(); fig.savefig(OUT / "fig1_arm_outcomes.png"); plt.close(fig)

# Summarize actual unsafe fraction among fault cases, retaining per-arm sample counts.
unsafe=[]; ns=[]
for arm in order:
    rows=df[df.arm==arm]; ns.append(len(rows)); unsafe.append((rows.cls2=="UNSAFE").mean()*100 if len(rows) else float("nan"))
fig, ax=plt.subplots(figsize=(9,4.5))
ax.bar(range(len(order)),unsafe,color="#D55E00")
ax.set(xticks=range(len(order)),xticklabels=order,ylabel="Unsafe runs (%)",title="Unsafe outcome rate across controller arms",ylim=(0,100))
ax.tick_params(axis="x",rotation=55)
for i,(v,n) in enumerate(zip(unsafe,ns)): ax.text(i,v+1,f"n={n}",ha="center",fontsize=7,rotation=90)
fig.tight_layout(); fig.savefig(OUT / "fig2_unsafe_rate.png"); plt.close(fig)

# Spread of maximum altitude over repeats for the three primary arms.
fig, ax=plt.subplots(figsize=(7,4))
for i,arm in enumerate(["FULL","NOGUARD","BARE"]):
    vals=pd.to_numeric(df.loc[df.arm==arm,"alt_max"],errors="coerce").dropna()
    ax.scatter([i]*len(vals),vals,s=18,alpha=.6,color=OKABE[i])
    if len(vals): ax.plot([i-.18,i+.18],[vals.median()]*2,color="black",lw=2)
ax.set(xticks=range(3),xticklabels=["FULL","NOGUARD","BARE"],ylabel="Maximum altitude (m)",title="Altitude excursions in primary arm comparison")
fig.tight_layout(); fig.savefig(OUT / "fig3_altitude_spread.png"); plt.close(fig)
