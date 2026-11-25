from pathlib import Path
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OKABE = ["#0072B2", "#E69F00", "#009E73", "#D55E00", "#CC79A7", "#56B4E9", "#F0E442", "#000000"]
plt.rcParams.update({"figure.dpi": 150, "savefig.dpi": 150, "savefig.bbox": "tight", "font.size": 10, "axes.titlesize": 11, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.3, "axes.prop_cycle": matplotlib.cycler(color=OKABE), "figure.facecolor": "white"})
HERE = Path(__file__).resolve().parent; OUT = HERE / "figures"; OUT.mkdir(exist_ok=True)
df = pd.read_csv(HERE / "results.csv")

# Dataflash optical-flow rates from existing result rows.
hz = df[df.run.astype(str).str.match(r"p1_hz(10|20|50|100)$")].copy()
hz["target_hz"] = hz.run.str.extract(r"hz(\d+)").astype(float)
hz = hz.sort_values("target_hz")
hz["measured_hz"] = pd.to_numeric(hz.of_hz, errors="coerce")
fig, ax = plt.subplots(figsize=(7, 4))
ax.scatter(hz.target_hz, hz.measured_hz, s=50)
ax.plot([0,105],[0,105], color="#000000", lw=1, ls="--", label="1:1")
ax.set(xlabel="Configured send rate (Hz)", ylabel="Measured dataflash OF rate (Hz)", title="Optical-flow rate follows the configured rate", xlim=(0,105), ylim=(0,105)); ax.legend()
fig.tight_layout(); fig.savefig(OUT / "fig1_flow_rate.png"); plt.close(fig)

# Transmit/receive counts for the named diagnostics; available counts are direct results.csv fields.
sel = df[df.run.isin(["p1_main200", "p1_main1", "p1_arm_of"])].copy()
sel["flow_rx"] = pd.to_numeric(sel.fc_optical_flow_rx, errors="coerce")
sel["range_rx"] = pd.to_numeric(sel.fc_rangefinder_rx, errors="coerce")
fig, ax = plt.subplots(figsize=(7,4))
xs = range(len(sel)); width=.36
ax.bar([x-width/2 for x in xs], sel.flow_rx, width, label="Optical flow received", color=OKABE[0])
ax.bar([x+width/2 for x in xs], sel.range_rx, width, label="RANGEFINDER message received", color=OKABE[1])
for x, f, r in zip(xs, sel.flow_rx, sel.range_rx):  # print every value, so a zero is visible as a result
    ax.text(x - width/2, f, f"{int(f)}", ha="center", va="bottom", fontsize=9)
    ax.text(x + width/2, r, f"{int(r)}", ha="center", va="bottom", fontsize=9)
ax.set_ylim(0, max(sel.flow_rx.max(), sel.range_rx.max()) * 1.25)
ax.set(xticks=list(xs), xticklabels=sel.run, ylabel="Received messages (count)", title="Sensor message reception in diagnostic runs"); ax.legend()
fig.tight_layout(); fig.savefig(OUT / "fig2_rx_counts.png"); plt.close(fig)

# Flow component deviations from sent values, using results.csv plus the transmitted values in report.md §1 table.
hz["flow_x"] = pd.to_numeric(hz.of_flowX_mean, errors="coerce")
hz["flow_y"] = pd.to_numeric(hz.of_flowY_mean, errors="coerce")
hz["flow_x_error_mrad"] = (hz.flow_x - 0.5) * 1000
hz["flow_y_error_mrad"] = (hz.flow_y + 0.25) * 1000
fig, ax = plt.subplots(figsize=(7,4))
ax.plot(hz.target_hz, hz.flow_x_error_mrad, marker="o", label="flowX")
ax.plot(hz.target_hz, hz.flow_y_error_mrad, marker="s", label="flowY")
ax.axhline(0, color="#000000", ls=":", alpha=.6)
ax.set(xlabel="Configured send rate (Hz)", ylabel="Mean component error (mrad/s)", title="Dataflash flow components stay close to transmitted values"); ax.legend()
fig.tight_layout(); fig.savefig(OUT / "fig3_flow_components.png"); plt.close(fig)
