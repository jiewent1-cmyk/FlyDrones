import json
import sys

keys = ["tag", "truth_max_all", "alt_max", "est_leg_rms", "est_cnr_rms", "alt_err_max",
        "climb_est_rms", "climb_overshoot", "f_true_max", "f_after_max", "mission_complete",
        "mon_trigger_rel", "left_auto_at", "final_mode", "rfnd_dist_air_min", "rfnd_dist_air_max",
        "rfnd_stat_air"]
for t in sys.argv[1:]:
    d = json.load(open("runs/%s/metrics.json" % t))
    print({k: d.get(k) for k in keys})
