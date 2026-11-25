import glob, json, sys
from multiprocessing import Pool
import pandas as pd
from lat_timing import one, HARD
F = HARD + ["r_a", "r_i", "m_ooN", "m_blip"]
if __name__ == "__main__":
    rds = [rd for a in sys.argv[1].split(",") for f in F for rd in sorted(glob.glob("runs/ab_%s__%s__r*" % (a, f)))]
    with Pool(3) as p:
        res = [r for r in p.map(one, rds) if r]
    json.dump(res, open("lat_timing_r4.json", "w"))
    d = pd.DataFrame(res); d["arm"] = d.tag.str.extract(r"ab_(.+?)__"); d["fault"] = d.tag.str.extract(r"__(.+?)__r")
    pd.set_option("display.width", 250); pd.set_option("display.max_rows", 500)
    print(d.groupby(["arm", "fault"]).agg(n=("tag", "size"), lat_pre_max=("lat_pre", "max"), lat_post_med=("lat_post", "median"), lat_post_max=("lat_post", "max"), desc_s=("desc_s", "median"), v_post_max=("vmax_post", "max")).round(2))
    print(d.groupby("arm").agg(n=("tag", "size"), lat_post_med=("lat_post", "median"), lat_post_p90=("lat_post", lambda x: x.quantile(.9)), lat_post_max=("lat_post", "max"), v_post_max=("vmax_post", "max")).round(2))
