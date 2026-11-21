import sys
sys.path.insert(0, ".")
from flydrones.config import load_config
from ecps_retina import EcpsRetina
for y in ("minifly/s5.yaml", "minifly/sh5u.yaml", "minifly/v4.yaml"):
    cfg = load_config(y, {"brain": {"seed": 0}})
    r = EcpsRetina.from_config(cfg)
    sh = ((cfg.get("ecps_pilot", {}) or {}).get("struct", {}) or {}).get("shield", {})
    print(y, "tof_on", r.tof_on, r.tof_beams if r.tof_on else "", "shield", sh.get("enabled"), sh.get("beams"))
