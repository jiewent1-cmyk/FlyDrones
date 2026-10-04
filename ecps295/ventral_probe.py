"""Open-loop probe of the v4 LCv pathway: ventral level 0 -> 1 -> 0 with a static frame, DN rates and decoded command.

python ventral_probe.py minifly/v4.yaml
"""

import sys

import numpy as np
from ecps_decoder import EcpsDecoder
from ecps_retina import EcpsRetina

from flydrones.brain import Brain, load_connectome
from flydrones.config import load_config
from flydrones.senses import InputEncoder

cfg = load_config(sys.argv[1])
brain = Brain(load_connectome(str(__import__("pathlib").Path(sys.argv[1]).resolve().parent / cfg["brain"]["source"])), cfg)
retina = EcpsRetina.from_config(cfg)
enc = InputEncoder(brain.connectome, cfg)
dec = EcpsDecoder(cfg)
frame = (np.random.default_rng(0).random((144, 192)) * 255).astype(np.uint8)  # static texture: no flow
dt = 1.0 / cfg["control"]["hz"]
retina.baro_m = 0.0
for phase, rng, secs in (("floor", 0.7, 3.0), ("box under", 0.3, 2.0), ("floor", 0.7, 3.0)):
    for k in range(int(secs / dt)):
        retina.range_m = rng
        vf = retina.encode(frame)
        rates = brain.tick(enc.encode(vf, 0.0), ms=dt * 1000)
        cmd = dec.update(rates, dt)
        if k % int(0.5 / dt) == 0:
            dn = {n: round(v) for n, v in rates.items() if n.startswith(("DNp03", "DNp01", "MDN"))}
            print(
                f"{phase:9s} t+{k * dt:3.1f}s ventral {vf.eyes['L'].grids['ventral'].mean():.2f} {dn} "
                f"fwd {cmd.forward:+.2f} yaw {cmd.yaw:+.2f} thr {cmd.throttle:+.2f} escape {int(cmd.escape)}"
            )
