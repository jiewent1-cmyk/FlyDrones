# Ablation vs `no_lcv`

## contacts per 100 m flown (primary safety metric) (lower is better)

reference `no_lcv`: 1.133 [0.000, 2.648] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 2.674 [0.730, 5.252] | +1.541 [-0.552, +3.914] | +0.50 | 0.297 |

## success (no contact, no FC landing) (higher is better)

reference `no_lcv`: 0.850 [0.700, 1.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.700 [0.500, 0.900] | -0.150 [-0.350, +0.050] | -0.15 | 0.375 |

## episodes with contact (lower is better)

reference `no_lcv`: 0.150 [0.000, 0.300] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.300 [0.100, 0.500] | +0.150 [-0.050, +0.350] | +0.15 | 0.375 |

## FC landings (lower is better)

reference `no_lcv`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## contacts per episode (lower is better)

reference `no_lcv`: 0.250 [0.000, 0.600] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.650 [0.200, 1.250] | +0.400 [-0.100, +1.000] | +0.57 | 0.25 |

## near misses per episode (lower is better)

reference `no_lcv`: 0.700 [0.250, 1.200] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 1.300 [0.500, 2.200] | +0.600 [-0.050, +1.250] | +0.60 | 0.121 |

## min clearance m (higher is better)

reference `no_lcv`: 0.217 [0.117, 0.337] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.244 [0.111, 0.396] | +0.026 [-0.097, +0.163] | -0.01 | 0.985 |

## path m (higher is better)

reference `no_lcv`: 25.775 [23.520, 27.645] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 26.070 [23.960, 27.885] | +0.295 [-0.930, +1.490] | +0.19 | 0.467 |

## coverage (higher is better)

reference `no_lcv`: 0.540 [0.482, 0.593] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.542 [0.490, 0.588] | +0.002 [-0.033, +0.035] | +0.12 | 0.627 |

## time over boxes s (lower is better)

reference `no_lcv`: 26.740 [21.100, 32.370] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 25.525 [20.055, 30.690] | -1.215 [-7.040, +4.135] | -0.03 | 0.904 |

## max true height m (lower is better)

reference `no_lcv`: 0.976 [0.955, 0.999] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.965 [0.948, 0.981] | -0.011 [-0.036, +0.012] | -0.25 | 0.36 |

## episodes above the 1.3 m hard fence (lower is better)

reference `no_lcv`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## FC height error max m (lower is better)

reference `no_lcv`: 0.123 [0.104, 0.142] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcv | 20 | 0.126 [0.114, 0.136] | +0.003 [-0.013, +0.020] | +0.05 | 0.869 |

## invalid runs (excluded with their pair)

