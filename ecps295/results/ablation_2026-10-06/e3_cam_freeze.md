# Ablation vs `cam_freeze`

## contacts per 100 m flown (primary safety metric) (lower is better)

reference `cam_freeze`: 2.083 [0.000, 6.250] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 6.201 [0.897, 14.001] | +4.117 [-2.157, +12.707] | +0.40 | 0.625 |

## success (no contact, no FC landing) (higher is better)

reference `cam_freeze`: 0.900 [0.700, 1.000] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.600 [0.300, 0.900] | -0.300 [-0.600, +0.000] | -0.30 | 0.25 |

## episodes with contact (lower is better)

reference `cam_freeze`: 0.100 [0.000, 0.300] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.400 [0.100, 0.700] | +0.300 [+0.000, +0.600] | +0.30 | 0.25 |

## FC landings (lower is better)

reference `cam_freeze`: 0.000 [0.000, 0.000] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## contacts per episode (lower is better)

reference `cam_freeze`: 0.100 [0.000, 0.300] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.900 [0.200, 1.800] | +0.800 [+0.100, +1.700] | +1.00 | 0.125 |

## near misses per episode (lower is better)

reference `cam_freeze`: 0.300 [0.100, 0.600] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.900 [0.200, 1.600] | +0.600 [+0.000, +1.200] | +0.80 | 0.188 |

## min clearance m (higher is better)

reference `cam_freeze`: 0.480 [0.207, 0.776] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.298 [0.065, 0.572] | -0.182 [-0.454, +0.053] | -0.42 | 0.301 |

## path m (higher is better)

reference `cam_freeze`: 5.680 [5.080, 6.280] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 18.660 [14.330, 22.460] | **+12.980 [+8.400, +17.000]** | +1.00 | 0.00195 |

## coverage (higher is better)

reference `cam_freeze`: 0.175 [0.152, 0.196] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.398 [0.310, 0.479] | **+0.223 [+0.140, +0.300]** | +1.00 | 0.00391 |

## time over boxes s (lower is better)

reference `cam_freeze`: 2.210 [1.170, 3.610] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 15.710 [9.890, 21.730] | **+13.500 [+7.800, +19.230]** | +0.96 | 0.00391 |

## max true height m (lower is better)

reference `cam_freeze`: 0.873 [0.840, 0.909] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 1.054 [1.002, 1.093] | **+0.181 [+0.132, +0.229]** | +1.00 | 0.00195 |

## episodes above the 1.3 m hard fence (lower is better)

reference `cam_freeze`: 0.000 [0.000, 0.000] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## FC height error max m (lower is better)

reference `cam_freeze`: 0.092 [0.068, 0.116] (n=10)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| cam_freeze_no_wd | 10 | 0.215 [0.125, 0.368] | **+0.123 [+0.023, +0.287]** | +0.78 | 0.0273 |

## invalid runs (excluded with their pair)

