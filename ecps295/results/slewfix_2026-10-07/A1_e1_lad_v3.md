# Ablation vs `lad_v3`

## contacts per 100 m flown (primary safety metric) (lower is better)

reference `lad_v3`: 0.392 [0.000, 1.176] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 2.682 [0.000, 7.154] | +2.290 [-0.730, +6.958] | +0.60 | 0.375 |

## success (no contact, no FC landing) (higher is better)

reference `lad_v3`: 0.950 [0.850, 1.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.850 [0.700, 1.000] | -0.100 [-0.300, +0.100] | -0.10 | 0.625 |

## episodes with contact (lower is better)

reference `lad_v3`: 0.050 [0.000, 0.150] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.150 [0.000, 0.300] | +0.100 [-0.100, +0.300] | +0.10 | 0.625 |

## FC landings (lower is better)

reference `lad_v3`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.050 [0.000, 0.150] | +0.050 [+0.000, +0.150] | +0.05 | 1 |

## contacts per episode (lower is better)

reference `lad_v3`: 0.100 [0.000, 0.300] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.200 [0.000, 0.450] | +0.100 [-0.200, +0.400] | +0.30 | 0.75 |

## near misses per episode (lower is better)

reference `lad_v3`: 0.750 [0.300, 1.300] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.700 [0.250, 1.250] | -0.050 [-0.700, +0.650] | -0.11 | 0.812 |

## min clearance m (higher is better)

reference `lad_v3`: 0.275 [0.152, 0.410] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.183 [0.089, 0.287] | -0.092 [-0.199, +0.007] | -0.28 | 0.271 |

## path m (higher is better)

reference `lad_v3`: 26.420 [24.460, 27.885] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 24.345 [21.140, 27.145] | -2.075 [-5.015, +0.190] | -0.21 | 0.421 |

## coverage (higher is better)

reference `lad_v3`: 0.576 [0.518, 0.623] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.502 [0.436, 0.562] | -0.074 [-0.127, -0.022] | -0.51 | 0.0509 |

## time over boxes s (lower is better)

reference `lad_v3`: 26.895 [20.500, 33.325] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 24.210 [17.725, 30.805] | -2.685 [-8.305, +2.700] | -0.17 | 0.507 |

## max true height m (lower is better)

reference `lad_v3`: 0.958 [0.940, 0.977] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 1.011 [0.947, 1.116] | +0.052 [-0.015, +0.157] | +0.30 | 0.239 |

## episodes above the 1.3 m hard fence (lower is better)

reference `lad_v3`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.050 [0.000, 0.150] | +0.050 [+0.000, +0.150] | +0.05 | 1 |

## FC height error max m (lower is better)

reference `lad_v3`: 0.114 [0.099, 0.129] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_lad_v3 | 20 | 0.247 [0.113, 0.498] | +0.133 [-0.003, +0.383] | +0.32 | 0.204 |

## invalid runs (excluded with their pair)

