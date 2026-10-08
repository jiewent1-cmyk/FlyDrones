# Ablation vs `no_retreat`

## contacts per 100 m flown (primary safety metric) (lower is better)

reference `no_retreat`: 0.844 [0.000, 2.112] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 1.533 [0.000, 4.046] | +0.689 [-0.840, +2.491] | +0.67 | 0.5 |

## success (no contact, no FC landing) (higher is better)

reference `no_retreat`: 0.900 [0.750, 1.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.900 [0.750, 1.000] | +0.000 [-0.150, +0.150] | +0.00 | 1 |

## episodes with contact (lower is better)

reference `no_retreat`: 0.100 [0.000, 0.250] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.100 [0.000, 0.250] | +0.000 [-0.150, +0.150] | +0.00 | 1 |

## FC landings (lower is better)

reference `no_retreat`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## contacts per episode (lower is better)

reference `no_retreat`: 0.200 [0.000, 0.500] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.150 [0.000, 0.400] | -0.050 [-0.350, +0.250] | -0.17 | 1 |

## near misses per episode (lower is better)

reference `no_retreat`: 0.700 [0.150, 1.400] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.550 [0.200, 1.050] | -0.150 [-0.750, +0.400] | -0.24 | 0.719 |

## min clearance m (higher is better)

reference `no_retreat`: 0.231 [0.139, 0.334] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.199 [0.115, 0.293] | -0.032 [-0.103, +0.039] | -0.21 | 0.441 |

## path m (higher is better)

reference `no_retreat`: 25.115 [23.000, 26.985] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 24.165 [21.185, 26.610] | -0.950 [-3.420, +1.110] | -0.15 | 0.563 |

## coverage (higher is better)

reference `no_retreat`: 0.544 [0.480, 0.604] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.506 [0.439, 0.568] | -0.038 [-0.090, +0.014] | -0.29 | 0.297 |

## time over boxes s (lower is better)

reference `no_retreat`: 22.805 [16.465, 29.075] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 22.355 [15.665, 29.695] | -0.450 [-6.175, +5.080] | +0.00 | 1 |

## max true height m (lower is better)

reference `no_retreat`: 0.945 [0.925, 0.964] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.941 [0.925, 0.958] | -0.003 [-0.029, +0.021] | -0.04 | 0.877 |

## episodes above the 1.3 m hard fence (lower is better)

reference `no_retreat`: 0.000 [0.000, 0.000] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## FC height error max m (lower is better)

reference `no_retreat`: 0.121 [0.103, 0.138] (n=20)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_retreat | 20 | 0.103 [0.089, 0.117] | -0.018 [-0.040, +0.003] | -0.34 | 0.185 |

## invalid runs (excluded with their pair)

