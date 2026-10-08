# Ablation vs `full`

## episodes with contact (lower is better)

reference `full`: 0.000 [0.000, 0.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 0.500 [0.344, 0.688] | **+0.500 [+0.344, +0.656]** | +0.50 | 0.000122 |
| no_new_cells | 32 | 0.469 [0.281, 0.625] | **+0.469 [+0.312, +0.625]** | +0.47 | 0.000183 |
| no_saccade | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| no_efference | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## episodes with an escape (higher is better)

reference `full`: 1.000 [1.000, 1.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| no_new_cells | 32 | 0.844 [0.719, 0.969] | -0.156 [-0.281, -0.031] | -0.16 | 0.25 |
| no_saccade | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| no_efference | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## min clearance m (higher is better)

reference `full`: 0.682 [0.593, 0.772] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 0.102 [0.033, 0.172] | **-0.580 [-0.737, -0.427]** | -0.99 | 2.38e-06 |
| no_new_cells | 32 | 0.112 [0.042, 0.183] | **-0.570 [-0.727, -0.413]** | -1.00 | 2.38e-06 |
| no_saccade | 32 | 0.188 [0.152, 0.225] | **-0.494 [-0.563, -0.428]** | -1.00 | 2.38e-06 |
| no_efference | 32 | 0.438 [0.391, 0.487] | **-0.244 [-0.326, -0.167]** | -0.93 | 9.42e-07 |

## final clearance m (higher is better)

reference `full`: 0.820 [0.752, 0.888] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 0.430 [0.330, 0.529] | **-0.390 [-0.537, -0.250]** | -0.84 | 1.38e-05 |
| no_new_cells | 32 | 0.434 [0.338, 0.530] | **-0.386 [-0.518, -0.256]** | -0.86 | 3.21e-05 |
| no_saccade | 32 | 0.311 [0.253, 0.374] | **-0.508 [-0.580, -0.435]** | -0.99 | 5.59e-09 |
| no_efference | 32 | 0.601 [0.553, 0.646] | **-0.219 [-0.305, -0.138]** | -0.81 | 3.21e-05 |

## clearance at escape onset m (episodes with an escape) (higher is better)

reference `full`: 0.889 [0.788, 0.991] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 0.198 [0.112, 0.287] | **-0.691 [-0.879, -0.504]** | -1.00 | 3.15e-06 |
| no_new_cells | 27 | 0.319 [0.232, 0.398] | **-0.516 [-0.704, -0.340]** | -1.00 | 2.48e-05 |
| no_saccade | 32 | 0.869 [0.758, 0.983] | -0.020 [-0.041, -0.001] | -0.34 | 0.203 |
| no_efference | 32 | 0.874 [0.763, 0.986] | -0.015 [-0.040, +0.008] | -0.08 | 0.713 |

## clearance at brake onset m (episodes with a brake) (higher is better)

reference `full`: 1.097 [1.024, 1.158] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 0.923 [0.755, 1.073] | -0.173 [-0.346, -0.021] | -0.34 | 0.508 |
| no_new_cells | 32 | 0.980 [0.837, 1.102] | -0.117 [-0.269, +0.019] | -0.31 | 0.508 |
| no_saccade | 32 | 1.051 [0.955, 1.134] | -0.046 [-0.169, +0.073] | -0.16 | 0.88 |
| no_efference | 32 | 1.089 [1.005, 1.159] | -0.008 [-0.091, +0.073] | +0.03 | 0.882 |

## contacts per episode (lower is better)

reference `full`: 0.000 [0.000, 0.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| no_lcb | 32 | 1.469 [0.844, 2.188] | **+1.469 [+0.844, +2.188]** | +1.00 | 0.00163 |
| no_new_cells | 32 | 1.406 [0.812, 2.094] | **+1.406 [+0.781, +2.094]** | +1.00 | 0.00165 |
| no_saccade | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| no_efference | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## invalid runs (excluded with their pair)

