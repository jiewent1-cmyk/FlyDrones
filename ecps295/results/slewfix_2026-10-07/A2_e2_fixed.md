# Ablation vs `fx_full`

## episodes with contact (lower is better)

reference `fx_full`: 0.000 [0.000, 0.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 0.500 [0.344, 0.688] | **+0.500 [+0.344, +0.656]** | +0.50 | 0.000122 |
| fx_no_new_cells | 32 | 0.469 [0.312, 0.625] | **+0.469 [+0.281, +0.656]** | +0.47 | 0.000183 |
| fx_no_saccade | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| fx_no_efference | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## episodes with an escape (higher is better)

reference `fx_full`: 1.000 [1.000, 1.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | diff in proportion | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| fx_no_new_cells | 32 | 0.844 [0.719, 0.969] | -0.156 [-0.281, -0.031] | -0.16 | 0.25 |
| fx_no_saccade | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| fx_no_efference | 32 | 1.000 [1.000, 1.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## min clearance m (higher is better)

reference `fx_full`: 0.625 [0.538, 0.716] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 0.102 [0.029, 0.175] | **-0.523 [-0.685, -0.368]** | -0.93 | 1.2e-05 |
| fx_no_new_cells | 32 | 0.134 [0.060, 0.209] | **-0.491 [-0.653, -0.335]** | -0.96 | 2.4e-07 |
| fx_no_saccade | 32 | 0.175 [0.144, 0.207] | **-0.450 [-0.517, -0.385]** | -1.00 | 1.86e-09 |
| fx_no_efference | 32 | 0.478 [0.428, 0.531] | **-0.147 [-0.226, -0.075]** | -0.57 | 0.00413 |

## final clearance m (higher is better)

reference `fx_full`: 0.778 [0.698, 0.863] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 0.435 [0.327, 0.541] | **-0.344 [-0.504, -0.192]** | -0.70 | 0.000613 |
| fx_no_new_cells | 32 | 0.422 [0.329, 0.517] | **-0.356 [-0.501, -0.220]** | -0.83 | 2.62e-05 |
| fx_no_saccade | 32 | 0.296 [0.247, 0.348] | **-0.482 [-0.553, -0.415]** | -1.00 | 1.86e-09 |
| fx_no_efference | 32 | 0.607 [0.555, 0.660] | **-0.171 [-0.270, -0.076]** | -0.59 | 0.00256 |

## clearance at escape onset m (episodes with an escape) (higher is better)

reference `fx_full`: 0.887 [0.781, 0.994] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 0.199 [0.108, 0.293] | **-0.688 [-0.889, -0.487]** | -0.97 | 6.15e-06 |
| fx_no_new_cells | 27 | 0.404 [0.303, 0.503] | **-0.427 [-0.618, -0.254]** | -0.94 | 6.01e-05 |
| fx_no_saccade | 32 | 0.877 [0.763, 0.993] | -0.010 [-0.026, +0.008] | -0.27 | 0.396 |
| fx_no_efference | 32 | 0.894 [0.783, 1.008] | +0.007 [-0.015, +0.030] | +0.12 | 0.564 |

## clearance at brake onset m (episodes with a brake) (higher is better)

reference `fx_full`: 1.007 [0.906, 1.095] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 0.866 [0.701, 1.019] | -0.141 [-0.300, +0.009] | -0.34 | 0.428 |
| fx_no_new_cells | 32 | 0.987 [0.836, 1.116] | -0.020 [-0.182, +0.136] | -0.04 | 1 |
| fx_no_saccade | 32 | 1.076 [0.973, 1.157] | +0.069 [-0.083, +0.211] | +0.25 | 0.659 |
| fx_no_efference | 32 | 1.034 [0.947, 1.114] | +0.028 [-0.116, +0.176] | +0.06 | 1 |

## contacts per episode (lower is better)

reference `fx_full`: 0.000 [0.000, 0.000] (n=32)

| condition | n pairs | mean [95% CI] | diff vs ref [95% CI] | rank-biserial r | p (Holm) |
|---|---|---|---|---|---|
| fx_no_lcb | 32 | 1.188 [0.719, 1.688] | **+1.188 [+0.719, +1.656]** | +1.00 | 0.0015 |
| fx_no_new_cells | 32 | 1.062 [0.625, 1.531] | **+1.062 [+0.625, +1.562]** | +1.00 | 0.00173 |
| fx_no_saccade | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |
| fx_no_efference | 32 | 0.000 [0.000, 0.000] | +0.000 [+0.000, +0.000] | +0.00 | 1 |

## invalid runs (excluded with their pair)

