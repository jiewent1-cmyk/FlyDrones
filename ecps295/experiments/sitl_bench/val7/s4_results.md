### 8.1 无异常基线（10 次闭环，监视器在线但无异常注入）

| 运行数 | 完成数 | 监视器触发 | 真值最高 [m] | 真值最低 [m] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 |
|---|---|---|---|---|---|---|---|
| 10 | 10 | 0/10 | 2.056 [2.038..2.070] | -0.000 [-0.000..-0.000] | 0.497 [0.448..0.532] | 0.293 [0.128..0.525] | 10/10 |

基线取真值最高的中位数 **2.070 m**（范围 [2.038..2.070] m）。

10 次无异常闭环飞行监视器 **一次都没有触发**（0/10），飞控没有出现任何监视器相关的动作。


### 8.2 每个异常格 × 动作：触发、动作延迟、落地

| 异常格 | 动作 | 触发 | 注入→动作 [s] | 动作→触地 [s] | 真值最高 [m] | 相对基线 [m] | 着地速度 [m/s] | 下降率最陡 [m/s] | 空中悬停 [s] | 落点偏差 [m] | 任务完成 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| a | A1 | 5/5 | 0.810 [0.670..0.850] | 4.310 [4.210..4.590] | 1.789 [1.765..1.827] | -0.281 | 0.526 [0.514..0.537] | - | -0.300 | 3.274 [1.312..3.635] | 0/5 |
| a | A2 | 5/5 | 0.800 [0.600..0.900] | 5.360 [4.890..5.680] | 2.006 [1.862..2.061] | -0.064 | 0.463 [0.457..0.516] | - | -0.300 | 0.058 [0.010..0.106] | 0/5 |
| a | A3 | 5/5 | 0.790 [0.710..0.910] | 43.420 [41.840..43.950] | 2.186 [2.099..2.279] | +0.116 | 0.486 [0.463..0.507] | -0.738 | 4.200 | 0.375 [0.159..0.607] | 5/5 |
| a | none | 0/5 | - - | - - | 3.651 [3.629..3.699] | +1.581 | 0.854 [0.447..0.909] | - | -0.200 | 0.162 [0.085..0.283] | 0/5 |
| c | A1 | 5/5 | 0.250 [0.110..0.330] | 4.430 [4.230..4.630] | 2.070 [2.050..2.084] | +0.000 | 0.504 [0.449..0.538] | - | -0.300 | 0.377 [0.292..0.586] | 0/5 |
| c | A2 | 5/5 | 0.180 [0.160..0.300] | 4.970 [4.730..5.340] | 2.076 [2.030..2.129] | +0.006 | 0.520 [0.497..0.545] | - | -0.300 | 0.183 [0.013..0.506] | 0/5 |
| c | A3 | 5/5 | 0.260 [0.200..0.260] | 42.570 [41.300..43.100] | 2.111 [2.053..2.142] | +0.041 | 0.514 [0.475..0.517] | -0.705 | 4.200 | 0.929 [0.698..1.030] | 5/5 |
| c | none | 0/5 | - - | - - | 2.067 [2.063..2.094] | -0.003 | 0.471 [0.444..0.501] | -0.750 | 4.100 | 0.535 [0.304..1.045] | 5/5 |
| g3 | A1 | 5/5 | 0.230 [0.120..0.260] | 4.410 [4.220..4.430] | 2.058 [2.047..2.085] | -0.012 | 0.481 [0.417..0.557] | - | -0.300 | 1.033 [0.854..1.175] | 0/5 |
| g3 | A2 | 5/5 | 0.210 [0.150..0.250] | 5.270 [4.880..5.430] | 2.074 [2.028..2.134] | +0.004 | 0.489 [0.478..0.545] | - | -0.300 | 0.173 [0.112..0.807] | 0/5 |
| g3 | A3 | 5/5 | 0.200 [0.170..0.260] | 25.530 [24.830..41.860] | 2.095 [2.074..2.177] | +0.025 | 0.434 [0.328..0.486] | -0.872 | -0.300 | 86.653 [1.007..113.741] | 1/5 |
| g3 | none | 0/5 | - - | - - | 2.061 [2.050..2.083] | -0.009 | 0.488 [0.456..0.521] | -0.712 | 4.100 | 0.730 [0.641..1.022] | 5/5 |
| g5 | A1 | 5/5 | 0.200 [0.180..0.260] | 4.360 [4.170..4.550] | 2.052 [2.040..2.069] | -0.018 | 0.480 [0.428..0.511] | - | -0.300 | 1.102 [0.878..1.305] | 0/5 |
| g5 | A2 | 5/5 | 0.200 [0.150..0.260] | 5.230 [4.890..5.420] | 2.041 [2.025..2.060] | -0.029 | 0.459 [0.426..0.523] | - | -0.300 | 0.763 [0.566..0.841] | 0/5 |
| g5 | A3 | 5/5 | 0.210 [0.200..0.260] | 42.680 [16.600..45.200] | 2.343 [2.058..2.547] | +0.273 | 0.498 [0.295..0.502] | -0.648 | 4.100 | 7.857 [1.759..197.979] | 3/5 |
| g5 | none | 0/5 | - - | - - | 2.818 [2.131..4.213] | +0.748 | 0.517 [0.496..1.482] | - | -0.300 | 4.002 [2.854..36.534] | 0/5 |
| g10 | A1 | 5/5 | 0.220 [0.120..0.270] | 4.370 [4.240..4.570] | 2.061 [2.044..2.096] | -0.009 | 0.499 [0.449..0.514] | - | -0.300 | 1.002 [0.625..1.229] | 0/5 |
| g10 | A2 | 5/5 | 0.190 [0.140..0.250] | 5.300 [5.130..5.620] | 2.061 [2.049..2.137] | -0.009 | 0.502 [0.478..0.539] | - | -0.300 | 1.014 [0.230..1.409] | 0/5 |
| g10 | A3 | 5/5 | 0.200 [0.170..0.220] | 41.750 [17.490..44.990] | 2.312 [2.059..2.594] | +0.242 | 0.490 [0.466..0.848] | -0.565 | 4.100 | 3.739 [0.844..81.376] | 3/5 |
| g10 | none | 0/5 | - - | - - | 2.437 [2.068..3.969] | +0.367 | 0.551 [0.459..0.908] | - | -0.300 | 5.660 [1.465..7.424] | 0/5 |
| gN | A1 | 5/5 | 0.230 [0.160..0.270] | 4.300 [4.180..4.520] | 2.048 [2.041..2.075] | -0.022 | 0.500 [0.480..0.509] | - | -0.300 | 0.956 [0.739..1.267] | 0/5 |
| gN | A2 | 5/5 | 0.230 [0.120..0.250] | 5.280 [5.090..5.800] | 2.058 [2.048..2.065] | -0.012 | 0.484 [0.468..0.532] | - | -0.300 | 0.762 [0.493..0.991] | 0/5 |
| gN | A3 | 5/5 | 0.200 [0.130..0.220] | 43.200 [12.820..44.930] | 2.501 [2.067..2.561] | +0.431 | 0.529 [0.467..0.531] | -0.562 | 4.600 | 1.152 [0.713..21.202] | 3/5 |
| gN | none | 0/5 | - - | - - | 2.416 [2.066..2.449] | +0.346 | 0.520 [0.473..0.543] | - | - | 5.409 [4.326..12.016] | 0/5 |
| h | A1 | 5/5 | 0.730 [0.630..0.860] | 3.530 [3.490..3.700] | 1.264 [1.204..1.319] | -0.806 | 0.468 [0.423..0.470] | - | -0.300 | 0.582 [0.506..0.799] | 0/5 |
| h | A2 | 5/5 | 0.800 [0.670..0.850] | 4.920 [4.430..5.210] | 1.591 [1.481..1.644] | -0.479 | 0.484 [0.469..0.500] | - | -0.300 | 1.427 [0.131..1.613] | 0/5 |
| h | A3 | 5/5 | 0.710 [0.620..0.900] | 45.510 [44.570..46.310] | 2.509 [2.358..3.097] | +0.439 | 0.546 [0.526..0.559] | -0.571 | 4.700 | 0.793 [0.694..1.782] | 5/5 |
| h | none | 0/5 | - - | - - | 4.023 [4.018..4.024] | +1.953 | 0.484 [0.457..0.535] | -0.765 | 11.800 | 7.292 [5.941..7.454] | 5/5 |
| i | A1 | 5/5 | 9.960 [7.800..10.280] | 6.510 [6.450..6.680] | 3.083 [2.826..3.105] | +1.013 | 0.521 [0.490..0.546] | - | -0.300 | 2.025 [1.973..2.181] | 0/5 |
| i | A2 | 5/5 | 9.210 [8.080..10.000] | 8.060 [7.800..8.440] | 3.059 [3.007..3.116] | +0.989 | 0.525 [0.481..0.582] | - | -0.300 | 2.199 [2.051..2.345] | 0/5 |
| i | A3 | 5/5 | 9.770 [8.560..10.020] | 34.520 [33.580..35.130] | 3.411 [3.228..3.480] | +1.341 | 0.487 [0.474..0.499] | -0.745 | 6.000 | 0.379 [0.256..0.593] | 5/5 |
| i | none | 0/5 | - - | - - | 3.226 [3.214..3.249] | +1.156 | 0.159 [0.057..0.491] | -0.591 | 6.400 | 2.899 [1.125..3.734] | 5/5 |
| j1 | A1 | 5/5 | 0.250 [0.210..0.310] | 4.370 [4.170..4.430] | 2.058 [2.036..2.068] | -0.012 | 0.497 [0.473..0.544] | - | -0.300 | 0.113 [0.038..0.198] | 0/5 |
| j1 | A2 | 5/5 | 0.230 [0.210..0.260] | 5.070 [4.480..5.170] | 2.061 [2.034..2.124] | -0.009 | 0.418 [0.402..0.499] | - | -0.300 | 0.239 [0.127..0.332] | 0/5 |
| j1 | A3 | 5/5 | 0.250 [0.190..0.310] | 42.490 [41.500..42.840] | 2.139 [2.101..2.145] | +0.069 | 0.488 [0.468..0.501] | -0.689 | 4.200 | 0.807 [0.311..0.883] | 5/5 |
| j1 | none | 0/5 | - - | - - | 2.061 [2.038..2.067] | -0.009 | 0.499 [0.439..1.215] | -0.629 | 4.000 | 0.559 [0.054..4.797] | 4/5 |
| j2 | A1 | 5/5 | 0.300 [0.240..0.340] | 4.280 [4.080..4.360] | 2.056 [2.054..2.068] | -0.014 | 0.502 [0.500..0.512] | - | -0.300 | 0.082 [0.060..0.104] | 0/5 |
| j2 | A2 | 5/5 | 0.280 [0.170..0.310] | 4.850 [4.690..4.940] | 2.057 [2.023..2.077] | -0.013 | 0.455 [0.420..0.466] | - | -0.300 | 0.106 [0.048..0.208] | 0/5 |
| j2 | A3 | 5/5 | 0.260 [0.240..0.310] | 46.590 [44.130..48.190] | 2.106 [2.074..2.112] | +0.036 | 0.318 [0.283..0.472] | -0.608 | 4.200 | 0.304 [0.222..0.329] | 5/5 |
| j2 | none | 0/5 | - - | - - | 2.055 [2.037..2.061] | -0.015 | 0.897 [0.820..0.909] | - | -0.200 | 1.417 [1.304..1.572] | 0/5 |
| k | A1 | 5/5 | 1.580 [1.320..1.650] | 5.700 [4.550..6.420] | 1.564 [1.367..1.691] | -0.506 | 0.387 [0.351..0.446] | - | -0.400 | 1.004 [0.765..1.597] | 0/5 |
| k | A2 | 5/5 | 1.310 [1.110..1.550] | 5.640 [5.230..6.530] | 1.730 [1.481..1.921] | -0.340 | 0.482 [0.439..0.532] | - | -0.300 | 1.677 [0.385..1.939] | 0/5 |
| k | A3 | 5/5 | 1.480 [1.400..1.500] | 44.370 [43.860..45.240] | 2.676 [2.649..2.774] | +0.606 | 0.501 [0.474..0.560] | -0.728 | 4.600 | 0.538 [0.229..1.100] | 5/5 |
| k | none | 0/5 | - - | - - | 2.911 [2.810..6.111] | +0.841 | 0.571 [0.513..0.588] | -0.610 | 11.100 | 0.513 [0.231..6.089] | 1/5 |

### 8.3 `D3`（LOITER 手动起飞）复测：`gN` / `h` / `k`

| 异常格 | 动作 | 触发 | 注入→动作 [s] | 动作→触地 [s] | 真值最高 [m] | 相对基线 [m] | 着地速度 [m/s] | 落点偏差 [m] | 任务完成 |
|---|---|---|---|---|---|---|---|---|---|
| gN | A1 | 5/5 | 0.150 [0.120..0.270] | 4.430 [4.140..4.530] | 2.066 [2.021..2.071] | -0.004 | 0.519 [0.443..0.531] | 0.852 [0.187..1.138] | 0/5 |
| gN | A2 | 5/5 | 0.220 [0.150..0.270] | 5.310 [5.190..5.730] | 2.071 [2.036..2.107] | +0.001 | 0.485 [0.448..0.499] | 0.896 [0.268..2.001] | 0/5 |
| gN | A3 | 5/5 | 0.220 [0.170..0.250] | 42.590 [16.060..43.590] | 2.556 [2.151..2.704] | +0.486 | 0.553 [0.537..0.584] | 1.631 [0.725..14.101] | 3/5 |
| gN | none | 0/5 | - - | - - | 3.990 [2.311..3.998] | +1.920 | 0.545 [0.520..0.553] | 5.726 [4.800..7.734] | 4/5 |
| h | A1 | 5/5 | 0.700 [0.670..0.870] | 3.510 [3.480..3.790] | 1.273 [1.250..1.324] | -0.797 | 0.471 [0.425..0.503] | 0.584 [0.473..0.749] | 0/5 |
| h | A2 | 5/5 | 0.670 [0.620..0.770] | 4.810 [4.680..4.970] | 1.567 [1.519..1.631] | -0.503 | 0.460 [0.431..0.512] | 0.212 [0.137..1.940] | 0/5 |
| h | A3 | 5/5 | 0.720 [0.670..0.800] | 46.420 [45.150..51.980] | 2.619 [2.527..2.979] | +0.549 | 0.531 [0.491..0.547] | 0.843 [0.566..16.464] | 4/5 |
| h | none | 0/5 | - - | - - | 4.017 [4.014..4.037] | +1.947 | 0.506 [0.466..0.540] | 5.867 [5.464..7.857] | 5/5 |
| k | A1 | 5/5 | 1.550 [1.520..1.720] | 4.130 [3.580..5.060] | 1.424 [1.235..1.561] | -0.646 | 0.410 [0.367..0.498] | 0.533 [0.484..0.804] | 0/5 |
| k | A2 | 5/5 | 1.570 [1.520..1.670] | 6.090 [5.100..7.770] | 1.808 [1.625..2.219] | -0.262 | 0.507 [0.496..0.556] | 1.266 [0.882..1.725] | 0/5 |
| k | A3 | 5/5 | 1.450 [1.200..1.570] | 45.860 [44.610..46.460] | 2.577 [2.392..3.039] | +0.507 | 0.536 [0.521..0.559] | 0.496 [0.169..0.895] | 5/5 |
| k | none | 0/5 | - - | - - | 3.983 [2.602..4.011] | +1.913 | 0.562 [0.491..0.575] | 5.223 [0.404..6.624] | 3/5 |

### 8.4 A3（只切源组、不切模式）的任务完成情况与动作后高度误差

| 组 | 任务完成 | 动作后高度误差 `alt_err_max` 中位 [m] | 动作后真值最低 [m] | EKF 高度跳变 [m] |
|---|---|---|---|---|
| a_A3 | 5/5 | 0.172 [0.098..0.271] | -0.000 | 0.290 [0.116..0.352] |
| c_A3 | 5/5 | 0.127 [0.111..0.154] | -0.000 | 0.000 [-0.109..0.054] |
| g3_A3 | 1/5 | 0.308 [0.177..0.619] | -0.000 | 0.040 [-0.083..0.113] |
| g5_A3 | 3/5 | 0.343 [0.145..0.495] | -0.000 | -0.090 [-0.157..0.152] |
| g10_A3 | 3/5 | 0.594 [0.286..0.813] | -0.000 | -0.063 [-0.150..0.004] |
| gN_A3 | 3/5 | 0.507 [0.061..0.591] | -0.000 | -0.064 [-0.152..-0.001] |
| h_A3 | 5/5 | 0.509 [0.358..1.097] | -0.000 | 0.384 [0.129..0.492] |
| i_A3 | 5/5 | 1.411 [1.228..1.480] | -0.000 | 0.118 [0.030..0.270] |
| j1_A3 | 5/5 | 0.142 [0.116..0.209] | -0.000 | 0.058 [-0.098..0.157] |
| j2_A3 | 5/5 | 0.112 [0.074..0.229] | -0.000 | -0.001 [-0.108..0.162] |
| k_A3 | 5/5 | 0.676 [0.649..0.774] | -0.000 | 0.320 [0.249..0.604] |
| d3_gN_A3 | 3/5 | - - | -0.000 | -0.014 [-0.109..0.135] |
| d3_h_A3 | 4/5 | - - | -0.000 | 0.384 [0.245..0.474] |
| d3_k_A3 | 5/5 | - - | -0.000 | 0.399 [0.197..0.467] |
| clean | 10/10 | 0.043 [0.037..0.062] | - | - - |

### 8.5 源切换（EKF 高度估计跳变）与实际高度变化

`mon_ekf_jump` = 切源组前后 XKF1 记录的 EKF 高度差；`mon_truth_change` = 同一时刻真值高度的变化。

| 组 | EKF 跳变 [m] | 真值变化 [m] | 真值最高 [m] |
|---|---|---|---|
| a_A2 | 0.319 [0.118..0.600] | 0.295 [0.275..0.313] | 2.006 |
| a_A3 | 0.290 [0.116..0.352] | 0.290 [0.277..0.307] | 2.186 |
| c_A2 | -0.096 [-0.222..0.147] | -0.032 [-0.073..-0.002] | 2.076 |
| c_A3 | 0.000 [-0.109..0.054] | -0.028 [-0.041..-0.013] | 2.111 |
| gN_A2 | -0.014 [-0.128..0.047] | -0.040 [-0.046..-0.000] | 2.058 |
| gN_A3 | -0.064 [-0.152..-0.001] | -0.039 [-0.044..-0.032] | 2.501 |
| h_A2 | 0.199 [0.111..0.395] | 0.293 [0.282..0.305] | 1.591 |
| h_A3 | 0.384 [0.129..0.492] | 0.315 [0.277..0.322] | 2.509 |
| k_A2 | 0.261 [0.182..0.452] | 0.375 [0.309..0.413] | 1.730 |
| k_A3 | 0.320 [0.249..0.604] | 0.410 [0.393..0.422] | 2.676 |
| d3_gN_A2 | -0.028 [-0.115..0.116] | -0.030 [-0.034..-0.009] | 2.071 |
| d3_gN_A3 | -0.014 [-0.109..0.135] | -0.017 [-0.043..-0.003] | 2.556 |
| d3_h_A2 | 0.242 [0.152..0.343] | 0.309 [0.305..0.323] | 1.567 |
| d3_h_A3 | 0.384 [0.245..0.474] | 0.287 [0.274..0.305] | 2.619 |
| d3_k_A2 | 0.476 [0.245..0.580] | 0.398 [0.299..0.529] | 1.808 |
| d3_k_A3 | 0.399 [0.197..0.467] | 0.402 [0.282..0.497] | 2.577 |
| i_A2 | -0.009 [-0.057..0.187] | 0.062 [0.054..0.092] | 3.059 |
| i_A3 | 0.118 [0.030..0.270] | 0.057 [0.042..0.072] | 3.411 |

### 8.6 各组出现过的飞控 `STATUSTEXT`（次数）

| 组 | n | STATUSTEXT |
|---|---|---|
| a_A1 | 5 | New fence(5); GCS:MONITOR: baro/range mismatch e=1.35 m, switching t(1); GCS:MONITOR: baro/range mismatch e=1.23 m, switching t(1); GCS:MONITOR: baro/range mismatch e=1.34 m, switching t(1); GCS:MONITOR: baro/range mismatch e=1.19 m, switching t(1); GCS:MONITOR: baro/range mismatch e=1.14 m, switching t(1); Circle fence breached(1); EKF3 IMU0 stopped aiding(1) |
| a_A2 | 5 | New fence(5) |
| a_A3 | 5 | New fence(5); Mission: 6 Land(5) |
| a_none | 5 | New fence(5); Max Alt fence breached(5); Fence breach cleared(5) |
| c_A1 | 5 | New fence(5); EKF variance(4); EKF Failsafe(4); EKF Failsafe Cleared(4); GCS:MONITOR: baro/range mismatch e=0.57 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.58 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.70 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.79 m, switching t(1) |
| c_A2 | 5 | New fence(5) |
| c_A3 | 5 | New fence(5); Mission: 6 Land(5) |
| c_none | 5 | New fence(5); Mission: 6 Land(5) |
| clean | 10 | New fence(10); Mission: 6 Land(10) |
| d3_gN_A1 | 5 | EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.83 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.69 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.55 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.63 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.72 m, switching t(1) |
| d3_gN_A2 | 5 | EKF variance(5); EKF Failsafe(4) |
| d3_gN_A3 | 5 | EKF variance(5); EKF Failsafe: changed to Land Mode(4); Mission: 6 Land(3); EKF Failsafe Cleared(2); EKF Failsafe(1); Circle fence breached(1) |
| d3_gN_none | 5 | EKF variance(5); EKF Failsafe: changed to Land Mode(5); Mission: 6 Land(4); EKF Failsafe Cleared(1); EKF Failsafe(1) |
| d3_h_A1 | 5 | EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.62 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.54 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.41 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.68 m, switching t(1) |
| d3_h_A2 | 5 | EKF variance(2); EKF Failsafe(1); EKF3 IMU0 stopped aiding(1); EKF3 IMU1 stopped aiding(1); EKF3 IMU0 started relative aiding(1); EKF3 IMU1 started relative aiding(1); EKF3 IMU0 fusing optical flow(1); EKF3 IMU1 fusing optical flow(1) |
| d3_h_A3 | 5 | EKF variance(5); Mission: 6 Land(4); EKF Failsafe: changed to Land Mode(3); Circle fence breached(2); EKF Failsafe Cleared(1); EKF Failsafe(1) |
| d3_h_none | 5 | Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(5) |
| d3_k_A1 | 5 | EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.60 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.40 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.43 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.83 m, switching t(1) |
| d3_k_A2 | 5 | EKF variance(5); EKF Failsafe(5) |
| d3_k_A3 | 5 | Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(2) |
| d3_k_none | 5 | EKF variance(5); Mission: 6 Land(3); EKF Failsafe: changed to Land Mode(3); Max Alt fence breached(2); Fence breach cleared(2); EKF Failsafe(2) |
| g10_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.58 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.64 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.68 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.54 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.66 m, switching t(1) |
| g10_A2 | 5 | New fence(5); EKF variance(5); EKF Failsafe(3) |
| g10_A3 | 5 | Circle fence breached(6); New fence(5); Mission: 6 Land(3); Fence breach cleared(1); EKF variance(1); EKF Failsafe: changed to Land Mode(1); EKF Failsafe Cleared(1) |
| g10_none | 5 | EKF Failsafe Cleared(6); New fence(5); EKF variance(3); EKF Failsafe: changed to Land Mode(3); EKF Failsafe(3); Circle fence breached(2); Max Alt fence breached(2); Fence breach cleared(2) |
| g3_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); EKF Failsafe Cleared(5); GCS:MONITOR: baro/range mismatch e=0.70 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.66 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.63 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.73 m, switching t(1) |
| g3_A2 | 5 | New fence(5) |
| g3_A3 | 5 | New fence(5); Circle fence breached(4); EKF variance(4); EKF Failsafe(4); EKF Failsafe Cleared(4); Fence breach cleared(3); Mission: 6 Land(1) |
| g3_none | 5 | New fence(5); Mission: 6 Land(5) |
| g5_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); EKF Failsafe Cleared(5); GCS:MONITOR: baro/range mismatch e=0.67 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.79 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.56 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.72 m, switching t(1) |
| g5_A2 | 5 | New fence(5) |
| g5_A3 | 5 | New fence(5); Mission: 6 Land(3); Circle fence breached(2); EKF Failsafe(2); EKF variance(1); EKF3 IMU1 stopped aiding(1); EKF3 IMU1 started relative aiding(1); EKF3 IMU0 stopped aiding(1) |
| g5_none | 5 | New fence(5); Max Alt fence breached(4); Fence breach cleared(4); EKF variance(3); EKF Failsafe Cleared(3); EKF Failsafe: changed to Land Mode(2); EKF Failsafe(1); Circle fence breached(1) |
| gN_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.73 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.53 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.86 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.70 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.61 m, switching t(1) |
| gN_A2 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5) |
| gN_A3 | 5 | New fence(5); EKF variance(5); EKF Failsafe: changed to Land Mode(5); EKF Failsafe Cleared(4); Mission: 6 Land(3); EKF Failsafe(2); Circle fence breached(2); EKF3 IMU1 stopped aiding(1) |
| gN_none | 5 | New fence(5); EKF variance(5); EKF Failsafe: changed to Land Mode(5); EKF Failsafe Cleared(4); EKF Failsafe(4); Circle fence breached(1) |
| h_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.60 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.57 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.64 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.53 m, switching t(1) |
| h_A2 | 5 | New fence(5); EKF variance(4); EKF Failsafe(3); EKF3 IMU1 stopped aiding(1); EKF3 IMU1 started relative aiding(1); EKF3 IMU0 stopped aiding(1); EKF3 IMU0 started relative aiding(1); EKF3 IMU0 fusing optical flow(1) |
| h_A3 | 5 | New fence(5); Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(2); EKF3 IMU0 stopped aiding(1); EKF3 IMU1 stopped aiding(1); EKF3 IMU0 started relative aiding(1); EKF3 IMU1 started relative aiding(1) |
| h_none | 5 | New fence(5); Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(5) |
| i_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.84 m, switching t(2); GCS:MONITOR: baro/range mismatch e=0.90 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.79 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.74 m, switching t(1) |
| i_A2 | 5 | New fence(5) |
| i_A3 | 5 | New fence(5); Mission: 6 Land(5) |
| i_none | 5 | New fence(5); Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(5) |
| j1_A1 | 5 | New fence(5); GCS:MONITOR: baro/range mismatch e=-0.73 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.68 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.80 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.52 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.75 m, switching (1) |
| j1_A2 | 5 | New fence(5) |
| j1_A3 | 5 | New fence(5); Mission: 6 Land(5) |
| j1_none | 5 | Circle fence breached(9); New fence(5); Mission: 6 Land(4); Max Alt fence breached(1); Fence breach cleared(1) |
| j2_A1 | 5 | New fence(5); GCS:MONITOR: baro/range mismatch e=-0.69 m, switching (2); GCS:MONITOR: baro/range mismatch e=-0.60 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.82 m, switching (1); GCS:MONITOR: baro/range mismatch e=-0.67 m, switching (1) |
| j2_A2 | 5 | New fence(5) |
| j2_A3 | 5 | New fence(5); Mission: 6 Land(5) |
| j2_none | 5 | New fence(5); Max Alt fence breached(5); Fence breach cleared(5) |
| k_A1 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5); GCS:MONITOR: baro/range mismatch e=0.46 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.55 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.38 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.61 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.54 m, switching t(1) |
| k_A2 | 5 | New fence(5); EKF variance(5); EKF Failsafe(5) |
| k_A3 | 5 | New fence(5); Mission: 6 Land(5); EKF variance(5); EKF Failsafe: changed to Land Mode(3); EKF3 IMU1 stopped aiding(1); EKF3 IMU1 started relative aiding(1); EKF3 IMU1 fusing optical flow(1) |
| k_none | 5 | New fence(5); EKF variance(5); Max Alt fence breached(4); Fence breach cleared(4); EKF Failsafe(4); Mission: 6 Land(1); EKF Failsafe: changed to Land Mode(1) |
| p0 | 3 | New fence(3); EKF variance(3); EKF Failsafe(3); GCS:MONITOR: baro/range mismatch e=0.55 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.69 m, switching t(1); GCS:MONITOR: baro/range mismatch e=0.75 m, switching t(1) |
| press | 3 | New fence(3); EKF variance(3); EKF Failsafe: changed to Land Mode(3); EKF Failsafe Cleared(2); EKF Failsafe(2); Mission: 6 Land(1); EKF3 IMU0 stopped aiding(1); EKF3 IMU1 stopped aiding(1) |
| stop | 3 | New fence(3); EKF variance(3); Mission: 6 Land(2); EKF Failsafe: changed to Land Mode(2); Circle fence breached(1); Fence breach cleared(1); EKF Failsafe(1) |


### 8.7 与任务书预期不一致的地方（实测原因）

**(1) 无监视器对照里，`c` / `g3` / `g5` / `g10` / `gN` / `j1` / `j2` 七格没有爬到 4–5 m（对照不够高）。**

测的是 `SIM_SONAR_POS_Z`（读数 = 真值 − POS_Z），POS_Z>0 时读数整体变小；这些故障把读数压到很小的值，
**落到量程/有效性判据之外**，飞控自己发现了，不是监视器发现的。实测证据：

* `runs/v7_s4_gN_none_r1`（`q3.py` 输出）：t=10 s 真值 2.065 m、EKF 1.970 m，测距读数掉到 0.035 m；
  随后飞控 `run.log` 里出现 `EKF variance` → `EKF Failsafe: changed to Land Mode`（40.7 s），自己切 LAND。
* `runs/v7_s4_g5_none_r1`：t=10 s 读数 0.167 m，t=18 s 模式变 9（LAND）。
* 飞控判“测距无效”的路径在 SITL 里是 EKF 方差/失效保护，与监视器的判据无关。

**(2) 读数仍在量程内的格，对照确实爬高，任务书的最坏情况被复现。**

* `runs/v7_s4_h_none_r1`（`h` 格，读数阶梯式变大）：真值 4.08 m 时测距读数 2.05 m、EKF 估计 1.99 m，
  飞控以为自己在 2 m，实际停在 4.08 m；对照组真值最高 4.023 m。
* `a_none` 真值最高 3.651 m（基线 2.056 m）。
* `i_none` 真值最高 3.226 m；`d3_gN_none` 3.990 m、`d3_h_none` 4.017 m、`d3_k_none` 3.983 m。

**(3) `i` 格（悬停时读数以 0.1 m/s 持续变小）三种动作的真值最高都超过 3 m。**

注入→动作延迟 8.1–10.3 s（中位 9.2–9.9 s）：偏置低通 `tau=30 s` 对 0.1 m/s 的斜坡有 ~3 m 的滞后，
残差 `e` 要到异常末段才越过 `E=0.7 m`，此时飞机已经跟着抬高了 1 m 以上。
`i_A1` 真值最高 3.083 m、`i_A2` 3.059 m、`i_A3` 3.411 m，对照 3.226 m —— 加了监视器也没有把这个格压到 3 m 以内。

**(4) A3 有几格任务没跑完。** A3 只切源组、不动飞行模式，但切到气压高度源后飞控自己触发
`EKF Failsafe: changed to Land Mode` / `Circle fence breached` 而降落：
`g3_A3` 1/5、`g5_A3` 3/5、`g10_A3` 3/5、`gN_A3` 3/5、`d3_gN_A3` 3/5、`d3_h_A3` 4/5 完成；
其余（`a_A3` 5/5、`c_A3` 5/5、`h_A3` 5/5、`i_A3` 5/5、`j1_A3` 5/5、`j2_A3` 5/5、`k_A3` 5/5、`d3_k_A3` 5/5）完成。

**(5) A1 的落点偏差明显大于 A2。** A1 不换高度源，直接在已经被测距污染的高度估计上切 LAND；
`a_A1` 落点偏差中位 3.274 m（1.312–3.635 m），`a_A2` 只有 0.058 m（0.010–0.106 m）。

**(6) 降落受控性。** 所有有监视器动作的飞行，触地速度中位 0.318–0.553 m/s，
最陡下降率中位 −0.56…−0.91 m/s，空中悬停时间中位 4.1–6.0 s；没有出现加速下降或长期悬空不落。
对照组里 `a_none` 的触地速度中位 0.854 m/s（0.447–0.909 m/s），比有监视器动作的组高。

**(7) 源切换时 EKF 高度估计的跳变。** `mon_ekf_jump` 的中位在 −0.09…+0.48 m，多数 < 0.1 m；
切源后 `baroHgtOffset` 跟随当前（可能被污染的）估计，所以切到气压源**不会自动纠正**已有的测距偏差——
这一点与源码核对一致（`AP_NavEKF3_PosVelFusion.cpp:1398-1402`、`AP_NavEKF3_Measurements.cpp:806-810`）。
