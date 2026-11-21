# V5 汇总表（自动生成，tables_v5.py）

中位数 [最小–最大]。每格 5 次。列：track RMS / 真值离航线最大距离 / 第 3 圈回角误差 / 落点偏差 /
EKF 位置单步跳变 / 真值速度峰值 / EKF 航向误差峰值 / 机载门限拒掉条数 / 新息门限拒掉条数 /
注入的错检位置条数 / 注入的错检航向条数 / 遮挡计时条数 / 其中标记本应可见的条数。

| 布置×故障 | n | 完成 | track RMS | truth max off track | lap3 err | land offset | EKF jump max | truth spd max | yaw err max | gated obs | innov rej | bad pos obs | bad yaw obs | block ticks | block ticks w/ marker | 结局 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| base_none | 5 | 5/5 | 0.166 [0.128–0.176] | 0.418 [0.272–0.474] | 0.413 [0.154–0.418] | 0.369 [0.251–0.380] | 0.023 [0.020–0.025] | 0.536 [0.511–0.560] | 2.400 [-1.400–2.900] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| base_sq4 | 5 | 5/5 | 0.063 [0.058–0.074] | 0.164 [0.155–0.207] | 0.099 [0.022–0.156] | 0.044 [0.004–0.083] | 0.043 [0.034–0.158] | 0.558 [0.502–0.603] | -0.800 [-5.600–3.900] | 38.000 [36.000–43.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| base_sq1 | 5 | 5/5 | 0.126 [0.116–0.149] | 0.340 [0.304–0.415] | 0.147 [0.092–0.168] | 0.042 [0.008–0.049] | 0.267 [0.130–0.294] | 0.574 [0.540–0.659] | -2.200 [-7.600–2.500] | 13.000 [10.000–15.000] | 3.000 [0.000–5.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_bad1m_lap1 | 5 | 5/5 | 0.064 [0.052–0.073] | 0.175 [0.118–0.255] | 0.129 [0.020–0.143] | 0.032 [0.016–0.047] | 0.057 [0.037–0.098] | 0.577 [0.488–0.634] | 0.900 [-5.600–3.800] | 63.000 [57.000–66.000] | 19.000 [17.000–25.000] | 21.000 [18.000–25.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_bad1m_lap2 | 5 | 5/5 | 0.063 [0.056–0.066] | 0.177 [0.138–0.192] | 0.114 [0.028–0.142] | 0.042 [0.021–0.074] | 0.056 [0.037–0.118] | 0.549 [0.535–0.663] | 0.900 [-5.600–3.700] | 53.000 [48.000–63.000] | 14.000 [12.000–16.000] | 14.000 [12.000–17.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_bad03_10s | 5 | 5/5 | 0.083 [0.079–0.094] | 0.316 [0.245–0.363] | 0.119 [0.019–0.159] | 0.039 [0.011–0.063] | 0.059 [0.042–0.139] | 0.540 [0.520–0.596] | -0.800 [-5.600–3.800] | 41.000 [39.000–46.000] | 2.000 [1.000–8.000] | 71.000 [67.000–72.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_yaw20_5s | 5 | 5/5 | 0.063 [0.055–0.067] | 0.155 [0.138–0.196] | 0.087 [0.040–0.130] | 0.045 [0.018–0.082] | 0.119 [0.045–0.155] | 0.539 [0.513–0.551] | 1.500 [-5.600–3.900] | 39.000 [36.000–44.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 34.000 [33.000–36.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_block30 | 5 | 5/5 | 0.104 [0.071–0.183] | 0.305 [0.169–0.529] | 0.132 [0.028–0.529] | 0.062 [0.011–0.172] | 0.121 [0.078–0.218] | 0.563 [0.526–0.673] | -4.300 [-5.600–1.900] | 23.000 [20.000–64.000] | 2.000 [0.000–44.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 250.000 [250.000–250.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_stop10 | 5 | 5/5 | 0.079 [0.072–0.094] | 0.228 [0.189–0.273] | 0.084 [0.064–0.240] | 0.034 [0.014–0.049] | 0.054 [0.040–0.107] | 0.511 [0.491–0.559] | 1.000 [-5.600–4.000] | 37.000 [32.000–38.000] | 1.000 [1.000–3.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_stop_norec | 5 | 5/5 | 0.120 [0.105–0.124] | 0.375 [0.249–0.415] | 0.365 [0.169–0.389] | 0.333 [0.131–0.408] | 0.037 [0.033–0.071] | 0.513 [0.481–0.554] | 1.500 [-1.500–3.900] | 22.000 [21.000–25.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq4_lat500_20s | 5 | 5/5 | 0.165 [0.150–0.240] | 0.622 [0.480–0.690] | 0.027 [0.003–0.070] | 0.046 [0.034–0.117] | 0.168 [0.086–0.186] | 0.727 [0.651–0.799] | 1.500 [-4.600–4.000] | 79.000 [66.000–136.000] | 43.000 [31.000–104.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_bad1m_lap1 | 5 | 5/5 | 0.116 [0.109–0.300] | 0.278 [0.258–0.604] | 0.143 [0.057–0.192] | 0.042 [0.018–0.059] | 0.223 [0.046–0.327] | 0.612 [0.574–0.761] | -1.500 [-7.300–4.300] | 38.000 [37.000–93.000] | 25.000 [24.000–83.000] | 25.000 [25.000–26.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_bad1m_lap2 | 5 | 5/5 | 0.130 [0.109–0.144] | 0.332 [0.260–0.392] | 0.134 [0.121–0.194] | 0.038 [0.012–0.057] | 0.099 [0.052–0.228] | 0.539 [0.535–0.623] | -2.400 [-8.100–2.000] | 21.000 [19.000–23.000] | 9.000 [8.000–9.000] | 9.000 [5.000–9.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_bad03_10s | 5 | 5/5 | 0.181 [0.161–0.197] | 0.490 [0.429–0.515] | 0.185 [0.090–0.485] | 0.061 [0.039–0.071] | 0.197 [0.102–0.230] | 0.576 [0.547–0.624] | -3.100 [-7.500–2.000] | 30.000 [18.000–70.000] | 21.000 [7.000–66.000] | 35.000 [35.000–37.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_yaw20_5s | 5 | 5/5 | 0.133 [0.108–0.177] | 0.350 [0.312–0.487] | 0.130 [0.115–0.184] | 0.023 [0.012–0.061] | 0.260 [0.133–0.313] | 0.584 [0.569–0.684] | 8.300 [-2.800–13.600] | 15.000 [12.000–17.000] | 5.000 [0.000–6.000] | 0.000 [0.000–0.000] | 24.000 [21.000–28.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_block30 | 5 | 5/5 | 0.152 [0.110–0.226] | 0.446 [0.317–0.561] | 0.140 [0.076–0.153] | 0.040 [0.025–0.079] | 0.221 [0.060–0.306] | 0.578 [0.540–0.701] | -2.900 [-8.300–2.700] | 11.000 [9.000–35.000] | 1.000 [0.000–29.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 250.000 [250.000–250.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_stop10 | 5 | 5/5 | 0.129 [0.092–0.187] | 0.371 [0.247–0.514] | 0.064 [0.034–0.166] | 0.053 [0.016–0.076] | 0.186 [0.100–0.197] | 0.590 [0.542–0.672] | 2.200 [-7.900–3.000] | 11.000 [8.000–35.000] | 3.000 [2.000–24.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_stop_norec | 5 | 5/5 | 0.165 [0.107–0.241] | 0.470 [0.357–0.588] | 0.442 [0.420–0.545] | 0.412 [0.312–0.455] | 0.157 [0.111–0.280] | 0.540 [0.518–0.615] | -2.800 [-7.700–2.400] | 5.000 [3.000–7.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| sq1_lat500_20s | 5 | 5/5 | 0.209 [0.169–0.239] | 0.579 [0.397–0.631] | 0.579 [0.138–0.641] | 0.505 [0.032–0.538] | 0.280 [0.104–0.348] | 0.661 [0.541–0.730] | -3.600 [-8.000–2.000] | 39.000 [30.000–49.000] | 32.000 [19.000–41.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | 0.000 [0.000–0.000] | no mode change / no failsafe |
| smk_sq4 | 1 | 1/1 | 0.063 | 0.162 | 0.142 | 0.022 | 0.101 | 0.519 | -4.600 | 41.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | no mode change / no failsafe |

## 每格逐次明细
### base_none

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| base_none_r1 | True | 0.167 | 0.474 | 0.418 | 0.375 | 0.024 (48.9) | 0.513 | 0 | 0 | 0 | New fence |
| base_none_r2 | True | 0.128 | 0.272 | 0.154 | 0.251 | 0.023 (23.1) | 0.536 | 0 | 0 | 0 | New fence |
| base_none_r3 | True | 0.142 | 0.309 | 0.184 | 0.32 | 0.022 (22.6) | 0.551 | 0 | 0 | 0 | New fence |
| base_none_r4 | True | 0.166 | 0.471 | 0.416 | 0.369 | 0.025 (48.9) | 0.511 | 0 | 0 | 0 | New fence |
| base_none_r5 | True | 0.176 | 0.418 | 0.413 | 0.38 | 0.02 (32.4) | 0.56 | 0 | 0 | 0 | New fence |

### base_sq4

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| base_sq4_r1 | True | 0.074 | 0.207 | 0.099 | 0.044 | 0.042 (48.4) | 0.549 | 42 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| base_sq4_r2 | True | 0.061 | 0.164 | 0.156 | 0.038 | 0.043 (68.8) | 0.558 | 37 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| base_sq4_r3 | True | 0.063 | 0.164 | 0.022 | 0.055 | 0.069 (10.5) | 0.603 | 38 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| base_sq4_r4 | True | 0.07 | 0.155 | 0.124 | 0.004 | 0.034 (27.0) | 0.577 | 36 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| base_sq4_r5 | True | 0.058 | 0.158 | 0.054 | 0.083 | 0.158 (9.2) | 0.502 | 43 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### base_sq1

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| base_sq1_r1 | True | 0.149 | 0.396 | 0.092 | 0.042 | 0.285 (27.4) | 0.604 | 13 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| base_sq1_r2 | True | 0.135 | 0.31 | 0.147 | 0.008 | 0.267 (26.8) | 0.659 | 14 | 3 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| base_sq1_r3 | True | 0.126 | 0.34 | 0.168 | 0.049 | 0.13 (26.9) | 0.54 | 10 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| base_sq1_r4 | True | 0.116 | 0.304 | 0.14 | 0.035 | 0.199 (26.7) | 0.574 | 13 | 3 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| base_sq1_r5 | True | 0.116 | 0.415 | 0.163 | 0.045 | 0.294 (26.9) | 0.54 | 15 | 5 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq4_bad1m_lap1

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_bad1m_lap1_r1 | True | 0.071 | 0.178 | 0.129 | 0.016 | 0.058 (22.9) | 0.577 | 66 | 25 | 25 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_bad1m_lap1_r2 | True | 0.052 | 0.118 | 0.107 | 0.047 | 0.049 (48.6) | 0.581 | 57 | 19 | 21 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_bad1m_lap1_r3 | True | 0.064 | 0.175 | 0.02 | 0.043 | 0.098 (10.6) | 0.541 | 63 | 21 | 24 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_bad1m_lap1_r4 | True | 0.073 | 0.255 | 0.136 | 0.029 | 0.057 (22.2) | 0.634 | 57 | 17 | 18 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_bad1m_lap1_r5 | True | 0.057 | 0.162 | 0.143 | 0.032 | 0.037 (63.8) | 0.488 | 64 | 19 | 20 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq4_bad1m_lap2

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_bad1m_lap2_r1 | True | 0.063 | 0.177 | 0.142 | 0.027 | 0.037 (58.4) | 0.535 | 55 | 14 | 14 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad1m_lap2_r2 | True | 0.058 | 0.192 | 0.029 | 0.072 | 0.115 (10.1) | 0.549 | 51 | 14 | 14 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad1m_lap2_r3 | True | 0.064 | 0.17 | 0.028 | 0.021 | 0.056 (10.3) | 0.557 | 53 | 15 | 15 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad1m_lap2_r4 | True | 0.066 | 0.184 | 0.114 | 0.074 | 0.118 (10.6) | 0.663 | 48 | 12 | 12 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad1m_lap2_r5 | True | 0.056 | 0.138 | 0.126 | 0.042 | 0.042 (68.3) | 0.536 | 63 | 16 | 17 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |

### sq4_bad03_10s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_bad03_10s_r1 | True | 0.079 | 0.245 | 0.078 | 0.029 | 0.042 (69.4) | 0.52 | 41 | 1 | 72 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad03_10s_r2 | True | 0.09 | 0.317 | 0.15 | 0.06 | 0.139 (10.0) | 0.56 | 46 | 8 | 71 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad03_10s_r3 | True | 0.079 | 0.287 | 0.019 | 0.063 | 0.059 (43.5) | 0.53 | 39 | 3 | 67 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad03_10s_r4 | True | 0.094 | 0.363 | 0.119 | 0.011 | 0.062 (10.1) | 0.54 | 39 | 2 | 71 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_bad03_10s_r5 | True | 0.083 | 0.316 | 0.159 | 0.039 | 0.047 (44.0) | 0.596 | 42 | 2 | 70 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq4_yaw20_5s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_yaw20_5s_r1 | True | 0.063 | 0.148 | 0.126 | 0.018 | 0.084 (9.8) | 0.546 | 44 | 0 | 33 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_yaw20_5s_r2 | True | 0.06 | 0.155 | 0.13 | 0.045 | 0.045 (69.0) | 0.536 | 37 | 0 | 36 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_yaw20_5s_r3 | True | 0.067 | 0.186 | 0.04 | 0.056 | 0.119 (10.1) | 0.551 | 39 | 0 | 35 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_yaw20_5s_r4 | True | 0.067 | 0.196 | 0.087 | 0.082 | 0.155 (10.2) | 0.539 | 36 | 0 | 33 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_yaw20_5s_r5 | True | 0.055 | 0.138 | 0.063 | 0.044 | 0.133 (9.7) | 0.513 | 43 | 0 | 34 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq4_block30

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_block30_r1 | True | 0.094 | 0.286 | 0.147 | 0.062 | 0.078 (10.3) | 0.563 | 23 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_block30_r2 | True | 0.071 | 0.169 | 0.028 | 0.066 | 0.187 (10.5) | 0.539 | 22 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_block30_r3 | True | 0.104 | 0.305 | 0.038 | 0.011 | 0.098 (10.1) | 0.673 | 20 | 2 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_block30_r4 | True | 0.137 | 0.429 | 0.132 | 0.027 | 0.218 (10.8) | 0.646 | 27 | 6 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_block30_r5 | True | 0.183 | 0.529 | 0.529 | 0.172 | 0.121 (10.1) | 0.526 | 64 | 44 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq4_stop10

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_stop10_r1 | True | 0.072 | 0.231 | 0.084 | 0.043 | 0.051 (64.0) | 0.511 | 38 | 3 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_stop10_r2 | True | 0.087 | 0.218 | 0.129 | 0.014 | 0.074 (63.5) | 0.559 | 32 | 1 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_stop10_r3 | True | 0.094 | 0.273 | 0.24 | 0.034 | 0.107 (10.0) | 0.523 | 35 | 3 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_stop10_r4 | True | 0.079 | 0.189 | 0.064 | 0.049 | 0.04 (63.5) | 0.491 | 37 | 1 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_stop10_r5 | True | 0.079 | 0.228 | 0.069 | 0.029 | 0.054 (63.5) | 0.504 | 38 | 1 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq4_stop_norec

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_stop_norec_r1 | True | 0.121 | 0.366 | 0.365 | 0.333 | 0.033 (37.6) | 0.501 | 22 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_stop_norec_r2 | True | 0.105 | 0.249 | 0.169 | 0.131 | 0.071 (9.8) | 0.554 | 21 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_stop_norec_r3 | True | 0.124 | 0.415 | 0.389 | 0.386 | 0.054 (21.4) | 0.513 | 21 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_stop_norec_r4 | True | 0.12 | 0.375 | 0.346 | 0.329 | 0.036 (32.2) | 0.548 | 22 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_stop_norec_r5 | True | 0.12 | 0.402 | 0.389 | 0.408 | 0.037 (31.7) | 0.481 | 25 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq4_lat500_20s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq4_lat500_20s_r1 | True | 0.156 | 0.59 | 0.041 | 0.043 | 0.09 (49.6) | 0.727 | 79 | 43 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq4_lat500_20s_r2 | True | 0.15 | 0.48 | 0.027 | 0.034 | 0.086 (48.8) | 0.659 | 66 | 34 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq4_lat500_20s_r3 | True | 0.24 | 0.69 | 0.007 | 0.069 | 0.168 (64.4) | 0.651 | 136 | 104 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_lat500_20s_r4 | True | 0.237 | 0.647 | 0.07 | 0.117 | 0.186 (10.3) | 0.742 | 133 | 103 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq4_lat500_20s_r5 | True | 0.165 | 0.622 | 0.003 | 0.046 | 0.181 (10.3) | 0.799 | 69 | 31 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq1_bad1m_lap1

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_bad1m_lap1_r1 | True | 0.116 | 0.294 | 0.143 | 0.053 | 0.223 (26.7) | 0.574 | 37 | 24 | 25 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_bad1m_lap1_r2 | True | 0.109 | 0.265 | 0.191 | 0.042 | 0.046 (26.9) | 0.612 | 38 | 24 | 25 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap1_r3 | True | 0.111 | 0.278 | 0.133 | 0.018 | 0.276 (27.2) | 0.761 | 37 | 29 | 26 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap1_r4 | True | 0.122 | 0.258 | 0.057 | 0.059 | 0.194 (27.2) | 0.59 | 38 | 25 | 25 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_bad1m_lap1_r5 | True | 0.3 | 0.604 | 0.192 | 0.039 | 0.327 (71.3) | 0.653 | 93 | 83 | 25 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq1_bad1m_lap2

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_bad1m_lap2_r1 | True | 0.14 | 0.348 | 0.121 | 0.038 | 0.099 (26.8) | 0.539 | 23 | 9 | 9 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap2_r2 | True | 0.13 | 0.295 | 0.164 | 0.057 | 0.052 (26.9) | 0.623 | 22 | 9 | 9 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap2_r3 | True | 0.144 | 0.392 | 0.194 | 0.057 | 0.133 (26.9) | 0.535 | 19 | 9 | 8 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap2_r4 | True | 0.109 | 0.26 | 0.128 | 0.012 | 0.09 (26.9) | 0.535 | 20 | 8 | 9 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad1m_lap2_r5 | True | 0.126 | 0.332 | 0.134 | 0.027 | 0.228 (26.9) | 0.559 | 21 | 9 | 5 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq1_bad03_10s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_bad03_10s_r1 | True | 0.191 | 0.429 | 0.14 | 0.039 | 0.102 (27.0) | 0.547 | 18 | 7 | 35 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq1_bad03_10s_r2 | True | 0.17 | 0.48 | 0.208 | 0.071 | 0.197 (27.4) | 0.576 | 26 | 16 | 37 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad03_10s_r3 | True | 0.197 | 0.49 | 0.485 | 0.047 | 0.23 (27.0) | 0.624 | 70 | 66 | 35 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_bad03_10s_r4 | True | 0.181 | 0.515 | 0.09 | 0.061 | 0.215 (27.4) | 0.576 | 48 | 40 | 35 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_bad03_10s_r5 | True | 0.161 | 0.5 | 0.185 | 0.069 | 0.126 (27.2) | 0.558 | 30 | 21 | 36 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq1_yaw20_5s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_yaw20_5s_r1 | True | 0.137 | 0.35 | 0.128 | 0.027 | 0.282 (27.4) | 0.584 | 16 | 5 | 25 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_yaw20_5s_r2 | True | 0.108 | 0.318 | 0.153 | 0.016 | 0.26 (26.8) | 0.573 | 15 | 3 | 21 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_yaw20_5s_r3 | True | 0.177 | 0.487 | 0.184 | 0.061 | 0.133 (26.9) | 0.595 | 13 | 5 | 24 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_yaw20_5s_r4 | True | 0.119 | 0.312 | 0.115 | 0.012 | 0.135 (27.0) | 0.569 | 12 | 0 | 28 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU1 yaw aligned |
| sq1_yaw20_5s_r5 | True | 0.133 | 0.43 | 0.13 | 0.023 | 0.313 (27.1) | 0.684 | 17 | 6 | 23 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq1_block30

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_block30_r1 | True | 0.151 | 0.35 | 0.149 | 0.079 | 0.221 (27.2) | 0.59 | 9 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_block30_r2 | True | 0.11 | 0.317 | 0.076 | 0.037 | 0.06 (27.3) | 0.54 | 11 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_block30_r3 | True | 0.226 | 0.561 | 0.153 | 0.04 | 0.306 (27.0) | 0.578 | 35 | 29 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_block30_r4 | True | 0.158 | 0.459 | 0.14 | 0.025 | 0.086 (26.8) | 0.541 | 10 | 1 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_block30_r5 | True | 0.152 | 0.446 | 0.081 | 0.053 | 0.222 (27.4) | 0.701 | 16 | 7 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq1_stop10

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_stop10_r1 | True | 0.187 | 0.514 | 0.166 | 0.025 | 0.1 (26.9) | 0.542 | 9 | 2 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_stop10_r2 | True | 0.173 | 0.371 | 0.055 | 0.061 | 0.128 (27.0) | 0.672 | 35 | 24 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_stop10_r3 | True | 0.111 | 0.248 | 0.064 | 0.076 | 0.186 (26.9) | 0.548 | 8 | 2 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_stop10_r4 | True | 0.092 | 0.247 | 0.121 | 0.016 | 0.197 (27.1) | 0.59 | 11 | 3 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_stop10_r5 | True | 0.129 | 0.433 | 0.034 | 0.053 | 0.192 (27.3) | 0.63 | 18 | 7 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |

### sq1_stop_norec

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_stop_norec_r1 | True | 0.187 | 0.548 | 0.545 | 0.412 | 0.111 (26.6) | 0.518 | 7 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_stop_norec_r2 | True | 0.163 | 0.422 | 0.421 | 0.312 | 0.157 (26.8) | 0.52 | 5 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_stop_norec_r3 | True | 0.241 | 0.588 | 0.442 | 0.439 | 0.28 (26.8) | 0.59 | 4 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_stop_norec_r4 | True | 0.107 | 0.357 | 0.453 | 0.455 | 0.196 (27.3) | 0.615 | 3 | 0 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_stop_norec_r5 | True | 0.165 | 0.47 | 0.42 | 0.357 | 0.155 (26.9) | 0.54 | 6 | 0 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |

### sq1_lat500_20s

| run | 完成 | track RMS | track max | lap3 | 落点 | EKF jump (t) | spd max | gated | innov rej | bad obs | 结局/消息 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sq1_lat500_20s_r1 | True | 0.209 | 0.579 | 0.579 | 0.505 | 0.244 (26.9) | 0.552 | 42 | 33 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_lat500_20s_r2 | True | 0.169 | 0.397 | 0.197 | 0.032 | 0.317 (27.5) | 0.661 | 30 | 19 | 0 | New fence; EKF3 IMU1 yaw aligned; EKF3 lane switch 1 |
| sq1_lat500_20s_r3 | True | 0.239 | 0.631 | 0.641 | 0.534 | 0.348 (26.8) | 0.72 | 39 | 32 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_lat500_20s_r4 | True | 0.191 | 0.516 | 0.138 | 0.048 | 0.104 (71.9) | 0.541 | 36 | 26 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
| sq1_lat500_20s_r5 | True | 0.235 | 0.63 | 0.637 | 0.538 | 0.28 (26.8) | 0.73 | 49 | 41 | 0 | New fence; EKF3 IMU0 yaw aligned; EKF3 IMU0 is using external nav data |
