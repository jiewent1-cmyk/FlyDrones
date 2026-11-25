# 系统图册（中文）

本页是 `ecps295/` 各文档里 Mermaid 图的中文版，图的结构与原图一致。
原图和详细说明见各文档；数据图见 [FIGURES.md](FIGURES.md)。

## 1. 系统总览

### 系统总览：真机部件连接与仿真替代

传感器、Orange Pi 机载电脑、飞控和地面控制台之间的连接。虚线表示仿真中由哪些部分替代真机部件。

```mermaid
flowchart LR
  subgraph SENS["传感器"]
    CAM["USB UVC 相机<br/>640x480, 30 fps"]
    TOF["2 × VL53L7CX 8×8<br/>前 + 后, 15 Hz"]
    DOWN["Matek 3901-L0X<br/>光流 + 下视 ToF"]
  end
  subgraph OPI["Orange Pi Zero 3W"]
    COMP["ecps_companion.py<br/>20 Hz：视网膜、MiniFly、解码器、<br/>局部地图、安全护盾、限幅器"]
    MR["mavlink-router"]
  end
  FC["Matek F405-WMO<br/>ArduCopter 4.7, EKF3"]
  RC["ELRS 接收机<br/>飞手 RC 遥控，模式开关"]
  MAC["Mac 地面控制台<br/>ECPS Console.app"]
  subgraph SIMS["仿真替代部分"]
    GZ["Gazebo Harmonic<br/>物理、鱼眼相机、下视 ToF"]
    SITL["ArduPilot SITL<br/>Copter-4.7.0"]
    RG4["PC 上的 run_g4.py<br/>相同的控制代码"]
  end
  CAM -->|"USB"| COMP
  TOF -->|"I²C TWI0 / TWI1"| COMP
  DOWN -->|"MSP, UART4"| FC
  COMP <-->|"UDP 14560"| MR
  MR <-->|"MAVLink2, UART3"| FC
  RC -->|"CRSF"| FC
  MAC <-->|"ECL1, UDP 14700"| COMP
  MAC <-->|"MAVLink, UDP 14550"| MR
  GZ <-->|"JSON 锁步"| SITL
  GZ -->|"相机画面经 cam_bridge.py"| RG4
  RG4 <-->|"MAVLink，SIM_STATE 用于 8×8 ToF 射线投射"| SITL
  SITL -.->|"替代"| FC
  RG4 -.->|"替代"| COMP
```

*原图：[README.md § At a glance · 概览](../README.md)*

### MiniFly 闭环数据通路

鱼眼相机、ToF 与气压计的数据经 `EcpsRetina` 进入 MiniFly 连接组脑模型，再经解码器、`EcpsPilot` 和安全护盾向 ArduPilot SITL 发速度指令。前后两块 8×8 ToF 同时用于建立局部三维地图，供安全护盾使用。

```mermaid
flowchart LR
  CAM["鱼眼相机"] --> RET["EcpsRetina<br/>光流 · 逼近膨胀 · 空白 · 近距 · 腹侧"]
  TOF["ToF + 气压计"] --> RET
  R8["2 × VL53L7CX 8×8<br/>前 + 后"] --> MAP["局部三维地图"]
  R8 -->|front zones| RET
  MAP --> SH["安全护盾（CBF）"]
  RET --> MF["MiniFly LIF 连接组<br/>+ LCb + LCn + LCv→MDN"]
  MF --> DEC["EcpsDecoder<br/>扫视 · 谨慎 · 后撤 · 传出副本"]
  DEC --> PIL["EcpsPilot<br/>看门狗 · 围栏掉头"]
  PIL --> SH
  SH -->|GUIDED velocity| AP["ArduPilot 4.7 SITL"]
  PIL -->|route B: height above floor| AP
  AP <-->|JSON lockstep| GZ["Gazebo Harmonic"]
```

*原图：[README.md § At a glance · 概览](../README.md)*

### 系统分层架构

分层原则是“脑模型做决定，安全护盾做约束，飞控做增稳”。每一层只能让它下面的指令更保守，下层不需要上层也能保证无人机安全。

```mermaid
flowchart TB
  subgraph SENSE["传感层"]
    direction LR
    CAM["USB 相机<br/>120° 鱼眼，下俯 12°"]
    TOF8["2 × VL53L7CX 8×8 ToF<br/>前 + 后, 60°×60°"]
    FLOW["Matek 3901-L0X<br/>PMW3901 光流 + VL53L0X 下视 ToF"]
    IMU["飞控 IMU · 气压计 · 罗盘"]
  end

  subgraph PERC["感知层 — EcpsRetina（Orange Pi）"]
    direction LR
    RET["每眼 6×8 网格：<br/>光流 FtB/BtF/上/下 · 逼近膨胀 · 空白 · 近距"]
    VEN["VentralCue<br/>表面高度 T = baro − ToF"]
    MAP["局部三维占据地图<br/>5 cm 体素，log-odds, τ = 10 s"]
  end

  subgraph MODEL["模型层 — MiniFly v4 连接组（Orange Pi）"]
    direction LR
    ENC["InputEncoder<br/>特征 → Poisson 发放率"]
    LIF["LIF 网络 ≈ 1,000 个神经元<br/>T4/T5 · HS/VS · LPLC2/LC4 · LCb · LCn · LCv"]
    DN["下行神经元<br/>DNg02 · DNp03 · DNp01 · MDN"]
  end

  subgraph DEC["决策层 — EcpsDecoder（Orange Pi）"]
    D1["读出 + 状态机<br/>巡航 · 扫视 · 谨慎 · 后撤 · 传出副本"]
  end

  subgraph SAFE["安全层（Orange Pi）"]
    direction LR
    GOV["SafetyGovernor<br/>限幅 · 变化率限制 · 升限 · 地理围栏 · 电池 · 脑模型看门狗"]
    PIL["EcpsPilot 看门狗<br/>相机数据过期 · 高度源过期 · 围栏掉头"]
    SH["RangeShield (CBF)<br/>v·uᵢ ≤ α(dᵢ − d_safe)"]
  end

  subgraph FC["飞行控制层 — Matek F405-WMO 上的 ArduCopter 4.7"]
    direction LR
    EKF["EKF3<br/>光流 + ToF + ExtNav z + IMU"]
    GUID["GUIDED 速度控制器"]
    ATT["姿态 / 角速率环 → 电机混控"]
    FS["失效保护<br/>RC · 机载电脑（GCS）· EKF · GUIDED 超时"]
  end

  CAM --> RET
  TOF8 -->|front zones| RET
  TOF8 --> MAP
  FLOW --> VEN
  IMU --> VEN
  VEN --> RET
  RET --> ENC --> LIF --> DN --> D1
  D1 --> PIL --> SH --> GOV
  MAP --> SH
  GOV -->|"MAVLink：机体系速度，20 Hz"| GUID
  VEN -->|"MAVLink：路线 B 离地高度"| EKF
  FLOW --> EKF
  IMU --> EKF
  EKF --> GUID --> ATT
  FS -.-> ATT
```

*原图：[ARCHITECTURE.md § 1. System overview: the layers](ARCHITECTURE.md)*

### MiniFly 累积神经回路（v0 → v4）

颜色表示引入各条通路的代次：蓝色为上游 v0，橙色为 v2（LCb），绿色为 v3（LCn），紫色为 v4（LCv → MDN），黄色为解码器读取的下行神经元。图中只画一侧，每个神经元群都有左右两份；LCn 和 LCb 经 PVLP_inh 抑制对侧 DNp03，因此扫视会转离较近或空白的一侧。

```mermaid
flowchart LR
  subgraph IN["视网膜特征"]
    direction TB
    fFlow["光流 FtB / BtF / 上 / 下"]
    fLoom["逼近膨胀"]
    fBlank["空白<br/>（中央纹理缺失）"]
    fNear["近距<br/>（侧向光流不对称；<br/>S5：+ 前向 ToF 接近度）"]
    fVen["腹侧<br/>（无人机下方表面升高）"]
    fHal["平衡棒 = 陀螺仪偏航角速率"]
  end

  T4["T4a–d / R1-R6"]:::v0
  LPi["LPi_h / LPi_v"]:::v0
  HS["HS"]:::v0
  VS["VS"]:::v0
  LAL["LAL_inh"]:::v0
  LPLC2["LPLC2 / LC4"]:::v0
  PVLP["PVLP"]:::v0
  PINH["PVLP_inh<br/>（左右竞争）"]:::v0
  HAL["平衡棒"]:::v0
  LCb["LCb ×24"]:::v2
  LCn["LCn ×24<br/>（→ 对侧 HS）"]:::v3
  LCv["LCv ×16"]:::v4

  DNg02(["DNg02<br/>油门 / 偏航"]):::dn
  DNp03(["DNp03<br/>扫视 + 刹车"]):::dn
  DNp01(["DNp01 巨纤维<br/>逃逸"]):::dn
  MDN(["MDN ×2<br/>后撤"]):::v4

  fFlow --> T4 --> HS & VS
  T4 --> LPi --> HS & VS
  HS --> DNg02
  HS --> LAL --> DNg02
  VS --> DNg02
  fHal --> HAL --> DNg02
  fLoom --> LPLC2 --> PVLP --> DNp03
  LPLC2 --> DNp01
  DNp03 --> PINH --> PVLP
  fBlank --> LCb --> PVLP
  LCb --> DNp01
  LCb --> PINH
  fNear --> LCn --> HS
  LCn --> PINH
  fVen --> LCv --> MDN

  classDef v0 fill:#e8eef7,stroke:#4a6fa5,color:#1b2b44
  classDef v2 fill:#fde7d9,stroke:#d9763b,color:#4a2410
  classDef v3 fill:#e3f4e1,stroke:#4c9a45,color:#173a14
  classDef v4 fill:#efe3f6,stroke:#8a54a8,color:#341a44
  classDef dn fill:#fff6cc,stroke:#b39500,color:#3d3300
```

*原图：[ARCHITECTURE.md § 3.1 The cumulative circuit](ARCHITECTURE.md)*

### 系统代次演进：从 GPS 笼到最终硬件

脑模型只是系统的一部分，每一代系统还改动了传感、状态估计和安全层。连线上的文字是推动进入下一代的原因。

```mermaid
flowchart LR
  A["<b>Gen A — GPS 笼</b><br/>v0 → v3_2<br/>仅相机<br/>GPS 定位<br/>SafetyGovernor"]
  B["<b>Gen B — 光流导航</b><br/>v4 + 路线 B<br/>相机 + 下视 ToF<br/>EKF3 用光流，ExtNav z<br/>+ 高度 / 机载电脑看门狗"]
  C["<b>Gen C — 硬件对齐</b><br/>v4_eff<br/>传感器相同<br/>经标定的孪生，CMA h2_*<br/>+ C1 传出副本"]
  D["<b>Gen D — 测距安全护盾</b><br/>s5：5 × 单区 ToF<br/>ToF → 脑模型（LCn）<br/>+ RangeShield"]
  E["<b>Gen E — 最终版</b><br/>w2map：前 + 后 VL53L7CX 8×8<br/>局部三维地图 → 安全护盾<br/>前向区 → 脑模型"]
  A -->|"真机没有 GPS；<br/>箱子会把飞控高度拉低"| B
  B -->|"GPS 笼 RL 解码器<br/>在真实传感器配置下失败"| C
  C -->|"lowbox 接触对解码器<br/>不敏感：存在盲区"| D
  D -->|"更便宜、更易安装的布局；<br/>记住见过的障碍物"| E
```

*原图：[ARCHITECTURE.md § 4. System generations: from the GPS cage to the final hardware](ARCHITECTURE.md)*

### 最终系统（Gen E，w2map）

前后两块 8×8 ToF 建立局部地图，取机体高度附近的切片作为 `RangeShield` 的虚拟波束，前向区同时进入脑模型。地图建在光流 EKF 坐标系中，EKF 漂移是主要的剩余风险。

```mermaid
flowchart LR
  subgraph S["传感器"]
    CAM["USB 相机"]
    F8["前向 VL53L7CX<br/>64 区"]
    R8["后向 VL53L7CX<br/>64 区"]
    DT["3901-L0X 下视 ToF<br/>（经飞控遥测）"]
    BARO["气压计（经飞控）"]
  end
  subgraph OPI["Orange Pi Zero 3W"]
    RET["EcpsRetina"]
    VC["VentralCue"]
    MAP["LocalMap<br/>128 区/帧 → 5 cm 体素"]
    SL["机体高度处切片 ± 10 cm<br/>→ 36 × 10° 扇区内 ≤ 1 m 的最近障碍物"]
    BR["MiniFly v4 + C1"]
    DE["EcpsDecoder"]
    SG["SafetyGovernor + 看门狗"]
    SH["RangeShield<br/>36 条虚拟波束，d_safe 0.25 m, α 1/s"]
  end
  FCN["ArduPilot GUIDED"]
  CAM --> RET
  F8 -->|"前向区 → 近距（LCn）"| RET
  F8 --> MAP
  R8 --> MAP
  DT --> VC
  BARO --> VC
  VC --> RET
  RET --> BR --> DE --> SH --> SG
  MAP --> SL --> SH
  SG -->|velocity| FCN
  VC -->|route-B height| FCN
```

*原图：[ARCHITECTURE.md § 4.1 Final system (Gen E, w2map)](ARCHITECTURE.md)*

### ecps_*.py 模块导入关系

实线表示导入，虚线表示继承上游类。`run_g4.py`（Gazebo）、`ecps_companion.py`（Orange Pi）和快速孪生的 `rl/episode.py` 这三个运行器构建同一个 pilot。

```mermaid
flowchart LR
  RG4["run_g4.py<br/>Gazebo 运行器"]
  COMP["ecps_companion.py + real_drone.py<br/>Orange Pi 运行时"]
  EPI["rl/episode.py + rl/twin.py<br/>快速孪生"]
  PIL["ecps_pilot.py"]
  RET["ecps_retina.py"]
  DEC["ecps_decoder.py"]
  STR["ecps_struct.py"]
  MAP["ecps_map.py"]
  VEN["ecps_ventral.py"]
  TGT["ecps_target.py"]
  LMK["ecps_landmark.py"]
  GPS["ecps_gps_anchor.py"]
  MT["mavlink_twin.py"]
  MB["mb.py"]
  UP["上游 flydrones<br/>Pilot, Retina, MotorDecoder, MavlinkDrone"]
  RG4 --> PIL & RET & DEC & LMK
  RG4 -->|"经 gz_camera_drone.py"| MT
  COMP --> PIL & RET & DEC & LMK & GPS & TGT
  COMP -->|"RealDrone"| MT
  EPI --> PIL & RET & DEC & MB
  EPI --> MAP
  PIL --> STR & TGT & LMK & GPS
  STR --> MAP
  RET --> VEN
  MT --> VEN & MAP
  PIL -.->|"子类"| UP
  RET -.->|"子类"| UP
  DEC -.->|"子类"| UP
  MT -.->|"子类"| UP
```

*原图：[ARCHITECTURE.md § 11. Code map](ARCHITECTURE.md)*

## 2. 控制流程与安全

### 一个 20 Hz 控制周期

按 `EcpsPilot.tick` 的执行顺序画出，并标有传感器和输出的频率。每一级只能替换或削弱前一级的指令；限幅器之后的环节无法撤销限幅或停止。

```mermaid
flowchart TD
  CAM["相机帧，192×144 灰度<br/>UVC 30 fps，Gazebo 60 Hz"]
  TEL["飞控遥测，20 Hz<br/>位姿、姿态、气压计、RANGEFINDER"]
  TOF["2 × 8×8 ToF 帧，15 Hz"]
  RET["EcpsRetina.encode<br/>每眼 6×8 网格：光流、逼近膨胀、空白、近距、腹侧<br/>+ 上一周期的偏航角速率和前进指令"]
  ENC["InputEncoder<br/>特征转为 Poisson 发放率，平衡棒 = 偏航角速率"]
  BR["MiniFly brain.tick<br/>LIF，仿真一个周期的时间"]
  DEC["EcpsDecoder.update<br/>DN 发放率转为 FlightCommand<br/>扫视、谨慎、后撤、传出副本"]
  OP["控制台 Operator.apply，可选<br/>HOLD、GOTO、MANUAL 替换原始指令"]
  LAY["EcpsPilot 转向层<br/>高度守卫、RTL、目标、航点、围栏转向"]
  SH["StructGuard + RangeShield<br/>LocalMap 扇区、脱困转向"]
  GOV["SafetyGovernor.filter<br/>限幅、变化率限制、升限、下限、地理围栏、脑模型数据时效"]
  WD["EcpsPilot 看门狗覆盖<br/>相机或高度源过期：悬停<br/>land_requested：LAND"]
  GATE["drone.send<br/>RealDrone：接管门限 + 传感器守卫"]
  HGT["路线 B 高度，VentralCue<br/>VISION_POSITION_ESTIMATE z, 20 Hz"]
  FC["ArduPilot GUIDED"]
  CAM --> RET
  TEL --> RET
  TOF -->|"前向区，S5"| RET
  RET --> ENC --> BR -->|"每组 DN 的平均 Hz"| DEC
  DEC --> OP --> LAY --> SH --> GOV --> WD --> GATE
  TOF --> SH
  TEL --> HGT
  GATE -->|"SET_POSITION_TARGET_LOCAL_NED<br/>机体系速度 + 偏航角速率，20 Hz"| FC
  HGT -->|"由遥测数据发送，每周期一次"| FC
```

*原图：[ARCHITECTURE.md § 2. One control tick](ARCHITECTURE.md)*

### 解码器逃逸行为状态机

图中数值取自最终硬件配置 `minifly/s7_w2map.yaml`。巨纤维和 DNp03 触发用的是经低通滤波的发放率；由刹车触发的扫视在上一次扫视后的 1 s 内被屏蔽。

```mermaid
stateDiagram-v2
  state "稳定阶段<br/>悬停，测量静息 DN 发放率" as Settle
  state "巡航<br/>前进 = 巡航速度，按刹车记忆削减" as Cruise
  state "扫视<br/>转离 1.5 s，前 0.6 s 以 0.3 后退" as Sacc
  state "谨慎<br/>2 s 内不巡航，随后 2 s 内逐步恢复" as Caut
  state "后撤<br/>以 0.5 后退，不偏航" as Ret
  [*] --> Settle
  Settle --> Cruise : settle_s 时间已到
  Cruise --> Sacc : DNp03 超过 15 Hz、DNp01 超过 20 Hz，或刹车量至少 0.6
  Caut --> Sacc : 新的触发
  Sacc --> Caut : 扫视结束
  Caut --> Cruise : 恢复完成
  Cruise --> Ret : MDN 超过 20 Hz
  Caut --> Ret : MDN 超过 20 Hz
  Sacc --> Ret : MDN 超过 20 Hz，覆盖扫视
  Ret --> Sacc : 至少 1.5 s 后 MDN 静默 1 s，或达到 2 s 上限
```

*原图：[ARCHITECTURE.md § 3.3 Decoder behaviour states](ARCHITECTURE.md)*

### 纵深防御五层

从脑模型自身的行为，到指令限幅、监督看门狗、几何安全护盾，再到飞控失效保护。各层的触发条件、动作和验证方式见原文表格。

```mermaid
flowchart TB
  L1["<b>L1 行为</b> — 脑模型自身<br/>扫视、刹车、后撤、居中、传出副本"]
  L2["<b>L2 指令限制</b> — SafetyGovernor<br/>限幅、变化率限制、升限渐减、地理围栏、电池、脑模型时效看门狗"]
  L3["<b>L3 监督看门狗</b> — EcpsPilot<br/>相机数据过期、路线 B 高度过期、高度守卫、围栏掉头"]
  L4["<b>L4 几何安全护盾</b> — RangeShield + 局部地图<br/>将速度投影到安全半平面、脱困"]
  L5["<b>L5 飞控失效保护</b> — ArduPilot<br/>机载电脑心跳丢失 → 降落，SITL 中 2 s、真实飞控上 5 s，GUIDED 超时，RC 丢失 → 降落，EKF 失效保护，坠机检查"]
  L1 --> L2 --> L3 --> L4 --> L5
```

*原图：[ARCHITECTURE.md § 5.1 Defence in depth](ARCHITECTURE.md)*

### 传感器故障时机载电脑的处置

Orange Pi 上针对各类传感器故障的判断与动作。机载电脑的 LAND 只在它掌握控制权时发出（高度守卫的 LAND 按图中标注，无论是否掌控都会发出）；否则由飞手飞行。

```mermaid
flowchart LR
  CAM["相机：没有新帧"] --> CW["EcpsPilot 相机看门狗<br/>超过 0.3 s：悬停<br/>超过 3 s：LAND"]
  TOF["VL53L7CX 帧超过 0.3 s<br/>或有效区少于 8 个"] --> TG["RealDrone 传感器守卫<br/>悬停，2 s 后 LAND"]
  POSE["LOCAL_POSITION_NED 或 ATTITUDE<br/>超过 0.5 s 或无效"] --> TG
  RNG["RANGEFINDER 超过 0.5 s"] --> FB["路线 B 高度改用<br/>气压计减地面基线"]
  FB -->|"气压计数据也过期"| NOH["不发送路线 B 高度"]
  NOH --> HW["EcpsPilot 高度源看门狗<br/>超过 0.5 s：悬停<br/>超过 3 s：LAND"]
  NOH --> EKF["飞控：ExtNav 超时，<br/>EKF 高度回退到气压计"]
  HG["高度守卫，默认关闭<br/>气压计与测距读数不一致"] --> HGA["EKF 源组 3，气压计高度<br/>0.5 s 后 LAND，无论是否掌控"]
  GATE["接管门限<br/>相机、ToF 或位姿数据不新鲜"] --> REF["不接管控制权"]
  CW --> LAND["机载电脑发送 LAND"]
  TG --> LAND
  HW --> LAND
```

*原图：[ARCHITECTURE.md § 5.1 Defence in depth](ARCHITECTURE.md)*

### 链路与机载电脑故障处置

当链路或机载电脑本身出故障时，先由控制台层处理，再由飞控自身的失效保护接手。

```mermaid
flowchart TD
  CON["控制台链路丢失<br/>1.5 s 内没有 HELLO"] --> C1["GOTO、MANUAL、TRACK 切到 HOLD<br/>PATROL 继续"]
  C1 -->|"rtl 层开启，保持 5 s"| RTL["RTL：飞回起点，稳定 2 s，LAND"]
  C1 -->|"rtl 层关闭，保持 20 s"| CL["原地 LAND"]
  FCL["飞控心跳超过 2 s"] --> REL["机载电脑交出控制权<br/>不发速度指令"]
  MODE["飞手用 RC 切出 GUIDED"] --> REL
  BRN["脑模型周期超过 0.5 s 未更新"] --> BH["SafetyGovernor：悬停"]
  LOOP["控制循环停止发送"] --> GT["飞控：GUID_TIMEOUT，停止并悬停"]
  DEAD["机载电脑进程结束或 Pi 死机"] -->|"sysid 254 无 HEARTBEAT"| FSG["飞控：FS_GCS_ENABLE 5, LAND<br/>FS_GCS_TIMEOUT 2 s SITL, 5 s fc_flight.parm"]
  RCL["RC 信号丢失"] --> FST["飞控：FS_THR_ENABLE 3, LAND"]
```

*原图：[ARCHITECTURE.md § 5.1 Defence in depth](ARCHITECTURE.md)*

### 机载电脑视角的飞行模式与行为状态

包括脑模型之上的控制台模式、飞手接管交接和两级急停；机载电脑从不解锁或起飞。控制台指令可在 InControl 内的模式间直接切换（图中只画出以 HOLD 为中心的转移）；飞控确认急停后 RC 仍然优先，锁存会让脑模型无法接管，直到 `estop_reset`（飞控处于解锁状态时会被拒绝，除非强制）。

```mermaid
stateDiagram-v2
  state "在地面，已上锁" as Ground
  state "飞手用 RC 飞行<br/>LOITER 或 ALT_HOLD" as Pilot
  state "已请求 LAND<br/>飞控执行降落" as Land
  state "一级急停已锁存<br/>向飞控发 LAND 直到确认" as Es1
  state "二级急停已锁存<br/>强制上锁直到确认" as Es2
  state InControl {
    state "HOLD<br/>悬停，安全护盾仍生效" as Hold
    state "PATROL<br/>脑模型巡航" as Patrol
    state "GOTO 或航线<br/>规划路径，航点层" as Goto
    state "TRACK<br/>搜索、接近、面向标记" as Track
    state "MANUAL<br/>控制台摇杆" as Sticks
    state "RTL<br/>先 RETURN 再 SETTLE 2 s" as Rtl
    [*] --> Hold : 接管模式 hold，默认
    [*] --> Patrol : 接管模式 patrol，或没有控制台
    Hold --> Patrol : patrol
    Hold --> Goto : goto 或 route
    Hold --> Track : track
    Hold --> Sticks : manual 开启
    Hold --> Rtl : rtl，或控制台丢失 5 s
    Patrol --> Rtl : 电量低于 rtl.battery_pct
    Goto --> Hold : 已到达、航线完成、超时、链路丢失
    Track --> Hold : 链路丢失
    Sticks --> Hold : manual 关闭、链路丢失
  }
  Ground --> Pilot : 飞手解锁并起飞
  Pilot --> InControl : GUIDED，已解锁，离地 0.3 m，传感器数据新鲜
  InControl --> Pilot : 切换模式，或飞控心跳超过 2 s
  InControl --> Land : 控制台 land、看门狗、RTL 结束
  Land --> Ground : 飞控上锁
  Pilot --> Es1 : 控制台急停降落
  InControl --> Es1 : 控制台急停降落
  Pilot --> Es2 : 控制台急停停桨
  InControl --> Es2 : 控制台急停停桨
  Es1 --> Es2 : 停桨
  Es1 --> Ground : 已降落，estop_reset
  Es2 --> Ground : 已上锁，estop_reset
```

*原图：[ARCHITECTURE.md § 8.4 Ground console (Mac)](ARCHITECTURE.md)*

### 一条 GOTO 指令从控制台到飞控的时序

该指令只改变目标点；脑模型继续负责飞行，其后的每一级安全环节仍然生效。

```mermaid
sequenceDiagram
  participant C as Mac 地面控制台
  participant L as ConsoleLink 接收线程
  participant O as Operator
  participant P as RoutePlanner 线程
  participant T as 控制循环，20 Hz
  participant D as RealDrone
  participant F as 飞控
  C->>L: 经 ECL1 发送 CMD goto x, y，UDP 14700，HMAC 签名
  L->>O: set_goal，检查是否在地理围栏内缩 0.4 m 的范围内
  O-->>L: 已接受，模式 GOTO，plan_start 已入队
  L-->>C: ACK id, ok
  loop every tick
    T->>O: on_control，执行排队的调用
    O->>P: 在 LocalMap 上请求路径，每 1 s 一次
    P-->>O: 绕开已建图障碍物的 A* 路径
    O->>T: pilot.set_waypoints 传入路径
    Note over T: 脑模型、解码器，Operator 放行脑模型指令，<br/>航点偏航偏置、围栏转向、安全护盾、限幅器
    T->>D: 发送指令
    D->>F: SET_POSITION_TARGET_LOCAL_NED 经 mavlink-router，仅在掌控时
    T-->>C: TICK 遥测
  end
  O->>O: 到达最后一个航点，模式 HOLD
```

*原图：[ARCHITECTURE.md § 8.4 Ground console (Mac)](ARCHITECTURE.md)*

## 3. 硬件与飞控

### 无 GPS 导航与状态估计（路线 B）

箱子从机下经过会使下视 ToF 读数变小并把 EKF 高度拉低。路线 B 由机载电脑向飞控发送相对地面的高度；若这些消息停止，EKF 回退到气压计，pilot 看门狗先悬停，再降落。

```mermaid
flowchart LR
  subgraph FCside["飞控"]
    FL["PMW3901 光流"] --> EKF["EKF3"]
    DTOF["VL53L0X 下视 ToF<br/>RNGFND1, 10 Hz"] --> EKF
    IMUc["IMU · 气压计 · 罗盘<br/>（COMPASS_DEC 固定）"] --> EKF
    EKF --> POS["LOCAL_POSITION_NED<br/>（xy 来自光流，z 来自 ExtNav）"]
  end
  subgraph OPIside["Orange Pi"]
    VC["VentralCue<br/>T = baro − ToF"]
    H["离地高度：<br/>机下无物体时用经倾角修正的 ToF，<br/>有物体时用气压计减冻结的地面基线"]
    VC --> H
  end
  DTOF -->|RANGEFINDER| VC
  IMUc -->|SCALED_PRESSURE| VC
  H -->|"VISION_POSITION_ESTIMATE z = −h, 20 Hz<br/>EK3_SRC1_POSZ 6 · VISO_TYPE 1"| EKF
```

*原图：[ARCHITECTURE.md § 6. Navigation and state estimation without GPS (route B)](ARCHITECTURE.md)*

### 硬件架构与接线

课程机架上只有 3901-L0X 是加到飞控上的新部件，其他新增部件都接在 Orange Pi 上。

```mermaid
flowchart LR
  BAT["2S LiPo<br/>Zeee 2200 mAh"]

  subgraph FCB["Matek F405-WMO · ArduCopter 4.7.0（定制固件，MatekF405-TE 目标）"]
    direction TB
    U2["UART2 / SERIAL6<br/>CRSF RCIN"]
    U3["UART3 / SERIAL2<br/>MAVLink2"]
    U4["UART4 / SERIAL4<br/>MSP, 115200"]
    U5["UART5 / SERIAL3<br/>MAVLink2, 57600"]
    I2CF["I²C"]
    PWM["S5–S8 电机输出"]
    VX["Vx 5 V (4.5 A)"]
    V45["4V5 电源轨"]
  end

  subgraph OPI["Orange Pi Zero 3W (A733, 2 × A76, 6 GB)"]
    direction TB
    GP["GPIO UART<br/>RX3 / TX3 / GND"]
    TWI0["I²C TWI0<br/>引脚 3 / 5"]
    TWI1["I²C TWI1<br/>引脚 38 / 40"]
    USBC["USB-C ①（host / OTG）"]
    P5["5 V 输入（GPIO）"]
    P33["3.3 V 输出"]
  end

  FLOWS["Matek 3901-L0X<br/>光流 + 下视 ToF"]
  GPS["HGLRC M100 Pro<br/>仅用罗盘（GPS UART 已拆除）"]
  RX["Happymodel EP2<br/>ELRS 接收机"]
  ESP["XIAO ESP32-C3<br/>DroneBridge 数传"]
  ESC["4 × SoloGood 20A<br/>BLHeli_S 电调 → 电机"]
  TF["前向 VL53L7CX<br/>Pololu #3418, 0x29"]
  TR["后向 VL53L7CX<br/>Pololu #3418, 0x29"]
  UCAM["USB 相机<br/>ELP OV7725 120° 或 Waveshare OV9281"]
  GCS["笔记本 GCS<br/>Mission Planner（Wi-Fi UDP）"]

  BAT --> FCB
  U4 <-->|"MSP：光流 + 测距"| FLOWS
  V45 -->|power| FLOWS
  I2CF <-->|compass| GPS
  U2 <-->|CRSF| RX
  U5 <-->|MAVLink2| ESP
  ESP <-.->|Wi-Fi| GCS
  PWM --> ESC
  U3 <-->|"MAVLink2，3 线<br/>推荐 115200（课程值：460800）"| GP
  VX -->|power| P5
  TWI0 <-->|"I²C, 3.3 V"| TF
  TWI1 <-->|"I²C, 3.3 V"| TR
  P33 -->|"两块合计 ≈ 200 mA<br/>（不足时由 Vx 经 LDO 供电）"| TF
  P33 --> TR
  USBC <-->|"USB 2.0 UVC"| UCAM
```

*原图：[ARCHITECTURE.md § 7. Hardware architecture and wiring](ARCHITECTURE.md)*

### 飞行配置总线图

飞行配置中的每个传感器和执行器要么挂在飞控上，要么挂在机载电脑上，连线上标有总线类型。电源轨见下一张图。

```mermaid
flowchart LR
  FC["Matek F405-WMO<br/>ArduCopter 4.7.0"]
  ESC["4 × SoloGood 20A BLHeli_S"]
  MOT["4 × Gemfan Hurricane 4024"]
  FLOW["Matek 3901-L0X<br/>光流 + 下视 ToF"]
  CMP["HGLRC M100 Pro<br/>仅用罗盘"]
  RX["Happymodel EP2 ELRS"]
  ESP["XIAO ESP32-C3 DroneBridge"]
  OPI["Orange Pi Zero 3W"]
  TOFF["前向 VL53L7CX<br/>Pololu 3418"]
  TOFR["后向 VL53L7CX<br/>Pololu 3418"]
  CAM["USB 相机<br/>ELP OV7725 或 OV9281"]
  FC -->|"DShot600, S5–S8"| ESC
  ESC --> MOT
  FC <-->|"MSP 115200, UART4 SERIAL4"| FLOW
  FC <-->|"I²C 罗盘"| CMP
  FC <-->|"CRSF, UART2 SERIAL6"| RX
  FC <-->|"MAVLink2 57600, UART5 SERIAL3"| ESP
  FC <-->|"MAVLink2, UART3 SERIAL2"| OPI
  OPI <-->|"I²C TWI0, 0x29"| TOFF
  OPI <-->|"I²C TWI1, 0x29"| TOFR
  OPI <-->|"UVC，经 USB-C"| CAM
```

*原图：[HARDWARE.md § 3. Wiring and pin-out](HARDWARE.md)*

### 供电分配

2S 电池接飞控电池焊盘和电调；Orange Pi 由飞控的 Vx 供电，相机和两块 VL53L7CX 由 Orange Pi 供电。

```mermaid
flowchart LR
  BAT["2S LiPo"] --> FC["F405-WMO<br/>电池焊盘"]
  FC -->|"Vx 5 V, 4.5 A"| OPI["Orange Pi Zero 3W"]
  FC -->|"4V5"| LOW["3901-L0X · EP2 · ESP32-C3 · GPS"]
  FC -->|"DShot600"| ESC["4 × 电调 → 电机"]
  BAT --> ESC
  OPI -->|"USB 5 V"| CAM["USB 相机"]
  OPI -->|"3.3 V, ≈ 200 mA"| TOF["2 × VL53L7CX"]
```

*原图：[HARDWARE.md § 3.3 Power](HARDWARE.md)*

### 飞控参数分层叠加

参数集分层构建，仿真器使用完全相同的分层；最后一步是原文第 3 节所列的真实飞控替换项。

```mermaid
flowchart LR
  A["course_base.parm<br/>课程 .param 文件<br/>（make_sitl_params.py）"] --> B["fhb_delta.parm<br/>光流 + ToF，无 GPS，<br/>笼内飞行包线"]
  B --> C["variants/G_extnav_height.parm<br/>路线 B"]
  C --> D["variants/companion_fs.parm<br/>机载电脑失效保护"]
  D --> E["真实飞控替换项<br/>（本页 §3）"]
```

*原图：[FLIGHT_CONTROLLER.md § Flight-controller configuration (ArduCopter 4.7 on Matek F405-WMO)](FLIGHT_CONTROLLER.md)*

### RC 模式开关与接管门限

飞手在 LOITER 中飞行，切到 GUIDED 把无人机交给机载电脑，切出 GUIDED 立即收回控制权。SITL 台架参数文件与 `course_base.parm` 的开关档位设置不同，见原图注。

```mermaid
flowchart LR
  RC["ELRS EP2 接收机<br/>模式开关在通道 6"]
  RC --> G["GUIDED<br/>custom_mode 4"]
  RC --> L["LOITER<br/>custom_mode 5"]
  RC --> S["STABILIZE<br/>custom_mode 0"]
  G -->|"仅当 --mode guided、飞控已解锁、<br/>飞控心跳新于 2 s、<br/>高度至少 0.3 m"| C["机载电脑发送<br/>速度设定值"]
  G -->|"切出 GUIDED"| P["飞手收回控制权"]
  L --> P
  S --> P
```

*原图：[FLIGHT_CONTROLLER.md § 2.1 RC mode switch](FLIGHT_CONTROLLER.md)*

### 无 GPS 时 EKF3 的数据来源

`EK3_SRC1_POSXY 0` 表示完全没有位置源，水平位置由光流速度积分得到。高度用路线 B（机载电脑的 z），气压计作回退，偏航来自罗盘。

```mermaid
flowchart LR
  FLOW["3901-L0X 光流<br/>FLOW_TYPE 7 MSP"] -->|"水平速度"| EKF
  RNG["3901-L0X 下视 ToF<br/>RNGFND1_TYPE 32 MSP"] -->|"距离，SITL POSZ 2"| EKF
  CMP["HGLRC M100 Pro 罗盘<br/>I²C"] -->|"偏航"| EKF
  OPI["Orange Pi 机载电脑<br/>VISION_POSITION_ESTIMATE z，20 Hz"] -->|"离地高度，POSZ 6"| EKF
  BARO["飞控气压计"] -->|"回退高度"| EKF
  IMU["飞控 IMU"] -->|"姿态"| EKF
  GPS["HGLRC M100 Pro GPS<br/>GPS1_TYPE 0"] -.->|"不融合"| EKF
  EKF["EKF3 源组 1<br/>POSXY 0 · VELXY 5 · YAW 1 · POSZ 6"]
  EKF --> CTRL["位置与速度控制器<br/>GUIDED 和 LOITER"]
```

*原图：[FLIGHT_CONTROLLER.md § 3. Sensors and EKF3 without GPS](FLIGHT_CONTROLLER.md)*

## 4. 仿真与优化

### 上真机前的四种测试环境

同一套 `ecps_*.py` 控制代码在上真机之前跑四种测试环境，每一种都让更多环节变成真实的。图中标明各环境里哪些部分是真实的、哪些是仿真的。

```mermaid
flowchart TD
  CODE["共享控制代码<br/>ecps_pilot, ecps_retina, ecps_decoder,<br/>ecps_struct, ecps_map, my_minifly"]
  subgraph M1["1 快速孪生，rl/twin.py"]
    T1["仿真：速度模式四旋翼，带 SITL 辨识出的滞后，<br/>射线投射鱼眼相机、ToF、气压计、光流 EKF 误差模型<br/>不用 ArduPilot 和 Gazebo，快于实时"]
  end
  subgraph M2["2 ArduPilot SITL"]
    S2["真实：ArduCopter 4.7 代码和课程参数<br/>仿真：来自 sitl/ecps295.json 的物理模型<br/>无相机：run_s5.py, s7_flow.py"]
  end
  subgraph M3["3 Gazebo + ArduPilot SITL，run_g4.py"]
    G3["真实：ArduCopter 代码<br/>仿真：Gazebo 物理、鱼眼相机、下视 ToF、<br/>SITL 光流、由 SIM_STATE 射线投射的 8×8 ToF"]
  end
  subgraph M4["4 处理器在环"]
    P4A["仿真 PC：Gazebo + SITL<br/>pil_rog.py 推送相机和 ToF 数据"]
    P4B["Orange Pi：ecps_companion.py --pil<br/>真实 CPU、RealDrone 后端、看门狗"]
  end
  subgraph M5["真机"]
    R5["真实：Orange Pi、相机、VL53L7CX、<br/>F405-WMO, 3901-L0X"]
  end
  CODE --> T1
  CODE -->|"仅 mavlink_twin.py"| S2
  CODE --> G3
  CODE --> P4B
  CODE --> R5
  P4A -->|"UDP：图像帧、8×8 区、起飞标志 10 Hz"| P4B
  P4A <-->|"SITL MAVLink, UDP 14561"| P4B
```

*原图：[README.md § Layers](../README.md)*

### 开发与验证架构

共享代码同时用于快速孪生和 Gazebo；优化结果经 Gazebo 配对验收后再进入真机阶段。真机部分在原图中标为计划中。

```mermaid
flowchart LR
  subgraph CODE["共享代码（各处逐位一致）"]
    RT["ecps_retina · my_minifly · ecps_decoder<br/>ecps_pilot · ecps_struct · mb"]
  end

  subgraph TWIN["快速孪生（rl/twin.py）"]
    T1["NED 运动学 + SITL 辨识出的滞后<br/>射线投射鱼眼相机、ToF、气压计<br/>随机化的光流误差模型"]
    T2["程序化世界：笼子、lowbox、空场"]
  end

  subgraph OPT["优化（rl/cmaes_run.py）"]
    O1["CMA-ES, 12–14 个候选 × 12–16 个回合<br/>适应度 v3 → v6，硬安全约束"]
    O2["训练 / 留出 / 测试划分<br/>（cage20、lowbox_s0–49 从不参与训练）"]
  end

  subgraph GZ["Gazebo Harmonic + ArduPilot SITL (run_g4.py)"]
    G1["ecps295_quad 机架、鱼眼相机、笼子、lowbox<br/>锁步 RTF ≥ 0.95 或仿真时钟模式"]
    G2["与参考配置做配对验收<br/>McNemar / bootstrap"]
  end

  subgraph HW["真机（计划中）"]
    H1["台架：ToF、相机、Pi 时序"]
    H2["R0–R6 笼内阶段<br/>悬停 → 安全护盾 → 路线 B → 故障 → 墙面 → 巡逻"]
  end

  RT --> T1
  RT --> G1
  T1 --> O1 --> O2 -->|"留出集最优代<br/>→ minifly/*.yaml"| G1
  G1 --> G2 -->|"通过验收的配置"| H1 --> H2
  G2 -.->|"保真度检查、误差模型"| T1
```

*原图：[ARCHITECTURE.md § 9. Development and verification architecture](ARCHITECTURE.md)*

### 未被采用的实验分支

这些分支都做过实现和测量；保留它们是因为论文把它们作为对照或阴性结果报告。

```mermaid
flowchart LR
  subgraph BYP["C4 旁路对照臂（rl/arms.py）"]
    BF["视网膜特征"] -->|"8 个可调的固定符号增益"| BDN["伪 DN 发放率"] --> BDEC["相同的解码器 + 安全护盾"]
  end
  subgraph MBX["阶段 E 蘑菇体（mb.py）"]
    KC["300 个 Kenyon 细胞<br/>每个 4–7 个输入，5 % k-WTA"] --> MBON["MBON 趋近输出 m"]
    DA["PPL1（接触、险情）<br/>PAM（新覆盖区域）"] -.->|"多巴胺门控抑制"| MBON
    MBON -->|"缩放前进量，增加扫视驱动"| DEC2["解码器"]
  end
```

*原图：[ARCHITECTURE.md § 10. Experimental branches that were not adopted](ARCHITECTURE.md)*

### 一次 Gazebo 运行的进程与链路

一次 Gazebo 运行在机器上是四个进程：仿真器、相机桥、ArduPilot SITL 和控制脚本 `run_g4.py`；`monitor.py` 为可选面板。并行实例各有自己的 TCP 端口、FDM 端口、共享内存文件和 Gazebo 分区。

```mermaid
flowchart LR
  GZ["Gazebo Harmonic<br/>gz sim -s -r --headless-rendering<br/>世界文件 ecps295_*.sdf"]
  CB["cam_bridge.py<br/>系统 Python"]
  SHM["共享内存<br/>/dev/shm/ecps295_cam<br/>192×144 灰度"]
  SITL["ArduPilot SITL<br/>arducopter --model JSON<br/>Copter-4.7.0"]
  RUN["run_g4.py<br/>视网膜、脑模型、解码器、pilot、安全护盾<br/>20 Hz 控制循环"]
  MON["monitor.py<br/>可选的实时面板"]
  GZ -->|"相机话题 /ecps295/camera"| CB
  CB --> SHM
  SHM --> RUN
  GZ <-->|"FDM, UDP 9002"| SITL
  SITL -->|"MAVLink2, TCP 5760"| RUN
  RUN -->|"逐周期数据流，UDP 5799"| MON
```

*原图：[SIMULATION_GUIDE.md § 4. Gazebo + SITL](SIMULATION_GUIDE.md)*

### 一次 Gazebo 运行的执行时序

按 `gz_g4.sh` 的执行顺序，画出从命令行启动到生成结果文件的各个步骤。

```mermaid
sequenceDiagram
  participant OP as 操作员
  participant S as gz_g4.sh
  participant GZ as Gazebo
  participant CB as cam_bridge.py
  participant FC as ArduPilot SITL
  participant R as run_g4.py
  OP->>S: bash gz_g4.sh world tag args
  S->>GZ: 启动 gz sim -s -r --headless-rendering
  S->>CB: 启动 cam_bridge.py
  GZ-->>CB: /ecps295/camera 上的相机帧
  CB-->>R: /dev/shm/ecps295_cam 中的最新帧
  S->>FC: 启动 arducopter --model JSON --defaults params
  FC->>GZ: 经 UDP 9002 传 FDM 状态
  S->>R: python run_g4.py --url tcp:127.0.0.1:5760
  R->>FC: MAVLink2 连接，等待飞控心跳
  R->>FC: 设置 GUIDED，解锁，起飞
  loop every 20 Hz tick
    R->>R: 读帧、视网膜、脑模型、解码器、安全护盾
    R->>FC: 速度设定值
    R->>FC: 带 --ext-height 时发 VISION_POSITION_ESTIMATE z
    FC-->>R: LOCAL_POSITION_NED, RANGEFINDER, SIM_STATE
  end
  R->>FC: LAND 并等待上锁状态
  R-->>S: run.json 和逐周期 CSV
  S->>S: 由两条 stats 消息得出整段 RTF
  S->>FC: 结束进程
  S->>CB: 结束进程
  S->>GZ: 结束进程
```

*原图：[SIMULATION_GUIDE.md § 4. Gazebo + SITL](SIMULATION_GUIDE.md)*

### CMA-ES 优化与 Gazebo 验收流程

一次优化从起点到 Gazebo 验收的流程。快速孪生（原文称快仿真）只负责筛选，是否采用以 Gazebo 上的配对统计为准。

```mermaid
flowchart TD
  X0["起点 x0<br/>v3_2，或 --base / --x0-yaml"]
  W["每代抽 K 个程序化世界<br/>worlds.py episode_set，cage20 不参与训练"]
  ASK["es.ask：popsize 个候选<br/>params.py 解码器参数，z 空间 0..1"]
  RUN["batch.py 并行跑 episode.py + twin.py<br/>候选和参考飞同一批回合，公共随机数"]
  FIT["适应度 F，v1–v6<br/>es.tell，写 gens.jsonl 和 es.pkl"]
  VAL{"每 5 代或最后一代？<br/>--val-every 5"}
  HO["分布均值在 24 回合留出集上评估<br/>更好则写 best_val.json"]
  NEXT{"还有下一代？"}
  EXP["导出留出集最优点<br/>validate.py export / export_arms.py<br/>→ configs/、minifly/*.yaml"]
  GZ["Gazebo 复测<br/>server/gz_batch.py 或 validate.py gazebo<br/>cage20 32 个种子，lowbox 50 个世界"]
  ST["配对统计 stats.py<br/>McNemar、bootstrap CI、Wilcoxon、Holm"]
  GATE{"显著好于参考？"}
  ACC["采用为新配置"]
  KEEP["保留为参数变体或对照"]
  X0 --> W --> ASK --> RUN --> FIT --> VAL
  VAL -->|"是"| HO --> NEXT
  VAL -->|"否"| NEXT
  NEXT -->|"是"| W
  NEXT -->|"否"| EXP --> GZ --> ST --> GATE
  GATE -->|"是"| ACC
  GATE -->|"否"| KEEP
```

*原图：[rl/README.md § 1. 内容](../rl/README.md)*

### 台式机仿真环境的安装与验收阶段

安装各阶段按图中顺序执行，只有 apt 和驱动需要 sudo。已完成的阶段会在 `~/sim/.deploy/` 打标记，因此脚本可重复运行（`FORCE=1` 重做某阶段）。

```mermaid
flowchart LR
  CH["check<br/>系统版本 · 磁盘至少 30 GB · 网络"] --> DR["driver<br/>NVIDIA open 驱动 570 或更新<br/>重启"]
  DR --> AP["apt<br/>Gazebo Harmonic · gz Python 绑定"]
  AP --> CO["conda<br/>flydrones · ardupilot · Python 3.11"]
  CO --> FD["flydrones<br/>分支 ecps295-sim<br/>MiniFly 权重"]
  FD --> AD["ardupilot<br/>Copter-4.7.0 加补丁<br/>编译 SITL"]
  AD --> GZ["gzplugin<br/>ardupilot_gazebo 082a0fe"]
  GZ --> HW["hw<br/>运行树位于 ~/sim/hw"]
  HW --> LA["launchers<br/>gz_g4.sh · run_matrix*.sh"]
  LA --> VE["verify_desktop.sh<br/>门禁 · 锚点 · RTF · Gazebo 闭环"]
```

*原图：[deploy/desktop/README.md § 用法](../deploy/desktop/README.md)*

## 5. 机载部署与地面控制台

### 机载软件部署：Orange Pi 与飞控

Orange Pi 上的机载电脑进程负责 20 Hz 控制循环和路线 B 高度发送，经 mavlink-router 与飞控通信。原文说明（2026-10-15/16 状态）：软件已部署到 Pi 上，传感器尚未接线。

```mermaid
flowchart TB
  subgraph OPI["Orange Pi Zero 3W — Linux, Python"]
    direction TB
    subgraph MAIN["ecps 机载电脑进程（run_g4 式运行器，真机后端）"]
      direction TB
      CAMT["相机线程<br/>V4L2 / UVC 采集 → 最新帧"]
      TOFT["ToF 线程 ×2<br/>/dev/i2c-0, /dev/i2c-1 → 最新 8×8 帧"]
      MAVT["MAVLink I/O<br/>pymavlink：遥测缓存，心跳 1 Hz"]
      LOOP["20 Hz 控制循环<br/>视网膜 → MiniFly → 解码器 → 转向层 → 地图 → 安全护盾 → 限幅器 → 看门狗"]
      HGT["路线 B 高度发送器，20 Hz<br/>(VentralCue)"]
      LOG["运行日志<br/>逐周期 CSV + run.json"]
      CAMT --> LOOP
      TOFT --> LOOP
      MAVT --> LOOP
      LOOP --> MAVT
      HGT --> MAVT
      LOOP --> LOG
    end
    MR["mavlink-router<br/>（课程配置）"]
    MON["可选：monitor.py 逐周期数据流<br/>（飞行时关闭）"]
    MAVT <-->|"UDP localhost"| MR
    LOOP -.-> MON
  end

  subgraph FC["Matek F405-WMO — ArduCopter 4.7.0 (ChibiOS)"]
    direction TB
    MAVFC["MAVLink 端点<br/>SERIAL2（机载电脑），SERIAL3（数传）"]
    MSP["MSP 驱动<br/>光流 + 测距"]
    EKF3["EKF3<br/>光流 + 下视 ToF + ExtNav z + IMU + 气压计 + 罗盘"]
    GD["GUIDED 模式<br/>速度目标"]
    PC["位置 / 速度控制器"]
    AC["姿态 + 角速率控制器 → 混控 → 电调输出"]
    FSF["失效保护：FS_GCS（机载电脑 sysid 254）→ 降落，<br/>GUID_TIMEOUT, FS_THR（RC）→ 降落，FS_EKF"]
    LOGF["SD 卡上的 DataFlash 日志"]
    MSP --> EKF3
    MAVFC --> GD --> PC --> AC
    MAVFC -->|VISION_POSITION_ESTIMATE| EKF3
    EKF3 --> PC
    FSF -.-> AC
    AC -.-> LOGF
  end

  MR <-->|"UART3 · MAVLink2"| MAVFC
```

*原图：[ARCHITECTURE.md § 8. Onboard software deployment: Orange Pi vs flight controller](ARCHITECTURE.md)*

### Orange Pi 上的服务与进程

机载常驻服务、传感器线程和控制台链路。虚线表示 `ecps-security.service` 对入站 UDP 端口的过滤。

```mermaid
flowchart LR
  FC["Matek F405-WMO<br/>ArduCopter 4.7.0"]
  SEC["ecps-security.service<br/>nftables 白名单"]
  MR["mavlink-router.service<br/>mavlink-routerd"]
  CP["ecps-companion.service<br/>ecps_companion.py<br/>ECPS_MODE=shadow"]
  CAM["相机线程<br/>V4L2 /dev/video0"]
  TOF["ToF 线程 ×2<br/>/dev/i2c-0 和 /dev/i2c-1"]
  MKT["标记检测线程<br/>ArUco，A55 核心"]
  CON["ECPS Console.app<br/>Mac 地面控制台"]
  FC <-->|"UART2 /dev/ttyS2<br/>MAVLink2 115200"| MR
  MR <-->|"UDP 14560 localhost"| CP
  MR -->|"UDP 14550"| CON
  CAM -->|"192×144 灰度帧"| CP
  TOF -->|"8×8 区，15 Hz"| CP
  CP --> MKT
  CP <-->|"UDP 14700 ECL1"| CON
  SEC -.->|"过滤入站 UDP 14550"| MR
  SEC -.->|"过滤入站 UDP 14700"| CP
```

*原图：[deploy/orangepi/README.md § 服务与进程 / Services and processes](../deploy/orangepi/README.md)*

### Orange Pi 开机顺序

`ecps-security.service` 和 `mavlink-router.service` 开机自启。`ecps-companion.service` 已安装但不开机自启，地面测试通过后手动 `systemctl start`。

```mermaid
flowchart TD
  BOOT["上电<br/>armbianEnv overlays i2c0 i2c1 uart2"] --> DEV["/dev/i2c-0 和 /dev/i2c-1<br/>/dev/ttyS2"]
  DEV --> SEC["ecps-security.service<br/>开机自启<br/>nftables 白名单"]
  SEC --> MR["mavlink-router.service<br/>开机自启<br/>飞控 UART 转到 UDP 14560 和 14550"]
  MR --> MAN["systemctl start ecps-companion<br/>已安装，未设开机自启<br/>ECPS_MODE=shadow"]
  MAN --> CON["ECPS Console.app 连接<br/>HELLO 心跳 5 Hz<br/>指引需要 ECPS_MODE=guided"]
```

*原图：[deploy/orangepi/README.md § 开机流程 / Boot sequence](../deploy/orangepi/README.md)*

### 地面控制台、机载电脑与飞控的报文流

控制台到机载电脑走 ECL1（UDP 14700），到飞控走 Pi 上的 mavlink-router（UDP 14550）。机载电脑只在掌控时经路由向飞控发速度设定值和路线 B 高度。

```mermaid
flowchart LR
  CON["ECPS Console.app<br/>运行在 Mac 上"]
  ECL["ECL1，经 UDP 14700<br/>Pi 上的 companion/console_link.py"]
  MR["Pi 上的 mavlink-router"]
  CP["机载电脑进程<br/>ecps_companion.py"]
  FC["Matek F405-WMO<br/>ArduCopter 4.7.0"]
  CON -->|"HELLO 心跳 5 Hz · CMD · ESTOP · PING"| ECL
  ECL -->|"TICK 20 Hz · BRAIN · CAM · MAP 2 Hz · TOF · STATUS 2 Hz"| CON
  CON -->|"MAVLink2，经 UDP 14550"| MR
  MR <-->|"MAVLink2，经 UART /dev/ttyS2 115200"| FC
  MR <-->|"UDP 14560 localhost"| CP
  CP -.->|"经路由器：速度设定值<br/>和路线 B 高度，仅在掌控时"| FC
```

*原图：[deploy/console/README.md § 链路 / Message flow](../deploy/console/README.md)*

### 地面控制台两级急停状态机

一级是紧急降落（LAND），二级是停桨（强制上锁）。飞控确认之前一直重发，确认之后不再重发（飞手重新切模式仍然优先）；解除需要飞控状态新鲜且已上锁。

```mermaid
stateDiagram-v2
  state "空闲" as Idle
  state "一级，每 200 ms 发一次 LAND" as Land
  state "一级已确认" as LandOK
  state "二级，每 150 ms 发一次强制上锁" as Kill
  state "二级已确认" as KillOK
  state "已停止重发，飞控未确认" as Held
  Idle --> Land : Esc 或红色大按钮
  Idle --> Kill : 按住 1.5 s
  Land --> LandOK : 飞控报告 LAND 或已上锁，或收到 COMMAND_ACK
  Land --> Held : 已停止发送，飞控未确认
  Kill --> KillOK : 飞控报告已上锁，或收到 COMMAND_ACK
  Kill --> Held : 已停止发送，飞控未确认
  LandOK --> Idle : estop_reset，飞控状态新鲜且已上锁
  KillOK --> Idle : estop_reset，飞控状态新鲜且已上锁
  Held --> Idle : estop_reset，飞控状态新鲜且已上锁
```

*原图：[deploy/console/README.md § 使用流程（真机）](../deploy/console/README.md)*

## 6. 测试流程

### 从台架到笼内飞行的总体路线

计划从台架推进到处理器在环，再到笼内飞行，每个阶段必须完全通过后才开始下一个。原文说明（2026-10-15 状态）这些阶段都尚未飞过。

```mermaid
flowchart LR
  B["台架<br/>3901-L0X · VL53L7CX · 相机 A/B"] --> F["首飞 10-28<br/>仅 RC，完整日志"]
  F --> T["更新孪生<br/>依据飞行日志 + 台架"]
  T --> P["Orange Pi 上的 PIL<br/>p99 < 25 ms"]
  P --> R0["R0 手动 Loiter<br/>光流 + ToF"]
  R0 --> R1["R1 机载电脑<br/>GUIDED 悬停"]
  R1 --> R1b["R1b 真机上的<br/>安全护盾"]
  R1b --> R2["R2 路线 B"]
  R2 --> R3["R3 机载电脑故障"]
  R3 --> R4["R4 接近墙面"]
  R4 --> R5["R5 120 s 巡逻"]
  R5 -.-> R6["R6 关闭脑模型对照臂<br/>（可选）"]
```

*原图：[FLIGHT_TEST_PLAN.md § Real-drone test plan](FLIGHT_TEST_PLAN.md)*

### 进笼前的步骤与放行门槛

进笼前的各个步骤和放行门槛，每道门槛就是原文第 2 节表格中的退出标准。

```mermaid
flowchart LR
  B["台架<br/>3901-L0X · VL53L7CX · 相机 A/B"] -->|"门槛：数值回填到孪生<br/>和 Gazebo 模型，相机已选定"| F["首飞 10-28<br/>仅 RC，完整日志"]
  F -->|"门槛：取得用于更新孪生的日志"| T["更新孪生<br/>悬停油门、时间常数、<br/>障碍物布局"]
  T -->|"门槛：回归测试悬停、单面墙、<br/>巡逻各 × 5，得出孪生误差表"| P["Orange Pi 上的 PIL<br/>taskset, pil_replay.py"]
  P -->|"门槛：p99 每周期低于 25 ms，<br/>无过热降频"| C["笼内飞行"]
```

*原图：[FLIGHT_TEST_PLAN.md § 2. Before the cage](FLIGHT_TEST_PLAN.md)*

### 笼内测试阶段与放行门槛

只有达到上一阶段的门槛才能进入下一阶段。门槛文字取自原文第 3 节的指标和第 5 节的里程碑标准。

```mermaid
flowchart LR
  R0["R0 手动 Loiter<br/>光流 + ToF, 3 × 60 s"] -->|"门槛：机载电脑看门狗和 FS_GCS_ENABLE 5<br/>地面测试通过，能靠光流 + ToF 保持位置"| R1["R1 机载电脑 GUIDED 悬停<br/>5 × 60 s"]
  R1 -->|"门槛：GUIDED 悬停至少 60 s，<br/>记录位置和高度 SD（G2）"| R1B["R1b 真机上的安全护盾<br/>前进、后退、斜向 × 3"]
  R1B -->|"门槛：记录停止距离与 d_safe 的对比，<br/>无停滞或振荡"| R2["R2 路线 B<br/>开启和关闭各飞 5 次"]
  R2 -->|"门槛：路线 B 在真机上验证通过（G3）"| R3["R3 机载电脑故障<br/>kill -STOP、拔掉相机，各 3 次"]
  R3 -->|"门槛：记录到悬停和降落的时间<br/>以及降落质量"| R4["R4 接近墙面<br/>3 种墙面 × 2 种控制器 × 至少 5 次"]
  R4 -->|"门槛：每种条件至少<br/>5 次接近墙面（G3）"| R5["R5 120 s 巡逻<br/>2 种控制器 × 至少 5 次"]
  R5 -.-> R6["R6 关闭脑模型对照臂<br/>可选，× 3"]
```

*原图：[FLIGHT_TEST_PLAN.md § 3. Cage stages](FLIGHT_TEST_PLAN.md)*
