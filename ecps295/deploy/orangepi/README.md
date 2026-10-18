# ECPS295 Orange Pi Zero 3W 部署包 / deployment package

部署于 2026-10-15（Orange Pi `root@192.168.86.20`，Armbian 26.8.3 / Ubuntu 26.04，内核 6.6.98，Python 3.14.4）。
完整测试报告见 `项目文档/07_机载部署/ECPS295_OrangePi部署与性能测试报告_20261015.zh.md` / `.en.md`（中英双语）。
Full test report: `项目文档/07_机载部署/ECPS295_OrangePi部署与性能测试报告_20261015.en.md` (+ `.zh.md`).

## Pi 上的布局 / Layout on the Pi (`/opt/ecps295`)

| 路径 / path | 内容 / content |
|---|---|
| `venv/` | Python 3.14 venv: numpy 2.4.6, scipy 1.17.1, cma 4.5.0, PyYAML 6.0.3, pymavlink 2.4.50, opencv-python-headless 5.0.0.93, pyserial, smbus2, psutil |
| `src/flydrones/` | upstream FlyDrones（与 ROG / Mac mini 相同 / identical） |
| `ecps295/` | 机载运行代码 = ROG `~/sim/rl_h2/ecps295`（Gazebo 验收用的同一份；含 S7 地图 `ecps_map.py`、`minifly/s7_w2map.yaml`；不含 gazebo/） |
| `twin/` | 快仿真代码 = Mac mini `~/rl_exp/s1b`（SIL 等价性测试、`rl.gates`） |
| `companion/` | 本包：`ecps_companion.py`（主程序）、`real_drone.py`（真机/台架后端）、`camera.py`、`tof_vl53l7cx.py`、`pil_link.py`（Gazebo 处理器在环，10-16） |
| `drivers/vl53l7cx/` | ST ULD（stm32duino/VL53L7CX，BSD-3，未改动）+ Linux i2c-dev 平台层 → `linux/libvl53l7cx.so` |
| `config/companion.yaml` | 硬件配置：模型、MAVLink、相机、两块 ToF（总线、方向翻转） |
| `config/mavlink-router.conf` | → `/etc/mavlink-router/main.conf` |
| `config/fc_flight.parm` | 飞控飞行参数：只认机载程序 sysid 254 为地面站，失联 2 s LAND / FC flight params: companion-loss failsafe (`MAV_GCS_SYSID_HI 254`) |
| `systemd/ecps-companion.service` | → `/etc/systemd/system/`（已安装，**未设开机启动**） |
| `scripts/` | `install_pi.sh`、`profile_matrix.sh`、`sil_equiv.py`、`sitl_hil_test.py`、`tof_probe.py`、`cam_probe.py` |
| `build/` | mavlink-router 源码与编译、ArduPilot Copter-4.7.0 SITL（测试用） |
| `logs/` | 每次运行一个目录：`ticks.csv`、`system.csv`、`run.json` |

系统改动 / system changes: `/boot/armbianEnv.txt` 加 `overlays=i2c0 i2c1`（备份 `/root/ecps_backup/armbianEnv.txt.pre_ecps`）
→ `/dev/i2c-0`（TWI0，PB2/PB3，排针 3/5）、`/dev/i2c-1`（TWI1，PB7/PB8，排针 38/40）；apt: i2c-tools meson ninja-build
libsystemd-dev pkg-config build-essential；`mavlink-routerd` 装到 `/usr/bin`，`mavlink-router.service` 开机启动。
原有的 `uart0-pb9-pb10` overlay（`/dev/ttyS0`，排针 8/10）和 `/root/opiRasPiUartTest` 未改动。

## 常用命令 / Commands

```sh
R=/opt/ecps295; PY=$R/venv/bin/python
# 台架（不接飞控）：真实相机 + 真实 ToF / bench, no FC
$PY $R/companion/ecps_companion.py --mode bench --affinity 6,7 --seconds 60
# 无传感器的冒烟测试 / smoke test without sensors
$PY $R/companion/ecps_companion.py --mode bench --camera sim --tof sim --affinity 6,7 --seconds 30
# 传感器台架 / sensor bench
$PY $R/scripts/tof_probe.py --sensor front            # 再加 --orient 做方向标定
$PY $R/scripts/cam_probe.py --device /dev/video0
# 接飞控、只旁路（发心跳 + 路线 B 高度，不发速度）/ FC connected, shadow
systemctl start ecps-companion        # 默认 ECPS_MODE=shadow
journalctl -u ecps-companion -f
# 地面测试通过后才改 guided / guided only after the ground tests
systemctl edit ecps-companion          # [Service] Environment=ECPS_MODE=guided
```

**安全约定 / safety contract:** 伴飞程序从不解锁、不起飞、不切模式（只有在它掌控时、看门狗要求时发 LAND）。
只有 `--mode guided` 且飞控自己的心跳报告 GUIDED + 已解锁（心跳 < 2 s）时才发送速度指令；飞手把模式开关拨离
GUIDED 立即收回控制权。The companion never arms, takes off or changes mode (except LAND from a watchdog while in control).

## Gazebo 处理器在环 / Gazebo PIL (2026-10-16)

Gazebo + SITL 在 ROG（`~/sim/rl_h2`，单实例），大脑和安全层用 Pi 上的真实机载程序；相机帧和按真值计算的前后 8×8 ToF 经 UDP 从 ROG 发来，
MAVLink 由 SITL 直接发给 Pi。所有连接由 ROG 发起，走局域网。详见报告第 10 节。
Gazebo + SITL on ROG, the real companion on the Pi; camera frames and the emulated ToF arrive over UDP, SITL talks MAVLink straight
to the Pi. All flows start from ROG, over the LAN. See §10 of the report.

```sh
# Pi（先启动；等不到飞控心跳会以退出码 1 结束，重试即可 / start first, retry on exit code 1）
$PY $R/companion/ecps_companion.py --mode guided --pil 14600 --url udpin:0.0.0.0:14561 --seed <k> \
    --set safety.max_flight_s=420 --affinity 6,7 --tag pil_<name> --log-dir $R/logs/pil
# ROG（不加 --ext-height：路线 B 高度由 Pi 发送 / no --ext-height, the Pi sends route B）
INST=0 ECPS_SIMCLOCK=1 ECPS_TOF_CONE3D=1 ECPS_PIL=192.168.86.20:14600 \
  SITL_EXTRA=--serial2=udpclient:192.168.86.20:14561 NAV=flow EXTRA_PARM=<parms> \
  bash run_matrix_mine.sh <matrix> <log>      # 每行 / line: <tag> <world> s7_w2map@intact <k> --mode patrol --seconds 120 ...
```

Pi 只在 ROG 置位 go（巡逻阶段）时接管，go 归零后自行退出 / the Pi takes control only while ROG sets go and exits when it drops.

## 重新部署 / redeploy (from the Mac)

```sh
deploy/orangepi/scripts/deploy_to_pi.sh <rog_tree_with_src_and_ecps295> <twin_tree_with_src_and_ecps295> root@192.168.86.20
```

## 地面控制台 / ground console (2026-10-17)

`companion/console_link.py`：把大脑、相机、局部 3D 地图、飞行状态推给 Mac 控制台（`deploy/console`，`ECPS Console.app`），
接收指引（悬停 / 巡逻 / 前往目标 / 手动 / 降落）和急停。`config/companion.yaml` 的 `console.port: 14700` 打开它，
`--console 0` 关闭；没有控制台连接时控制路径不变。协议、指引层和急停语义见模块文件头和
`项目文档/08_地面控制台/ECPS295_地面控制台设计与测试报告_20261017.md`（仓库里在 `deploy/console/docs/`）。
`companion/sitl_world.py` 只用于 SITL 测试（`--tof sitl:<layout.json>`：两块 ToF 由 SIM_STATE 真值 + 场地布局模拟）。
`camera.py` 现在保留最后一帧彩色图（`last_bgr`，控制台的彩色画面用）。

The console link is off unless configured; see the module docstring for the protocol (ECL1 over UDP 14700), the
operator layer (applied before the safety governor) and the emergency-stop semantics. **Deployed to `/opt/ecps295` on
2026-10-17 03:16** (backup of the replaced files: `/root/ecps295_backups/20261017_pre_console/` on the Pi); tested
there over the LAN against SITL with the production ports (`deploy/console/tools/sitl_rig.py --deployed`), not yet on
the real drone. `ecps-companion.service` is unchanged (disabled, shadow): in shadow mode the console shows everything
and its emergency stop works, but guidance needs `ECPS_MODE=guided`.
