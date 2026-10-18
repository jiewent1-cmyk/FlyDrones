# ECPS 控制台（Mac 地面站）/ ECPS Console

2026-10-17。MiniFly v4 无人机的 Mac 地面控制台：实时看大脑（FlyDrones Live 风格的脑区面板、脉冲栅格、下行神经元与指令）、
机载 3D 建图（局部占据栅格 + 两块 8×8 ToF 实时射线 + 安全盾约束）、机载相机（大脑视角 / 彩色）、飞行参数（姿态、高度、
速度、电池、EKF、光流、振动、链路健康），指引飞行（悬停 / 大脑巡逻 / 地图点选目标 / 返回起点 / 键盘手动 / 降落），
以及两级急停（**紧急降落**：大红按钮或 Esc；**停桨**：按住 1.5 s）。

设计与测试报告：`项目文档/08_地面控制台/ECPS295_地面控制台设计与测试报告_20261017.md`（GitHub 仓库里在本目录的
`docs/`，原始结果在 `docs/results/`）。

## 组成

| 位置 | 内容 |
|---|---|
| `mac/Sources/*.swift` | 原生外壳：窗口与菜单、Esc 急停（原生按键监听，页面卡死也有效）、MAVLink 编解码与飞控链路、机载遥测链路（ECL1）、急停逻辑、飞行记录、本机网页服务 |
| `mac/web/` | 界面（three.js r170 来自 FlyDrones `docs/vendor`，MIT）：`brain.js` 大脑面板、`eyes.js` 相机、`map3d.js` 3D 建图、`flight.js` + `charts.js` 飞行参数、`app.js` 指引与急停交互、`i18n.js` 界面语言 |
| `mac/Info.plist`、`mac/AppIcon.icns` | 应用包信息、图标 |
| `build.sh` | 只用 Command Line Tools（无需 Xcode）构建 `ECPS Console.app`；`--install` 装到 `/Applications` |
| `tools/ecl_client.py` | ECL1 参考客户端（命令行看遥测、发指令、急停），也用于测试 |
| `tools/sitl_rig.py`、`rig_start.sh`、`rig_stop.sh` | Pi 上的 SITL 测试台（无 GPS 光流导航 SITL + 第二个 mavlink-router + 机载程序 + 由 SITL 真值模拟的 ToF） |
| `tools/console_e2e.py` | 端到端测试（扮演飞手 + 控制台，13 项检查） |
| `tools/gen_mavmeta.py` | 由 pymavlink 生成 `MavMeta.swift`（报文布局与 CRC_EXTRA） |
| 机载端（在 `deploy/orangepi/`） | `companion/console_link.py`（链路 + 指引层）、`companion/sitl_world.py`（仅 SITL 测试）、`ecps_companion.py --console`、`camera.py`（保留彩色帧）、`config/companion.yaml` 的 `console:` 段 |

## 构建与安装

```sh
cd deploy/console
./build.sh --install        # -> /Applications/ECPS Console.app（ad-hoc 签名，约 1.8 MB）
```

构建产物放在 `~/Library/Caches/ECPSConsole/`：项目目录在 iCloud 同步的「文稿」里，同步服务给文件加的属性会让 codesign
失败，而且刚编辑过的文件偶尔会被读成旧的或不完整的内容，所以 `build.sh` 先把源码拷到缓存目录再编译，失败自动重试（最多 3 次）。

第一次连接时 macOS 会询问「允许查找本地网络上的设备」，必须允许（访问 Pi 的局域网地址需要这个权限）。

## 界面语言（中文 / English）

顶栏的「中 / EN」或菜单「显示 → 语言」（英文界面下是 View → Language）切换，选择保存在应用设置（`lang`）里；没选过时跟随
macOS 的首选语言。网页界面、菜单、对话框、窗口标题、急停提示和之后记录的事件都跟着切换；已经在事件栏里的条目保持原来的语言。
机载程序发来的文字本身是英文，中文界面下照旧翻译成中文。系统自动加的菜单项（如「进入全屏幕」）和第一次连接时的本地网络
权限提示由 macOS 的系统语言决定。

实现：原生侧 `Lang` / `L("中文", "English")`（`Util.swift`），网页 `js/i18n.js`：`index.html` 里的固定文字用 `data-en`
（提示文字用 `data-en-title`）写英文，脚本里的文字用 `L(...)`。网页从地址里的 `?lang=` 开始，之后跟随状态里的 `settings.lang`；
切换命令是 `{"c": "lang", "lang": "en"}`。

## 机载端部署（2026-10-17 03:16 已部署到 Pi 的 /opt/ecps295）

已装入 `companion/console_link.py`、`companion/sitl_world.py`（只用于 SITL）、`companion/ecps_companion.py`、`companion/camera.py`、
`config/companion.yaml`（`console: port 14700`）。md5 和回滚方法见报告第 6.1 节，被替换的文件备份在 Pi 的
`/root/ecps295_backups/20261017_pre_console/`。以后更新可以用 `deploy/orangepi/scripts/deploy_to_pi.sh`，或者手动拷贝这几个文件。

`console.port: 14700` 打开后，没有控制台连接时，机载程序的控制路径完全不变，只多一个空闲的接收线程。

`ecps-companion.service` 保持原样：停用，`ECPS_MODE=shadow`。影子模式下控制台能看全部数据，也能急停，但机载程序从不接管，
所以指引按钮不可用。地面测试通过后，用 `systemctl edit ecps-companion` 设 `Environment=ECPS_MODE=guided`，指引才生效。

## 使用流程（真机）

1. Pi 与 Mac 在同一局域网；控制台 设置 → 地址 `192.168.86.20`（或 NetBird 地址），端口 14700 / 14550。
2. 起飞前：顶栏「机载遥测」「飞控」「机载心跳」为绿；「交接后」选「悬停」（默认）。
3. 飞手用遥控器起飞，拨到 GUIDED：机载程序接管，进入所选模式（悬停）。
4. 指引：`P` 大脑巡逻 · `G` 在地图上点选目标（大脑带着避障/扫视飞过去，经航点层）· `B` 返回起点 · `M` 键盘手动
   （W/S 前后，A/D 偏航，Q/E 平移，R/F 升降，松开即悬停）· `H` 悬停 · `L` 降落。所有指令仍经过安全调速、看门狗、围栏掉头和安全盾。
5. 急停：**Esc** 或大红按钮 = 紧急降落（飞控直连 + 机载程序两条路，重发直到飞控确认）；**停桨**按住 1.5 s = 强制上锁。
   急停后机载程序锁定、不再接管，直到在控制台「解除急停」。飞手拨模式开关永远可以接管（急停只在第一次被飞控确认前强制）。

**遥控器是第一道急停**。控制台急停走 Wi-Fi：家里的 Wi-Fi 下实测按下到飞控确认 0.04–0.8 s，但 Pi Zero 3W 的 Wi-Fi
（AIC8800）在上行超过约 40 kB/s 时会卡顿数秒，所以控制台按往返延迟自动降低遥测带宽（顶栏「带宽自动·高/中/低/极低」）。

## 测试

```sh
# Pi 上起 SITL 测试台（独立目录，用 /opt/ecps295 的 venv 和 SITL，只读）
ssh root@192.168.86.20 'sh /root/ecps_console_test/rig_start.sh 1500'
python tools/console_e2e.py --pi 192.168.86.20 --out e2e.json        # 13 项自动检查
ssh root@192.168.86.20 'sh /root/ecps_console_test/rig_stop.sh'       # 停止（不要用 pkill -f）

# 测试已部署的运行时（/opt/ecps295 的程序和配置，按系统服务方式启动，正式端口 14550 / 14560 / 14700）：
# 测试台运行期间会停掉 mavlink-router.service，rig_stop.sh 会把它重新启动。不要同时开着控制台应用跑自动测试（P7 需要所有控制台都断开）
ssh root@192.168.86.20 'sh /root/ecps_console_test/rig_start.sh 2700 --deployed'
python tools/console_e2e.py --pi 192.168.86.20 --ecl 14700 --out lan_auto.json
# 经由 Mac 应用（脚本按应用的界面按钮发同样的消息）：先启动应用，日志重定向到文件，打开截图钩子
mkdir -p snaps && ECPS_SNAPSHOT=$PWD/snaps/ui.png "/Applications/ECPS Console.app/Contents/MacOS/ECPSConsole" > app.out 2>&1 &
python tools/console_e2e.py --pi 192.168.86.20 --ecl 14700 --scenario app_full --app-log app.out --app-pid $! \
    --snap-dir snaps --out lan_app.json
ssh root@192.168.86.20 'sh /root/ecps_console_test/rig_stop.sh'
```

`app_full` 的手动一项（A5）注入的摇杆会和应用网页按真实键盘状态发出的摇杆消息（没按键就是 0）交替到达，测出的速度偏低，
详见报告第 6.4 节。

调试：`ECPS_WEB_ROOT=<mac/web 目录>` 让应用直接读网页源文件（改完 ⌘R 重载）；`ECPS_SNAPSHOT=<png>` 后
`kill -USR1 <pid>` 保存界面截图；菜单「显示 → 在浏览器中打开界面」。飞行记录在 `~/Library/Logs/ECPS Console/<时间>/`
（`fc.tlog` 可用 Mission Planner / pymavlink 打开，`ecps.jsonl` 为机载遥测，`events.jsonl` 为控制台事件）。
