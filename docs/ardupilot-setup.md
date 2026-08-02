# ArduPilot 设置（BiCopter + Lua 固飞差动 / 油门）

本机：Matek **H743-MINI V3** + 官方 **ArduPlane**（BiCopter）+ Lua 固飞等效副翼与固飞油门直通。硬件与接线见 [hardware.md](./hardware.md)，模式见 [flight-modes.md](./flight-modes.md)。

**不需要**自编译固件。首次 DFU / 本地固件下载见 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md)。

## 1. 刷官方固件

推荐按 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md) 操作（本地下载 + 首次 DFU）。摘要：

1. 下载固件：
   ```powershell
   .\scripts\download-matekh743-plane.ps1
   ```
2. **首次**：按住 Boot，用 STM32CubeProgrammer（或 dfu-util）烧写 `firmware/Plane/stable/MatekH743/arduplane_with_bl.hex`。
3. **已装 ArduPilot 后升级**：Mission Planner → **Install Firmware** → Load custom firmware → `arduplane.apj`；或在线选 **MatekH743** → **Plane**。
4. 确认固件含 **Scripting**（近年官方 Plane 默认包含；若无 `SCR_ENABLE`，换较新稳定版）。
5. 刷写完成后连接飞控（115200），不要急着装桨。验证清单：[flash-verify-checklist.md](./flash-verify-checklist.md)。

## 2. 导入参数

两种上传方式：

| 模式 | 用途 | 顺序 |
|------|------|------|
| **全量** | 新板 / 需可复现基线 | [`init.param`](../params/init.param)（默认快照且 `Q_ENABLE=1`）→ **重启**（`Q_*` 出现）→ [`matek-h743-mini-bicopter.param`](../params/matek-h743-mini-bicopter.param) → 重启 |
| **增量** | 已配置过、只改项目差异 | 仅写项目配置（飞控上须已有 `Q_*`） |

`init.param` 来自恢复默认后的导出，并把 `Q_ENABLE` 置为 1，因此全量不再需要单独的 q-enable 文件。已用官方 Plane 参数元数据去掉 `Volatile` / `ReadOnly` / `Calibration` 项（见 [`init.param.removed.txt`](../params/init.param.removed.txt)；重跑：`python scripts/filter-init-params.py`）。未开 `Q_ENABLE` 时 `Q_TILT_*` 不在参数表中，故全量必须在 init 之后重启再写项目配置。

### 2.1 Mission Planner（GUI）

**全量：**

1. Full Parameter List → **Load from file** → `init.param` → Write → **重启** → Refresh。
2. 再 Load → `matek-h743-mini-bicopter.param` → Write → **重启**。
3. 按 [hardware.md](./hardware.md) 完成 Accel **水平校准**（竖装 Roll90 后必做）。

**增量：** 只做第 2 步（并确认 `Q_*` 已存在）。

### 2.2 CLI（`upload-params.py`）

先在 Mission Planner 中 **Disconnect**（串口不能被占用），然后：

```powershell
pip install -r requirements.txt
# 全量（脚本内会在 init 后自动重启并等待，再写项目配置）
python scripts/upload-params.py --port COMx --mode full
# 增量
python scripts/upload-params.py --port COMx --mode incremental
```

默认波特率 115200；`--no-reboot` 可跳过**最终**重启（全量中间的 init 后重启仍会执行）。

### 参数摘要

| 类别 | 关键项 |
|------|--------|
| 机架 | `Q_ENABLE=1`，`Q_FRAME_CLASS=10`，`Q_TILT_TYPE=3`，`Q_TILT_MASK=3`，`Q_ASSIST_SPEED=-1`（关闭固飞空速辅助；有空速计且需要辅助时再改为略高于失速的正值），`SCHED_LOOP_RATE=300`（QuadPlane 要求 ≥100） |
| 姿态 | `AHRS_ORIENTATION=16` |
| CRSF | `BRD_ALT_CONFIG=1`，`SERIAL7_PROTOCOL=23` |
| 脚本 | `SCR_ENABLE=1` |
| 模式 | `FLTMODE_CH=8`；`FLTMODE1..6=17,17,2,2,0,0`（按低/中/高三段垫档，勿循环 `17/2/0`） |
| 输出 | S5=75，S6=76，S7=19，S11=73，S12=74 |
| 无 GPS/罗盘 | `COMPASS_ENABLE=0`，`GPS1_TYPE=0`，`AHRS_GPS_USE=0`，`EK3_SRC1_POSXY/VELXY/VELZ/YAW=0`，`ARMING_CHECK=1048562`（Plane 4.6：启用除 Compass/GPS 外的解锁检查；4.7+ 可改为 `ARMING_SKIPCHK=12`），`ARMING_RUDDER=2`（油门最低时舵右解锁、舵左锁定） |

`Q_TILT_YAW_ANGLE`、倾转 `SERVO*_MIN/TRIM/MAX`、`BTILT_*` 为占位，台架后改写。本项目不做电池监测标定与罗盘校准。

H743-MINI 无内置罗盘；项目默认按**姿态模式**运行（无外置 GPS/罗盘），可解锁台架与 `QSTABILIZE` / `STABILIZE` / `MANUAL`。勿使用需定位的模式（`AUTO` / `RTL` / `QLOITER` / `QRTL` 等）；无罗盘时偏航会漂，垂起偏航保持较差。日后外接 GPS+罗盘时：恢复 `GPS1_TYPE`、打开罗盘、还原 `EK3_SRC1_*`（水平位置/速度用 GPS，航向用罗盘）、`ARMING_CHECK=1`（或 4.7+ 的 `ARMING_SKIPCHK=0`），并完成罗盘校准与 GPS 定位后再飞自主模式。

## 3. 部署 Lua 脚本

### 3.1 CLI（`upload-lua.py`，推荐）

先 Disconnect Mission Planner，再：

```powershell
python scripts/upload-lua.py --port COMx
```

脚本经 MAVFTP 写入飞控 SD 的 `APM/scripts/`（目录不存在会创建），检查 `SCR_ENABLE`，默认重启以加载脚本。跳过重启：`--no-reboot`。其它文件：`--script path\to\file.lua`。

### 3.2 手动拷 SD（备选）

1. 飞控插入 MicroSD，目录：`APM/scripts/`（若无则新建）。
2. 复制 [`lua/bicopter_fw_tilt_aileron.lua`](../lua/bicopter_fw_tilt_aileron.lua) 到该目录（覆盖旧版）。
3. 确认 `SCR_ENABLE=1`，重启飞控。

### 3.3 验证

1. GCS 消息应出现类似：`BTILT: fw tilt+throttle running`。
2. Full Parameter List 中应出现脚本表参数：

| 参数 | 默认 | 含义 |
|------|------|------|
| `BTILT_HORIZ_L` | 1200 | 左倾转真水平 PWM（固飞中心） |
| `BTILT_HORIZ_R` | 1200 | 右倾转真水平 PWM（固飞中心） |
| `BTILT_TRAVEL` | 100 | 满杆时相对 HORIZ 的单侧最大偏置（µs） |
| `BTILT_GAIN` | 0.3 | 倾转差动增益 0..1（由低到高试） |
| `BTILT_REV` | 1 | 倾转横滚符号：`1` 或 `-1`，反了改符号 |
| `BTILT_THR` | 1 | `1` 固飞油门直通 S11/S12；`0` 仅倾转 |
| `BTILT_YAWDT` | 0.1 | 固飞偏航差动增益 -1..1（负号反转；约等于 `RUDD_DT_GAIN` 量级） |

脚本仅在 **`STABILIZE`(2)** / **`MANUAL`(0)** 覆写倾转与（可选）油门；**`QSTABILIZE`** 不覆写。

`BTILT_THR` / `BTILT_YAWDT` 使用独立脚本表键 100；`BTILT_HORIZ_R` 使用表键 101（与倾转表键 89 分开；ArduPilot 不能扩大已有表的槽位数）。旧版 `BTILT_HORIZ` 升级后可忽略，台架时把原值抄到 L/R。

### 固飞油门不转（已知 BiCopter 路径）

现象：`QSTABILIZE` 解锁后电机可转；`MANUAL` / `STABILIZE` 已 Arm，推油门但 Mission Planner **Servo Output** 里 S11/S12 PWM 不变（卡在约 `SERVO*_MIN`）。

原因：stock BiCopter 在固飞下用 `AP_MotorsTailsitter` 关断写 PWM，盖掉 Plane 双发混控对 73/74 的 scaled 输出。本仓库用脚本在固飞模式直通油门（`BTILT_THR=1`），不改固件。

台架核对（拆桨）：

1. 更新 SD 卡脚本 → 重启 → GCS 见 `BTILT: fw tilt+throttle running`，参数表有 `BTILT_THR`。
2. `QSTABILIZE` Arm：抬油门仍应慢转（脚本未接管）。
3. `MANUAL` Arm：推油门 → S11/S12 PWM 上升且电机转；回中停转。
4. 固飞打偏航 → 左右油门差动；方向反了把 `BTILT_YAWDT` 设为负值。

## 4. 台架标定（拆桨）

### 需标定参数一览

项目 param 中倾转 / `BTILT_*` / 升降舵端点为占位，须台架改写后再飞。悬停 PID、过渡速率等保持默认，试飞后再调（见 §6）。

**地面（台架前）：**

| 项 | 相关参数 | 说明 |
|----|----------|------|
| 加速度计水平校准 | `INS_ACC*` 等（MP 向导） | 设好 `AHRS_ORIENTATION=16` 并重启后必做；HUD 与机身一致 |
| 遥控行程校准 | `RCn_MIN` / `TRIM` / `MAX` | 链路通后按实际杆量校准；CH8 与模式档对齐 |

**台架（拆桨）：**

| 参数 | 占位默认 | 标定目标 |
|------|----------|----------|
| `SERVO5_MIN` / `SERVO6_MIN` | 1100 | 机械「水平以下」极限；尽量靠近水平 |
| `SERVO5_TRIM` / `SERVO6_TRIM` | 1500 | 垂直（垂起中心） |
| `SERVO5_MAX` / `SERVO6_MAX` | 2000 | 垂直后再仰极限 |
| `SERVO5_REVERSED` / `SERVO6_REVERSED` | 0 | 左右外段同向、垂起时电机轴朝上 |
| `Q_TILT_YAW_ANGLE` | 15 | 与 MAX 对应的后仰角（度）一致 |
| `BTILT_HORIZ_L` / `BTILT_HORIZ_R` | 1200 | 固飞杆回中：左右外段各自与中段齐平（介于 MIN 与 TRIM） |
| `BTILT_REV` | 1 | 左滚 → 左减迎角、右增迎角；反了改为 `-1` |
| `BTILT_TRAVEL` | 100 | 满杆相对 HORIZ 的单侧最大偏置（µs） |
| `BTILT_GAIN` | 0.3 | 差动增益 0..1；由低到高试 |
| `SERVO7_MIN` / `TRIM` / `MAX` | 1000 / 1500 / 2000 | 平尾行程端点与中立 |
| `SERVO7_REVERSED` | 0 | 固飞俯仰方向正确 |
| `SERVO11_REVERSED` / `SERVO12_REVERSED` | 0 | 对转方向按机身要求 |
| `SERVO11/12_MIN` / `TRIM` / `MAX` | 1000 / 1000 / 2000 | 一般可沿用；电调校准区不同再微调 |

操作步骤见下文 4.1–4.3。端点语义见 [hardware.md](./hardware.md)。

### 4.1 倾转方向与 VTOL 端点

1. 模式切 **QSTABILIZE**：倾转应到垂直附近（TRIM）。
2. 必要时改 `SERVO5/6_REVERSED`，使左右外段同向、电机轴朝上。
3. 调 `SERVO5/6_TRIM` 到垂直中心；`MAX` 到垂起后仰极限（与 `Q_TILT_YAW_ANGLE` 一致）。
4. 切 **MANUAL** / **STABILIZE**（固飞）：在脚本接管前/失效时固件会把输出拉向 **MIN**。先把 `MIN` 设在「略低于水平」的机械极限。

### 4.2 固飞水平与差动（Lua）

1. 固飞模式、杆回中：分别调 `BTILT_HORIZ_L` / `BTILT_HORIZ_R`，使左右外段各自与中段**齐平**。
2. 打横滚：应出现差动。设计符号（`BTILT_REV=1`）：**向左滚** → 左外段减迎角、右外段增迎角（见 [固定翼形态-向左滚转图](./固定翼形态-向左滚转-副翼位置.jpg)）。右舵机镜像安装，脚本对左右写**同号** PWM 偏移；整体横滚反了把 `BTILT_REV` 设为 `-1`（改 `SERVO6_REVERSED` 无效，Lua 直写 PWM 绕过该参数）。
3. `BTILT_TRAVEL` / `BTILT_GAIN`：从保守值加大，避免打满杆撞机械限位。
4. 再切回 **QSTABILIZE**，确认垂起倾转正常、脚本未干扰。

### 4.3 升降舵与电机

- 升降舵：固飞俯仰方向正确；必要时 `SERVO7_REVERSED`。
- 电机：S11/S12，普通 PWM；对转方向按机身要求。垂起用固件混控；固飞推力靠脚本直通（见 §3），台架按上文「固飞油门」核对。

## 5. EdgeTX：形态 / 固飞模式 → CH8

飞控只有一路 `FLTMODE_CH`。在 Zorro 上将两路开关混成 CH8（建议 SA=形态，SB=固飞模式）：

| 形态开关 | 固飞模式开关 | 目标模式 | 建议 CH8 PWM 区 |
|----------|--------------|----------|-----------------|
| 垂起 | （忽略） | QSTABILIZE (17) | 低（如 ~1165）→ `FLTMODE1` |
| 固飞 | 自稳 | STABILIZE (2) | 中（官方三档 ~1425 或回中 ~1500）→ `FLTMODE3`/`4` |
| 固飞 | 纯手动 | MANUAL (0) | 高（如 ~1835）→ `FLTMODE6` |

ArduPilot 将 `FLTMODE_CH` PWM 划成六段；本机按低/中/高三段垫档（勿把 `17/2/0` 循环两遍，否则中位 ~1500 会落到 `FLTMODE4=QSTABILIZE`）：

| 槽位 | PWM 区间 | 本机模式 |
|------|----------|----------|
| `FLTMODE1` | ≤1230 | QSTABILIZE (17) |
| `FLTMODE2` | 1231–1360 | QSTABILIZE (17) |
| `FLTMODE3` | 1361–1490 | STABILIZE (2) |
| `FLTMODE4` | 1491–1620 | STABILIZE (2) |
| `FLTMODE5` | 1621–1749 | MANUAL (0) |
| `FLTMODE6` | ≥1750 | MANUAL (0) |

原则：

- 形态=垂起时，混控**强制**输出 QSTABILIZE 对应 PWM，与 SB 无关。
- 形态=固飞时，按 SB 在 STABILIZE / MANUAL 两档间选。
- 飞控侧已设 `FLTMODE1..6=17,17,2,2,0,0`；用 Mission Planner 看 CH8 Current PWM 与模式指示对齐即可。

完整 `.etx` 不提供；按上表在 EdgeTX 混控页自建。

## 6. 性能与安全

- Lua 固飞滚转带宽低于源码补丁；增益宁低勿高。
- 脚本未加载、报错或覆写超时 → 倾转回到固件锁定位（**MIN**，双侧略低于水平）；固飞油门也会失去直通。起飞前确认 GCS 有 BTILT 运行消息。
- 低速 / 应急：用**形态开关**切回垂起（`QSTABILIZE`）。勿在低速切固飞并停在 `MANUAL` 当应急。
- 悬停 PID、过渡速率等保持默认，试飞后再调；本仓库不提供精调值。

## 7. 推荐顺序小结

```mermaid
flowchart LR
  download[下载固件]
  flash[DFU或MP刷写]
  init[全量init含Q_ENABLE]
  reboot1[重启]
  param[导入项目param重启]
  lua[复制脚本到SD]
  calib[台架标定]
  stick[固飞横滚差动确认]
  download --> flash --> init --> reboot1 --> param --> lua --> calib --> stick
```
