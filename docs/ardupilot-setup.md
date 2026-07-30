# ArduPilot 设置（BiCopter + Lua 固飞差动）

本机：Matek **H743-MINI V3** + 官方 **ArduPlane**（BiCopter）+ Lua 固飞等效副翼。硬件与接线见 [hardware.md](./hardware.md)，模式见 [flight-modes.md](./flight-modes.md)。

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
| 模式 | `FLTMODE_CH=8`；`FLTMODE1=17`，`2=2`，`3=0`（其余垫档） |
| 输出 | S5=75，S6=76，S7=19，S11=73，S12=74 |

`Q_TILT_YAW_ANGLE`、倾转 `SERVO*_MIN/TRIM/MAX`、`BTILT_*` 为占位，台架后改写。本项目不做电池监测标定与罗盘校准。

## 3. 部署 Lua 脚本

1. 飞控插入 MicroSD，目录：`APM/scripts/`（若无则新建）。
2. 复制 [`scripts/bicopter_fw_tilt_aileron.lua`](../scripts/bicopter_fw_tilt_aileron.lua) 到该目录。
3. 确认 `SCR_ENABLE=1`，重启飞控。
4. GCS 消息应出现类似：`BTILT: fw differential tilt running`。
5. Full Parameter List 中应出现脚本表参数：

| 参数 | 默认 | 含义 |
|------|------|------|
| `BTILT_HORIZ` | 1200 | 真水平 PWM（固飞中心） |
| `BTILT_TRAVEL` | 100 | 满杆时相对 HORIZ 的单侧最大偏置（µs） |
| `BTILT_GAIN` | 0.3 | 差动增益 0..1（由低到高试） |
| `BTILT_REV` | 1 | `1` 或 `-1`，反了改符号 |

脚本仅在 **`STABILIZE`(2)** / **`MANUAL`(0)** 覆写倾转；**`QSTABILIZE`** 不覆写。

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
| `BTILT_HORIZ` | 1200 | 固飞杆回中：外段与中段齐平（介于 MIN 与 TRIM） |
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

1. 固飞模式、杆回中：调 `BTILT_HORIZ`，使左右外段与中段**齐平**。
2. 打横滚：应出现差动。设计符号（`BTILT_REV=1`）：**向左滚** → 左外段减迎角、右外段增迎角（见 [固定翼形态-向左滚转图](./固定翼形态-向左滚转-副翼位置.jpg)）。反了把 `BTILT_REV` 设为 `-1`。
3. `BTILT_TRAVEL` / `BTILT_GAIN`：从保守值加大，避免打满杆撞机械限位。
4. 再切回 **QSTABILIZE**，确认垂起倾转正常、脚本未干扰。

### 4.3 升降舵与电机

- 升降舵：固飞俯仰方向正确；必要时 `SERVO7_REVERSED`。
- 电机：S11/S12，普通 PWM；对转方向按机身要求，拆桨用电机测试确认。

## 5. EdgeTX：形态 / 固飞模式 → CH8

飞控只有一路 `FLTMODE_CH`。在 Zorro 上将两路开关混成 CH8（建议 SA=形态，SB=固飞模式）：

| 形态开关 | 固飞模式开关 | 目标模式 | 建议 CH8 PWM 区 |
|----------|--------------|----------|-----------------|
| 垂起 | （忽略） | QSTABILIZE (17) | 低（如 ~1165） |
| 固飞 | 自稳 | STABILIZE (2) | 中（如 ~1500） |
| 固飞 | 纯手动 | MANUAL (0) | 高（如 ~1835） |

原则：

- 形态=垂起时，混控**强制**输出 QSTABILIZE 对应 PWM，与 SB 无关。
- 形态=固飞时，按 SB 在 STABILIZE / MANUAL 两档间选。
- 飞控侧 `FLTMODE1..6` 已按 17/2/0 垫档；PWM 阈值与 Mission Planner 模式指示对齐即可。

完整 `.etx` 不提供；按上表在 EdgeTX 混控页自建。

## 6. 性能与安全

- Lua 固飞滚转带宽低于源码补丁；增益宁低勿高。
- 脚本未加载、报错或覆写超时 → 倾转回到固件锁定位（**MIN**，双侧略低于水平）。起飞前确认 GCS 有 BTILT 运行消息。
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
