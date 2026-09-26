# ArduPilot 设置（Tilt-Tri）

本机：Matek **H743-MINI V3** + **ArduPlane**。焊盘接线见 [hardware.md](./hardware.md)，模式见 [flight-modes.md](./flight-modes.md)。机架为 Tilt-Tri vectored yaw（`--config tilttri`，默认）：

| 机架 | Lua |
|------|-----|
| `Q_FRAME_CLASS=7`，`Q_TILT_TYPE=2`；尾电机 Motor4；Lua 固飞差动倾转 + 前电机油门直通（无空速/高度，不走等空速定高）。前后电机功率不同时用自编译 `V4.6.3-thstfac` 的 `Q_M_THST_FRONT` / `Q_M_THST_REAR` | [`tilttri_fw_tilt_aileron.lua`](../lua/tilttri_fw_tilt_aileron.lua) |

参数在 [`params/configs/tilttri/`](../params/configs/tilttri/)。`upload-lua.py` 会删除 SD 上已退役的 `bicopter_fw_tilt_aileron.lua`，避免和当前脚本抢输出。

等功率三旋翼可用官方 Plane。前对大电机 + 小尾桨需要 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md) 里的 **V4.6.3-thstfac** 自定义固件（只编译一次；比例在地面站改参）。

## 1. 刷固件

推荐按 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md) 操作（本地下载 + 首次 DFU）。摘要：

1. 下载官方固件（等功率）：
   ```powershell
   .\scripts\download-matekh743-plane.ps1
   ```
   不等功率：在 GitHub Actions 编译后下载（本机不必装 WSL）：
   ```powershell
   .\scripts\download-thstfac-plane.ps1
   ```
   工作流：仓库 **Actions** → **Build MatekH743 Plane** → **Run workflow**。说明见 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md)。
2. **首次**：按住 Boot，用 STM32CubeProgrammer（或 dfu-util）烧写 `firmware/Plane/stable/MatekH743/arduplane_with_bl.hex`（官方）或 `firmware/Plane/custom/MatekH743/arduplane_with_bl.hex`（thstfac）。
3. **已装 ArduPilot 后升级**：Mission Planner → **Install Firmware** → Load custom firmware → 对应目录的 `arduplane.apj`。**不等功率不要从在线列表选 MatekH743 Plane**，会盖掉 `THST_FRONT/REAR` 混控。
4. 自定义固件 GCS 应显示 **ArduPlane V4.6.3-thstfac**。官方固件含 **Scripting**（近年 Plane 默认包含；若无 `SCR_ENABLE`，换较新稳定版）。
5. 刷写完成后连接飞控（115200），不要急着装桨。验证清单：[flash-verify-checklist.md](./flash-verify-checklist.md)。

## 2. 导入参数

两种上传方式：

| 模式 | 用途 | 顺序 |
|------|------|------|
| **全量** | 新板 / 需可复现基线 | [`init.param`](../params/init.param)（默认快照且 `Q_ENABLE=1`）→ **重启**（`Q_*` 出现）→ [`params/configs/tilttri/project.param`](../params/configs/tilttri/project.param) →（可选）[`params/configs/tilttri/aircraft/NN.param`](../params/configs/tilttri/aircraft/) → 重启 |
| **增量** | 已加载本项目、只改项目差异 | 仅写 project（飞控上须已有 `Q_*`）；可选再写机号 overlay |

`init.param` 来自恢复默认后的导出，并把 `Q_ENABLE` 置为 1，因此全量不再需要单独的 q-enable 文件。已用官方 Plane 参数元数据去掉 `Volatile` / `ReadOnly` / `Calibration` 项（见 [`init.param.removed.txt`](../params/init.param.removed.txt)；重跑：`python scripts/filter-init-params.py`）。未开 `Q_ENABLE` 时 `Q_TILT_*` 不在参数表中，故全量必须在 init 之后重启再写项目配置。

项目 param 中的 `SERVO5/6_*` 端点与 `Q_TILT_YAW_ANGLE` 为占位。台架标定后按机号入库到 `params/configs/tilttri/aircraft/NN.param`（含 `BTILT_HORIZ_L/R`），上传时用 `--aircraft NN` 叠加写入。

### 2.1 Mission Planner（GUI）

**全量：**

1. Full Parameter List → **Load from file** → `init.param` → Write → **重启** → Refresh。
2. 再 Load → `params/configs/tilttri/project.param` → Write → **重启**。
3. 按 [hardware.md](./hardware.md) 完成 Accel **水平校准**（竖装 Roll90 后必做）。

**增量：** 只做第 2 步（并确认 `Q_*` 已存在）。

### 2.2 CLI（`upload-params.py`）

先在 Mission Planner 中 **Disconnect**（串口不能被占用），然后：

```powershell
pip install -r requirements.txt
python scripts/upload-params.py --list-configs
# 全量（脚本内会在 init 后自动重启并等待，再写项目配置；默认 tilttri）
python scripts/upload-params.py --port COMx --mode full
# 增量
python scripts/upload-params.py --port COMx --mode incremental
# 全量 / 增量 + 1 号机倾转标定 overlay
python scripts/upload-params.py --port COMx --mode full --aircraft 01
python scripts/upload-params.py --port COMx --mode incremental --aircraft 01
```

默认波特率 115200；`--no-reboot` 可跳过**最终**重启（全量中间的 init 后重启仍会执行）。

### 2.3 按机号导出标定（`export-aircraft-calib.py`）

台架标定完成后，从飞控读出并写入 `params/configs/tilttri/aircraft/NN.param`（机号两位，如 `01`）：

| 参数 | 说明 |
|------|------|
| `SERVO5/6_MIN` / `TRIM` / `MAX` / `REVERSED` | 倾转舵机行程与中位 |
| `Q_TILT_YAW_ANGLE` | 与 MAX 后仰角一致 |
| `BTILT_HORIZ_L` / `BTILT_HORIZ_R` | 固飞真水平（须已加载 Lua） |

```powershell
# 须已部署 Lua 并重启，否则无 BTILT_*，脚本会中止且不写文件
python scripts/export-aircraft-calib.py --port COMx --aircraft 01
```

之后对该机上传参数时带 `--aircraft 01`。新机标定完同样 export 为 `02.param` 等。

### 参数摘要（共用）

| 类别 | 关键项 |
|------|--------|
| 垂起姿态限幅 | `Q_OPTIONS=16384`（bit14：Q 模式忽略固飞 `PTCH_LIM_*` / `ROLL_LIMIT_DEG`），`Q_ANGLE_MAX=4500`（Plane 4.6：百分度，4500=45°） |
| 垂起手感 | `Q_M_THST_EXPO=0.80`，`Q_M_SLEW_UP_TIME=0.8`，`Q_M_SPIN_MIN=0.12`，`Q_A_INPUT_TC=0.25`，`Q_A_ANG_RLL/PIT_P=3.5` |
| 姿态 | `AHRS_ORIENTATION=16` |
| CRSF | `BRD_ALT_CONFIG=1`，`SERIAL7_PROTOCOL=23` |
| 脚本 | `SCR_ENABLE=1` |
| 模式 | `FLTMODE_CH=8`；`FLTMODE1..6=17,17,2,2,0,0` |
| PWM | `Q_M_PWM_TYPE=0`（S8 与舵机同组，禁止 DShot） |
| 无 GPS/罗盘 | `COMPASS_ENABLE=0`，`GPS1_TYPE=0`，`AHRS_GPS_USE=0`，`EK3_SRC1_POSXY/VELXY/VELZ/YAW=0`，`ARMING_CHECK=1048562`，`ARMING_RUDDER=2` |

### 参数摘要（机架）

| 类别 | 关键项 |
|------|--------|
| 机架 | `Q_FRAME_CLASS=7`，`Q_TILT_TYPE=2`，`Q_TILT_MASK=3`，`Q_TILT_RATE_UP/DN=90`，`Q_ASSIST_SPEED=-1`，`SCHED_LOOP_RATE=300` |
| 输出 | S5=75，S6=76，S7=19，S8=**36**（Motor4，`MIN`/`TRIM=1000`），S11=**34**，S12=**33** |
| 尾电调 | 单向 PWM；停转在 MIN；约 50 Hz |
| 前后推力比 | 需固件 `V4.6.3-thstfac`。`Q_M_THST_FRONT` / `Q_M_THST_REAR`：1.0 为等功率；本仓库 project 为 0.8 / 1.2（01 号机台架）。同一油门下前对 PWM 仍明显高于尾则再降 FRONT；尾贴怠速且 S8 未到 MAX 可略升 REAR。写入 `aircraft/NN.param`，不必再编译。 |

`Q_TILT_YAW_ANGLE`、倾转 `SERVO*_MIN/TRIM/MAX`、`BTILT_*` 为占位，台架后改写，并用 §2.3 导出到机号文件。本项目不做电池监测标定与罗盘校准。

H743-MINI 无内置罗盘；项目默认按**姿态模式**运行（无外置 GPS/罗盘），可解锁台架与 `QSTABILIZE` / `STABILIZE` / `MANUAL`。勿使用需定位的模式（`AUTO` / `RTL` / `QLOITER` / `QRTL` 等）；无罗盘时偏航会漂，垂起偏航保持较差。日后外接 GPS+罗盘时：恢复 `GPS1_TYPE`、打开罗盘、还原 `EK3_SRC1_*`（水平位置/速度用 GPS，航向用罗盘）、`ARMING_CHECK=1`（或 4.7+ 的 `ARMING_SKIPCHK=0`），并完成罗盘校准与 GPS 定位后再飞自主模式。

## 3. 部署 Lua 脚本

### 3.1 CLI（`upload-lua.py`，推荐）

先 Disconnect Mission Planner，再：

```powershell
python scripts/upload-lua.py --port COMx
```

脚本经 MAVFTP 写入飞控 SD 的 `APM/scripts/`（目录不存在会创建），并删除已退役的 `bicopter_fw_tilt_aileron.lua`，检查 `SCR_ENABLE`，默认重启以加载脚本。跳过重启：`--no-reboot`。其它文件：`--script path\to\file.lua`。

### 3.2 手动拷 SD（备选）

1. 飞控插入 MicroSD，目录：`APM/scripts/`（若无则新建）。
2. 复制 [`tilttri_fw_tilt_aileron.lua`](../lua/tilttri_fw_tilt_aileron.lua) 到该目录；删掉 `bicopter_fw_tilt_aileron.lua`（若仍在）。
3. 确认 `SCR_ENABLE=1`，重启飞控。

### 3.3 验证

1. GCS 消息应出现类似：`BTILT: tilttri fw tilt+throttle running`。
2. Full Parameter List 中应出现脚本表参数（**无** `BPIT_*`）：

| 参数 | 默认 | 含义 |
|------|------|------|
| `BTILT_HORIZ_L` | 1200 | 左倾转真水平 PWM（固飞中心） |
| `BTILT_HORIZ_R` | 1200 | 右倾转真水平 PWM（固飞中心） |
| `BTILT_TRAVEL` | 100 | 满杆时相对 HORIZ 的单侧最大偏置（µs） |
| `BTILT_GAIN` | 0.12 | 倾转差动增益 0..1（由低到高试） |
| `BTILT_REV` | 1 | 倾转横滚符号：`1` 或 `-1`，反了改符号 |
| `BTILT_THR` | 1 | `1` 固飞前电机按油门杆直通；`0` 仅倾转 |
| `BTILT_YAWDT` | 0.1 | 固飞偏航差动增益 -1..1（负号反转） |

倾转参数用表键 89，`BTILT_HORIZ_R` 用表键 101，`BTILT_THR` / `BTILT_YAWDT` 用表键 100。不注册尾桨表（102–104）。

3. `QSTABILIZE` 解锁：S8/S11/S12 由混控驱动（尾电机跟俯仰/油门；S8 在 1000 以上怠速）。脚本不覆写电机。
4. `MANUAL` / `STABILIZE` Arm、油门最低：S11/S12 在各自 MIN 附近（电机基本停），S8 在 MIN（1000）附近停转。推油门后 S11/S12 一起升高；打偏航左右差动（`BTILT_YAWDT`，默认 0.1；反了改符号）。这是 Lua 按油门杆直通，不跟悬停油门。本机无空速计、无高度计；若交给固件，`STABILIZE` 会停在等空速的定高过渡里，主电机维持大约一半油门。
5. `QSTABILIZE` → 固飞时，脚本按 `Q_TILT_RATE_DN`（为 0 则用 `Q_TILT_RATE_UP`）将倾转扫到 `BTILT_HORIZ_*`，扫角期间即可差动。固飞打横滚 → S5/S6 差动。切回 `QSTABILIZE` 后 Lua 立即松手，三电机交回混控，倾转由固件按 `Q_TILT_RATE_UP` 收到垂直。
6. 尾电调为单向 PWM（非 3D）；`SERVO8_MIN`/`TRIM=1000`。若电调仍是双向且 MIN=1000，油门最低会进反转区乱转。

## 4. 台架标定（拆桨）

### 需标定参数一览

项目 param 中倾转 / `BTILT_*` / 升降舵端点为占位，须台架改写后再飞。倾转端点与 `BTILT_HORIZ_*` 标定完成后用 `export-aircraft-calib.py` 按机号入库（见 §2.3）。悬停 PID 保持默认，试飞后再调；过渡速率见 §6（约 1 s）。

**地面（台架前）：**

| 项 | 相关参数 | 说明 |
|----|----------|------|
| 加速度计水平校准 | `INS_ACC*` 等（MP 向导） | 设好 `AHRS_ORIENTATION=16` 并重启后必做；HUD 与机身一致 |
| 遥控行程校准 | `RCn_MIN` / `TRIM` / `MAX` | 链路通后按实际杆量校准；CH8 与模式档对齐 |

**台架（拆桨）：**

| 参数 | 占位默认 | 标定目标 |
|------|----------|----------|
| `SERVO5_MIN` / `SERVO6_MIN` | 1100 | 机械「水平以下」极限（TYPE=2 的 fully-fwd 侧）；尽量靠近水平 |
| `SERVO5_TRIM` / `SERVO6_TRIM` | 1500 | 不作为固件垂起中心。01 号机标在垂直附近（左 1150 / 右 1950），配合 `REVERSED` 使 QSTABILIZE 落在 TRIM |
| `SERVO5_MAX` / `SERVO6_MAX` | 2000 | 垂直后再仰极限 |
| `SERVO5_REVERSED` / `SERVO6_REVERSED` | project 1 / 0；01 号机 0 / 1 | 左右外段同向、垂起电机轴朝上，且 QSTABILIZE 落在垂直附近 |
| `Q_TILT_YAW_ANGLE` | 15 | 与 MAX 对应的后仰角（度）一致；垂直约在 `YAW_ANGLE/(90+YAW_ANGLE)` |
| `BTILT_HORIZ_L` / `BTILT_HORIZ_R` | 1200 | 固飞杆回中：左右外段各自与中段齐平（01：左 2000 / 右 1090） |
| `BTILT_REV` | 1 | 左滚 → 左减迎角、右增迎角；反了改为 `-1` |
| `BTILT_TRAVEL` | 100 | 满杆相对 HORIZ 的单侧最大偏置（µs） |
| `BTILT_GAIN` | 0.12 | 差动增益 0..1；由低到高试 |
| `SERVO7_MIN` / `TRIM` / `MAX` | 1000 / 1500 / 2000 | 平尾行程端点与中立 |
| `SERVO7_REVERSED` | 0 | 固飞俯仰方向正确 |
| `SERVO8_MIN` / `TRIM` / `MAX` | 1000 / 1000 / 2000 | Motor4 单向电调；MIN/TRIM=停转 |
| `Q_M_THST_FRONT` / `Q_M_THST_REAR` | 0.8 / 1.2 | 前后 PWM 比例；等功率改为 1.0。须 `V4.6.3-thstfac` |
| `SERVO11_REVERSED` / `SERVO12_REVERSED` | 0 | 对转方向按机身要求 |
| `SERVO11/12_MIN` / `TRIM` / `MAX` | 1000 / 1000 / 2000 | 一般可沿用；电调校准区不同再微调 |

操作步骤见下文 4.1–4.4。端点语义见 [hardware.md](./hardware.md)。参数表无 `BPIT_*`。

### 4.1 倾转方向与 VTOL 端点

`Q_TILT_TYPE=2` 把 `MIN`↔`MAX` 当成「后仰极限 ↔ 水平以下」，**不用 TRIM 当垂起中心**。垂直约在 `Q_TILT_YAW_ANGLE/(90+YAW_ANGLE)`。全量占位 1100/1500/2000 会让 QSTABILIZE 远离已标定垂直。01 号机 `SERVO5_REVERSED=0`、`SERVO6_REVERSED=1`，使 QSTABILIZE 落在 TRIM 附近（左 1150 / 右 1950）。固飞仍由 Lua `BTILT_HORIZ_*` 覆写。

1. 模式切 **QSTABILIZE**、杆回中：倾转应在垂直附近；三只电机混控。
2. 必要时改 `SERVO5/6_REVERSED`，使左右外段同向、电机轴朝上。
3. 调 MIN/MAX 覆盖「水平以下极限 ↔ 垂直后再仰」，`Q_TILT_YAW_ANGLE` 与 MAX 侧后仰角一致。垂直两侧都要留出偏航矢量行程，否则大偏航时一侧顶死。
4. 切 **MANUAL** / **STABILIZE**：脚本把倾转收到各自 `BTILT_HORIZ_*`。脚本失效时失去固飞差动，垂起混控仍在。

### 4.1.1 垂起俯仰权威与大倾角排查（拆桨）

垂起俯仰靠**前对 vs 尾 Motor4 差推力**（不是平尾主控）。偏航才是左右倾转差动。稳态 `QSTABILIZE` 下 Lua 不覆写倾转和三电机。默认若不设 `Q_OPTIONS` bit14，Q 模式俯仰目标会被固飞 `PTCH_LIM_MAX_DEG` / `PTCH_LIM_MIN_DEG` 卡住（本仓库 init 约为 +20° / −25°），满杆 DesPitch 上不去，大倾角后易饱和发散。

上传项目 param 后确认飞控上有 `Q_OPTIONS=16384`、`Q_ANGLE_MAX=4500`（Plane 4.6；若参数表为 `Q_A_ANGLE_MAX` 则应为 45），然后：

1. **QSTABILIZE**、杆回中：倾转在垂直附近；S8/S11/S12 由混控驱动。
2. 慢打俯仰满杆：Mission Planner **DesPitch** 应能到约 **±45°**（仍卡在 ~20° 说明 bit14 / `Q_ANGLE_MAX` 未生效，重新写参并重启）。S8 与前对的推力差应明显变化。
3. 打偏航看 S5/S6 PWM：双向都有足够行程；一侧几乎不动或很快顶死 → 按 §4.1 重标 `SERVO5/6_*`，并令 `Q_TILT_YAW_ANGLE` 与 MAX 侧后仰角一致。01 号机 overlay 见 [`params/configs/tilttri/aircraft/01.param`](../params/configs/tilttri/aircraft/01.param)；TRIM 贴边则重标后再 `export-aircraft-calib.py`。
4. 仍发散时再调 `Q_A_RAT_PIT_*` 与 `Q_M_THST_FRONT/REAR`（见 §4.4）。勿先靠加大升降舵混控救垂起俯仰。

### 4.2 固飞水平与差动（Lua）

1. 固飞模式、杆回中：分别调 `BTILT_HORIZ_L` / `BTILT_HORIZ_R`，使左右外段各自与中段**齐平**。
2. 打横滚：应出现差动（含 `QSTABILIZE` → 固飞扫角未结束时）。设计符号（`BTILT_REV=1`）：**向左滚** → 左外段减迎角、右外段增迎角（见 [固定翼形态-向左滚转图](./固定翼形态-向左滚转-副翼位置.jpg)）。右舵机镜像安装，脚本对左右写**同号** PWM 偏移；整体横滚反了把 `BTILT_REV` 设为 `-1`（改 `SERVO6_REVERSED` 无效，Lua 直写 PWM 绕过该参数）。
3. `BTILT_TRAVEL` / `BTILT_GAIN`：从保守值加大，避免打满杆撞机械限位。
4. 再切回 **QSTABILIZE**：Lua 立即松手，固件按 `Q_TILT_RATE_UP` 收到垂直；到位后偏航矢量由固件控制。

### 4.3 升降舵与电机

- 升降舵：固飞俯仰方向正确；必要时 `SERVO7_REVERSED`。
- 主机：S11/S12，普通 PWM；对转方向按机身要求。垂起用固件混控；固飞前电机由 Lua 按油门杆直通（`BTILT_THR=1`，见 §3）。台架：固飞油门最低时 S11/S12 在 MIN 附近，推油门才升高。

### 4.4 垂起尾电机与前后推力比（S8，拆桨）

尾电机桨轴固定朝上，单向 PWM；`SERVO8_FUNCTION=36`，停转在 `SERVO8_MIN`/`TRIM=1000`。垂起由 QuadPlane 混控，固飞由 Lua 写在 MIN。离开固飞后三电机交回混控。S8 与升降舵同组，约 50 Hz，勿开 DShot。勿再用双向电调：MIN=1000 会进反转区。

1. 锁定或固飞：S8 ≈ 1000，尾电机不转。固飞打俯仰、推油门时 S8 仍保持约 1000。
2. **QSTABILIZE** 解锁、油门最低：S8 在 1000 以上怠速，随油门与俯仰变化（尾 vs 前对推力差）。
3. **前后电机功率不同**（本机小有刷尾桨）：必须刷 `ArduPlane V4.6.3-thstfac`。拆桨后改 `Q_M_THST_FRONT` / `Q_M_THST_REAR`（project 为 0.8 / 1.2）。同一油门下前对 PWM 仍明显高于尾则降低 FRONT；打俯仰时前对 PWM 变化也应同比缩小。尾贴怠速但 S8 未到 MAX 可略升 REAR。写好后存 `aircraft/NN.param`，用 `upload-params.py --mode incremental --aircraft NN`。系数只改 PWM，补不出尾电机没有的推力。官方 4.6.3 无这两项参数。混控公式自检（不连飞控）：`python scripts/tri-thst-mix-check.py`。
4. 标定导出：`export-aircraft-calib.py --aircraft NN`。

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
- 脚本未加载、报错或覆写超时 → 失去固飞差动，且固飞自稳油门回到固件等空速定高（主电机可能维持悬停油门）；垂起混控仍在。起飞前确认 GCS 有 `BTILT: tilttri fw tilt+throttle running`。SD 上若仍有 `bicopter_fw_tilt_aileron.lua`，用 `upload-lua.py` 删掉。
- 低速 / 应急：用**形态开关**切回垂起（`QSTABILIZE`）。勿在低速切固飞并停在 `MANUAL` 当应急。
- 悬停 PID 保持默认角度/速率环结构，试飞后再微调 D/FF。项目 param 已把**手感**放软：`Q_M_THST_EXPO=0.80`、`Q_M_SLEW_UP_TIME=0.8`、`Q_M_SPIN_MIN=0.12`（离地不那么窜），`Q_A_INPUT_TC=0.25`、`Q_A_ANG_RLL/PIT_P=3.5`（打舵不那么贼）。`Q_ANGLE_MAX` 仍为 45°，满杆改出能力保留。若怠速不跟转，把 `Q_M_SPIN_MIN` 改回 0.15。
- 过渡速率项目 param 已设 `Q_TILT_RATE_UP=90`、`Q_TILT_RATE_DN=90`（°/s），水平↔垂直约 90° 行程约 **1 s**；若实机角行程偏差可再微调。去固飞时 Lua 用 `Q_TILT_RATE_DN`（为 0 则用 UP）；回垂起由固件 `Q_TILT_RATE_UP`。
- 垂起大俯仰后杆量纠正不回：先查 DesPitch 是否被 `PTCH_LIM_*` 卡住（应用 `Q_OPTIONS` bit14 + `Q_ANGLE_MAX=4500`），再查倾转 PWM 是否饱和（见 §4.1.1），并确认尾 Motor4 与 `Q_M_THST_FRONT/REAR`（见 §4.4）。

## 7. 日志诊断（`analyze-log.py`）

USB 连上飞控后，可把机上 `APM/LOGS/*.BIN` 拉下来，对照 tilttri 参数（及可选机号 overlay）给出改参建议：解锁失败、Lua 是否加载、参数漂移、倾转占位、振动、姿态跟踪、S5/S6/S8/S11/S12 饱和等。

```powershell
# 列出机上日志
python scripts/analyze-log.py --port COMx --list-logs
# 下最新 1 条并分析（1 号机）
python scripts/analyze-log.py --port COMx --latest 1 --aircraft 01
# 指定编号，或只分析已下载文件
python scripts/analyze-log.py --port COMx --log-id 91
python scripts/analyze-log.py logs_download\00000091.BIN --aircraft 01
```

下载目录默认 `logs_download/`（不入库）。脚本只给建议，不会改飞控参数。无 GPS 时间戳的台架日志仍可分析 MSG/PARM/PWM。油门离地猛、打舵贼，会分别提示 `Q_M_SLEW_UP_TIME` / `Q_M_THST_EXPO` 与 `Q_A_INPUT_TC` / `Q_A_ANG_*_P`。

## 8. 推荐顺序小结

```mermaid
flowchart LR
  download[下载固件]
  flash[DFU或MP刷写]
  init[全量init含Q_ENABLE]
  reboot1[重启]
  param["导入tilttri/project.param"]
  ac[可选机号overlay]
  lua[upload-lua]
  calib[台架标定]
  export[export机号param]
  stick[固飞横滚差动确认]
  download --> flash --> init --> reboot1 --> param --> ac --> lua --> calib --> export --> stick
```
