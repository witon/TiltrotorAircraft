# Matek H743-MINI V3 安装 ArduPilot（ArduPlane）

本机架项目默认使用 **ArduPlane**，板型目标为 **MatekH743**（H743-Wing / SLIM / MINI / WLITE 共用固件）。

官方板级说明：<https://ardupilot.org/plane/docs/common-matekh743-wing.html>

刷写完成后的参数、Lua 与标定见 [ardupilot-setup.md](./ardupilot-setup.md)。硬件接线见 [hardware.md](./hardware.md)。

## 固件选择

| 用途 | 文件 | 来源 |
|------|------|------|
| 首次 DFU 刷写（含 bootloader） | `arduplane_with_bl.hex` | [Plane/stable/MatekH743](https://firmware.ardupilot.org/Plane/stable/MatekH743/) |
| 已装 ArduPilot 后的升级 | `arduplane.apj` | 同上 |
| 双向 DShot（可选） | `MatekH743-bdshot` 目录下对应文件 | [Plane/stable/MatekH743-bdshot](https://firmware.ardupilot.org/Plane/stable/MatekH743-bdshot/) |

优先使用 **stable**；需要新特性再换 `latest`。确认固件含 **Scripting**（近年官方 Plane 默认包含；若无 `SCR_ENABLE`，换较新稳定版）。

本地下载可用仓库脚本：

```powershell
.\scripts\download-matekh743-plane.ps1
```

固件会保存到 `firmware/Plane/stable/MatekH743/`。可选：`-Channel latest`、`-Bdshot`。

## 准备工具（Windows）

1. [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)（推荐，自带 DFU 驱动）
2. 备选：Zadig（WinUSB）+ `dfu-util`
3. [Mission Planner](https://firmware.ardupilot.org/Tools/MissionPlanner/)
4. 支持数据传输的 USB 线

## 首次 DFU 刷写

刷写时**不要**接电池、外部 5V，也不要接会给 UART 供电的 GPS 等外设（外设上电可能导致进不了 DFU）。

1. 飞控断电，不接电池。
2. **按住 Boot 键不放**，插入 USB；PC 应识别为 STM32 BOOTLOADER / DFU。
3. 打开 STM32CubeProgrammer，连接方式选 USB，刷新后应看到 DFU 设备。
4. 打开 `firmware/Plane/stable/MatekH743/arduplane_with_bl.hex`，起始地址保持默认 `0x08000000`，执行 Download。
5. 断开 USB，松开 Boot，再正常插入 USB 上电。
6. 设备管理器应出现串口；Mission Planner 选择该 COM 口，波特率 **115200**，能连上并读到固件版本即成功。

### dfu-util 等价命令

```powershell
dfu-util -a 0 -s 0x08000000:leave -D .\firmware\Plane\stable\MatekH743\arduplane_with_bl.hex
```

### DFU 进不去时

- 确认按住的是 **Boot**，不是其它按键
- 断开所有外设与外部供电
- 换 USB 线 / USB 口
- 用 Zadig 为 `STM32 BOOTLOADER` 安装 WinUSB（不要误换 `STM32 Virtual Com Port` 的驱动）

## 后续用 .apj 升级

1. 正常 USB 连接飞控（无需按 Boot）。
2. Mission Planner → **SETUP** → **Install Firmware**。
3. 若看不到 **Load custom firmware**：Config → Planner → Layout 设为 **Advanced**。
4. 选择 `arduplane.apj` 上传（不要选 `.hex`）。

也可在 Mission Planner **Install Firmware** 中直接选 **MatekH743** → **Plane**（在线列表）；需要可复现本地文件时优先用本仓库下载的 `.apj`。

部分较新 bootloader 支持 SD 卡刷写（`ardupilot.abin`）。首次安装仍以 DFU + `with_bl.hex` 为准。

## 默认 UART 顺序（MatekH743）

| 参数 | 用途 | 硬件 |
|------|------|------|
| SERIAL0 | Console | USB |
| SERIAL1 | Telem1 | UART7（支持 CTS/RTS） |
| SERIAL2 | Telem2 | USART1 |
| SERIAL3 | GPS1 | USART2 |
| SERIAL4 | GPS2 | USART3 |
| SERIAL5 | USER | UART8 |
| SERIAL6 | USER | UART4 |
| SERIAL7 | USER | UART6（默认仅 TX；`BRD_ALT_CONFIG=1` 时 RX 可用） |

RC：默认可用 Rx6；CRSF/ELRS 等双向协议需 `BRD_ALT_CONFIG=1`，并将接收机接到 `RX6`/`TX6`（SERIAL7）。本仓库 Zorro + ER6GV 接线与参数见 [hardware.md](./hardware.md)。

本板**无内置罗盘**，自主模式需外接 GPS/Compass。电池监测参数勿直接照搬 WING 的电流计标定；按 MINI 实际硬件再调。

## DFU + Mission Planner 验证清单

快速勾选页：[flash-verify-checklist.md](./flash-verify-checklist.md)

刷写与连接时按项勾选：

### A. 刷写前

- [ ] 已安装 STM32CubeProgrammer（或 dfu-util + DFU 驱动）
- [ ] 已安装 Mission Planner
- [ ] 已下载 `arduplane_with_bl.hex`（运行 `.\scripts\download-matekh743-plane.ps1`）
- [ ] 飞控未接电池 / 外部 5V / GPS 等外设
- [ ] USB 线可传数据

### B. DFU 刷写

- [ ] 按住 Boot 插 USB，PC 识别为 STM32 BOOTLOADER / DFU
- [ ] STM32CubeProgrammer 能连接 DFU 设备
- [ ] 已烧录 `arduplane_with_bl.hex`，地址 `0x08000000`，无报错
- [ ] 松开 Boot 后重新上电（正常插 USB）

### C. Mission Planner 连接

- [ ] 设备管理器出现新 COM 口
- [ ] Mission Planner 以 115200 连接成功
- [ ] 显示 **ArduPlane**，板型相关信息可对应 MatekH743
- [ ] 能读取固件版本号（记录下方）

**记录：**

| 项目 | 值 |
|------|-----|
| COM 口 | |
| 固件版本 | |
| 刷写日期 | |

### D. 刷写后快速确认（固件可运行即可）

- [ ] HUD / 姿态有响应（轻微倾斜飞控时姿态变化）
- [ ] 加速度计水平校准完成
- [ ] IMU / 气压计无持续 Error（罗盘可待外接 GPS 后再校）
- [ ] 未照搬 WING 电流计参数到 MINI（按实际再配）

倾转旋翼参数与 Lua 部署见 [ardupilot-setup.md](./ardupilot-setup.md)；官方说明：<https://ardupilot.org/plane/docs/guide-tilt-rotor.html>

## 风险说明

- 刷入 `with_bl.hex` 会写入 ArduPilot bootloader，后续主要走 ArduPilot 升级路径；若要切回 Betaflight / INAV，需再次 DFU 刷对应固件。
- `MatekH743-bdshot` 会改变 Rx6 与部分 PWM 行为，使用前阅读官方 Warning。
- H7 偶发起机异常时，查阅官方文档 *When Problems Arise* 中 H7 相关说明。
