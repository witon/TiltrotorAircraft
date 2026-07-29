# DFU + Mission Planner 验证清单（快速页）

完整说明见 [matek-h743-mini-v3-flash.md](./matek-h743-mini-v3-flash.md)。

## A. 刷写前

- [ ] 已安装 STM32CubeProgrammer（或 dfu-util + DFU 驱动）
- [ ] 已安装 Mission Planner
- [ ] 已下载固件：`.\scripts\download-matekh743-plane.ps1`
- [ ] 飞控未接电池 / 外部 5V / GPS 等外设
- [ ] USB 线可传数据

## B. DFU 刷写

- [ ] 按住 Boot 插 USB → STM32 BOOTLOADER / DFU
- [ ] STM32CubeProgrammer 连接 DFU 成功
- [ ] 烧录 `firmware/Plane/stable/MatekH743/arduplane_with_bl.hex`（`0x08000000`）无报错
- [ ] 松开 Boot，重新正常插 USB 上电

## C. Mission Planner

- [ ] 设备管理器出现 COM 口
- [ ] 115200 连接成功
- [ ] 显示 ArduPlane / MatekH743
- [ ] 已记录固件版本

| 项目 | 值 |
|------|-----|
| COM 口 | |
| 固件版本 | |
| 刷写日期 | |

## D. 快速确认

- [ ] 姿态响应正常
- [ ] 加速度计校准完成
- [ ] IMU / 气压计无持续 Error
- [ ] 未误用 WING 电流计参数

刷写完成后导参与 Lua：[ardupilot-setup.md](./ardupilot-setup.md)
