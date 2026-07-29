# TiltrotorAircraft

双旋翼倾转翼（BiCopter / Tilt-Wing）说明。飞控默认 Matek H743-MINI V3（ArduPlane）。

- 硬件构型与接线：[docs/hardware.md](docs/hardware.md)
- 飞行形态与模式：[docs/flight-modes.md](docs/flight-modes.md)
- 固件刷写（DFU / 本地下载）：[docs/matek-h743-mini-v3-flash.md](docs/matek-h743-mini-v3-flash.md)
- 刷写验证清单：[docs/flash-verify-checklist.md](docs/flash-verify-checklist.md)
- ArduPilot 设置（参数 / Lua / 标定）：[docs/ardupilot-setup.md](docs/ardupilot-setup.md)
- 参数：全量 [params/init.param](params/init.param)（含 `Q_ENABLE=1`，重启后）→ [params/matek-h743-mini-bicopter.param](params/matek-h743-mini-bicopter.param)；增量仅后者
- 固飞差动倾转脚本：[scripts/bicopter_fw_tilt_aileron.lua](scripts/bicopter_fw_tilt_aileron.lua)

## 脚本

```powershell
# 下载官方 Plane MatekH743 固件到 firmware/
.\scripts\download-matekh743-plane.ps1

# 上传参数（先 Disconnect Mission Planner；COMx 换成实际串口）
pip install -r requirements.txt
# 全量：init（基线 + Q_ENABLE=1）→ 重启 → 项目配置
python scripts/upload-params.py --port COMx --mode full
# 增量：仅项目配置
python scripts/upload-params.py --port COMx --mode incremental
```
