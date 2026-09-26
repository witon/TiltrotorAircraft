# TiltrotorAircraft

双旋翼倾转翼（Tilt-Wing）说明。飞控默认 Matek H743-MINI V3（ArduPlane）。

机架为 Tilt-Tri vectored yaw（`--config tilttri`，默认）。尾电机为 Motor4；Lua 做固飞差动倾转和前电机油门直通。不等功率尾桨需自编译 `V4.6.3-thstfac`。

| 参数 | Lua |
|------|-----|
| [params/configs/tilttri/](params/configs/tilttri/) | [lua/tilttri_fw_tilt_aileron.lua](lua/tilttri_fw_tilt_aileron.lua) |

- 硬件构型与接线：[docs/hardware.md](docs/hardware.md)
- 飞行形态与模式：[docs/flight-modes.md](docs/flight-modes.md)
- 固件刷写（DFU / 本地下载）：[docs/matek-h743-mini-v3-flash.md](docs/matek-h743-mini-v3-flash.md)
- 刷写验证清单：[docs/flash-verify-checklist.md](docs/flash-verify-checklist.md)
- ArduPilot 设置（参数 / Lua / 标定）：[docs/ardupilot-setup.md](docs/ardupilot-setup.md)
- 全量基线：[params/init.param](params/init.param)（含 `Q_ENABLE=1`，重启后）→ [params/configs/tilttri/project.param](params/configs/tilttri/project.param)；机号 overlay：[params/configs/tilttri/aircraft/](params/configs/tilttri/aircraft/)

上传 Lua 时会删掉 SD 上已退役的 `bicopter_fw_tilt_aileron.lua`。

## 本机工具（`scripts/`）

```powershell
# 下载官方 Plane MatekH743 固件到 firmware/
.\scripts\download-matekh743-plane.ps1

# 不等功率 tilttri：从 GitHub Release 下载 V4.6.3-thstfac（无需本机编译）
.\scripts\download-thstfac-plane.ps1

# 混控前后比公式自检（不连飞控）
python scripts/tri-thst-mix-check.py

# 列出构型
python scripts/upload-params.py --list-configs

# 上传参数（先 Disconnect Mission Planner；COMx 换成实际串口）
pip install -r requirements.txt
# 全量：init → 重启 → 项目配置（默认 tilttri）
python scripts/upload-params.py --port COMx --mode full
# 增量：仅项目配置
python scripts/upload-params.py --port COMx --mode incremental
# 带 1 号机倾转标定 overlay
python scripts/upload-params.py --port COMx --mode incremental --aircraft 01

# 从已标定飞控导出机号标定文件（需已加载 Lua，否则无 BTILT_*）
python scripts/export-aircraft-calib.py --port COMx --aircraft 01

# 上传 Lua 到飞控 SD（APM/scripts/；会删除已退役的 bicopter 脚本；默认重启）
python scripts/upload-lua.py --port COMx

# 分析飞控日志并给出改参建议（先 Disconnect Mission Planner）
python scripts/analyze-log.py --port COMx --list-logs
python scripts/analyze-log.py --port COMx --latest 1 --aircraft 01
python scripts/analyze-log.py logs_download/00000091.BIN
```
