# TiltrotorAircraft

双旋翼倾转翼（Tilt-Wing）说明。飞控默认 Matek H743-MINI V3（ArduPlane）。

仓库支持多种机架配置，用 `--config` 选择。焊盘接线相同，`SERVOn_FUNCTION` 与混控不同。

| `--config` | 说明 | 参数 | Lua |
|------------|------|------|-----|
| `bicopter`（默认） | BiCopter + Lua 尾桨俯仰 | [params/configs/bicopter/](params/configs/bicopter/) | [lua/bicopter_fw_tilt_aileron.lua](lua/bicopter_fw_tilt_aileron.lua) |
| `tilttri` | Tilt-Tri vectored yaw；尾电机为 Motor4；Lua 只做固飞差动倾转 | [params/configs/tilttri/](params/configs/tilttri/) | [lua/tilttri_fw_tilt_aileron.lua](lua/tilttri_fw_tilt_aileron.lua) |

- 硬件构型与接线：[docs/hardware.md](docs/hardware.md)
- 飞行形态与模式：[docs/flight-modes.md](docs/flight-modes.md)
- 固件刷写（DFU / 本地下载）：[docs/matek-h743-mini-v3-flash.md](docs/matek-h743-mini-v3-flash.md)
- 刷写验证清单：[docs/flash-verify-checklist.md](docs/flash-verify-checklist.md)
- ArduPilot 设置（参数 / Lua / 标定）：[docs/ardupilot-setup.md](docs/ardupilot-setup.md)
- 全量基线：[params/init.param](params/init.param)（含 `Q_ENABLE=1`，重启后）→ `params/configs/<id>/project.param`；机号 overlay：`params/configs/<id>/aircraft/`

切换构型必须 `--mode full` 并上传对应 Lua（会删掉其它构型脚本）。不要把 BiCopter 的机号 calib 套到 `tilttri`。

## 本机工具（`scripts/`）

```powershell
# 下载官方 Plane MatekH743 固件到 firmware/
.\scripts\download-matekh743-plane.ps1

# 列出构型
python scripts/upload-params.py --list-configs

# 上传参数（先 Disconnect Mission Planner；COMx 换成实际串口）
pip install -r requirements.txt
# 全量：init → 重启 → 项目配置（默认 bicopter）
python scripts/upload-params.py --port COMx --mode full
python scripts/upload-params.py --port COMx --config tilttri --mode full
# 增量：仅项目配置
python scripts/upload-params.py --port COMx --mode incremental
# 带 1 号机倾转标定 overlay
python scripts/upload-params.py --port COMx --config bicopter --mode incremental --aircraft 01

# 从已标定飞控导出机号标定文件（需已加载该构型 Lua，否则无 BTILT_*）
python scripts/export-aircraft-calib.py --port COMx --config bicopter --aircraft 01
python scripts/export-aircraft-calib.py --port COMx --config tilttri --aircraft 01

# 上传 Lua 到飞控 SD（APM/scripts/；会删除其它构型脚本；默认重启）
python scripts/upload-lua.py --port COMx --config bicopter
python scripts/upload-lua.py --port COMx --config tilttri
```
