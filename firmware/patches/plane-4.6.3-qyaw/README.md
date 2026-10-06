# Plane-4.6.3 过渡 vectored yaw 补丁

叠在前后推力比补丁之后，不改 `AP_MotorsTri`，也不改 Lua 扫角。

`QSTABILIZE` 且左右倾转（功能 75/76）的脚本 PWM 覆写仍有效时：

- 把姿态环的偏航差动加到 Lua 共模中心上
- 差动随前倾角按 `cos` 衰减，垂直为 1，固件认为的水平为 0
- 衰减低于 0.05 时，在 `rate_controller_run()` 之前清掉偏航目标、角速度和 I 项

稳态垂起不覆写倾转，vectored yaw 照常。固飞 `STABILIZE` / `MANUAL` 的横滚差动不碰。

地面站版本为 `ArduPlane V4.6.3-thstfac-qyaw`。滚动 Release 标签是 `firmware-qyaw`，不要写成 `firmware-thstfac`（那是方案 A / 推力比包）。

```bash
python firmware/patches/plane-4.6.3/apply.py /path/to/ardupilot
python firmware/patches/plane-4.6.3-qyaw/apply.py /path/to/ardupilot
```

公式自检（不连飞控）：`python scripts/qyaw-fade-check.py`。

编译在 GitHub Actions。本机下载：`.\scripts\download-qyaw-plane.ps1`。
