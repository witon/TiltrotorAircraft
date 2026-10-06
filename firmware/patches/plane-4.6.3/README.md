# Plane-4.6.3 自定义补丁

按顺序打到官方 tag **`Plane-4.6.3`** 上。不要手工改 C++。

## 1. 前后推力比

[`apply.py`](apply.py) 给 `AP_MotorsTri` 增加地面站可调参数：

- `Q_M_THST_FRONT`（Motor1/2 全输出倍率，默认 1.0）
- `Q_M_THST_REAR`（Motor4 全输出倍率，默认 1.0）

系数乘在集体油门 **和** 俯仰/横滚差动上。这一步把版本串设成 `ArduPlane V4.6.3-thstfac`。

## 2. 过渡清偏航

[`apply-qyawclr.py`](apply-qyawclr.py) 叠在上一步之后，不改 `AP_MotorsTri`，也不改 Lua。

`QSTABILIZE` 且左右倾转（功能 75/76）的脚本 PWM 覆写仍有效时，在 `rate_controller_run()` 之前清掉偏航目标、机体角速度和 I 项。横滚和俯仰不动。稳态垂起不覆写倾转，vectored yaw 照常。

最终地面站版本为 `ArduPlane V4.6.3-thstfac-qyawclr`。

```bash
python firmware/patches/plane-4.6.3/apply.py /path/to/ardupilot
python firmware/patches/plane-4.6.3/apply-qyawclr.py /path/to/ardupilot
```

编译在 GitHub Actions（[`.github/workflows/build-matekh743-plane.yml`](../../../.github/workflows/build-matekh743-plane.yml)）。本机下载：`.\scripts\download-thstfac-plane.ps1`。可选本地 WSL：[`scripts/build-matekh743-plane.sh`](../../../scripts/build-matekh743-plane.sh)。
