# Plane-4.6.3 TRI 前后推力比补丁

对官方 tag **`Plane-4.6.3`** 的 `AP_MotorsTri` 增加地面站可调参数：

- `Q_M_THST_FRONT`（Motor1/2 全输出倍率，默认 1.0）
- `Q_M_THST_REAR`（Motor4 全输出倍率，默认 1.0）

系数乘在集体油门 **和** 俯仰/横滚差动上。固件版本串变为 `ArduPlane V4.6.3-thstfac`。

不要手工改这份目录里的 C++；用：

```bash
python firmware/patches/plane-4.6.3/apply.py /path/to/ardupilot
```

编译在 GitHub Actions（[`.github/workflows/build-matekh743-plane.yml`](../../../.github/workflows/build-matekh743-plane.yml)）。本机下载：`.\scripts\download-thstfac-plane.ps1`。可选本地 WSL：[`scripts/build-matekh743-plane.sh`](../../../scripts/build-matekh743-plane.sh)。
