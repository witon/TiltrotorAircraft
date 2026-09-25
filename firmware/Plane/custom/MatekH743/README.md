Patched MatekH743 Plane binaries land here after GitHub Actions
(`firmware-thstfac` Release) or a local WSL build.

```powershell
# Preferred: download CI output (no compiler on this PC)
.\scripts\download-thstfac-plane.ps1
```

Optional local build: `.\scripts\build-matekh743-plane.ps1` (WSL).

Flash `arduplane.apj` with Mission Planner **Load custom firmware**. GCS version must read **ArduPlane V4.6.3-thstfac**. Do not install Plane from the online MatekH743 list — that restores equal-motor TRI mixing.
