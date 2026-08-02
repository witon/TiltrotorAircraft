#!/usr/bin/env python3
"""Export per-aircraft tilt calibration params from a connected ArduPilot board.

Reads SERVO5/6 endpoints, Q_TILT_YAW_ANGLE, and BTILT_HORIZ_L/R, then writes
params/aircraft/NN.param for use with: upload-params.py --aircraft NN

BTILT_* require the Lua script loaded; missing names abort without writing.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

PARAMS_DIR = Path(__file__).resolve().parents[1] / "params"
AIRCRAFT_DIR = PARAMS_DIR / "aircraft"

# Order matches Mission Planner-style readability in the exported file.
CALIB_PARAMS = (
    "SERVO5_MIN",
    "SERVO5_TRIM",
    "SERVO5_MAX",
    "SERVO5_REVERSED",
    "SERVO6_MIN",
    "SERVO6_TRIM",
    "SERVO6_MAX",
    "SERVO6_REVERSED",
    "Q_TILT_YAW_ANGLE",
    "BTILT_HORIZ_L",
    "BTILT_HORIZ_R",
)

BTILT_REQUIRED = ("BTILT_HORIZ_L", "BTILT_HORIZ_R")


def normalize_aircraft_id(raw: str) -> str:
    """Accept '1' or '01'; return zero-padded two-digit id."""
    s = raw.strip()
    if not s.isdigit():
        raise ValueError(f"aircraft id must be numeric, got: {raw!r}")
    n = int(s)
    if n < 0 or n > 99:
        raise ValueError(f"aircraft id must be 0..99, got: {n}")
    return f"{n:02d}"


def connect(port: str, baud: int):
    print(f"Connecting {port} @ {baud} ...")
    try:
        master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
    except Exception as exc:  # noqa: BLE001
        print(f"CONNECT FAIL: {exc}")
        print("请先在 Mission Planner 里断开连接（Disconnect），再重试。")
        return None

    print("Waiting for heartbeat...")
    try:
        hb = master.wait_heartbeat(timeout=15)
    except Exception as exc:  # noqa: BLE001
        print(f"HEARTBEAT FAIL: {exc}")
        print("请确认 USB 已连接，且 Mission Planner 未占用该串口。")
        return None

    print(
        f"Connected: sys={master.target_system} "
        f"comp={master.target_component} type={hb.type} autopilot={hb.autopilot}"
    )
    return master


def decode_param_id(pname) -> str:
    if isinstance(pname, bytes):
        pname = pname.decode("ascii", errors="ignore")
    return pname.rstrip("\x00")


def read_param(master, name: str, timeout_s: float = 3.0) -> float | None:
    """PARAM_REQUEST_READ by name; return value or None if missing/timeout."""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg is None:
            continue
        pname = decode_param_id(msg.param_id)
        if pname == name:
            return float(msg.param_value)
    return None


def format_value(value: float) -> str:
    """Compact Mission Planner-style number (ints without .0 when clean)."""
    if abs(value - round(value)) < 1e-4:
        return str(int(round(value)))
    return repr(value)


def write_param_file(path: Path, aircraft_id: str, values: dict[str, float]) -> None:
    lines = [
        f"# Aircraft {aircraft_id} — tilt servo + FW level calib",
        f"# Exported from FC; upload via: upload-params.py --aircraft {aircraft_id}",
        "",
        "# S5 TiltMotorLeft (FUNCTION 75)",
        f"SERVO5_MIN,{format_value(values['SERVO5_MIN'])}",
        f"SERVO5_TRIM,{format_value(values['SERVO5_TRIM'])}",
        f"SERVO5_MAX,{format_value(values['SERVO5_MAX'])}",
        f"SERVO5_REVERSED,{format_value(values['SERVO5_REVERSED'])}",
        "",
        "# S6 TiltMotorRight (FUNCTION 76)",
        f"SERVO6_MIN,{format_value(values['SERVO6_MIN'])}",
        f"SERVO6_TRIM,{format_value(values['SERVO6_TRIM'])}",
        f"SERVO6_MAX,{format_value(values['SERVO6_MAX'])}",
        f"SERVO6_REVERSED,{format_value(values['SERVO6_REVERSED'])}",
        "",
        f"Q_TILT_YAW_ANGLE,{format_value(values['Q_TILT_YAW_ANGLE'])}",
        "",
        "# Lua FW level (requires bicopter_fw_tilt_aileron.lua)",
        f"BTILT_HORIZ_L,{format_value(values['BTILT_HORIZ_L'])}",
        f"BTILT_HORIZ_R,{format_value(values['BTILT_HORIZ_R'])}",
        "",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="COM13",
        help="MAVLink serial port (default: COM13)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--aircraft",
        required=True,
        help="Aircraft id 01..99 (also accepts 1)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output .param path (default: params/aircraft/NN.param)",
    )
    args = parser.parse_args()

    try:
        aircraft_id = normalize_aircraft_id(args.aircraft)
    except ValueError as exc:
        print(exc)
        return 1

    out = args.output
    if out is None:
        out = AIRCRAFT_DIR / f"{aircraft_id}.param"

    master = connect(args.port, args.baud)
    if master is None:
        return 1

    values: dict[str, float] = {}
    missing: list[str] = []
    for name in CALIB_PARAMS:
        print(f"Reading {name} ...")
        val = read_param(master, name)
        if val is None:
            missing.append(name)
            print(f"  MISSING: {name}")
        else:
            values[name] = val
            print(f"  {name}={format_value(val)}")

    try:
        master.close()
    except Exception:  # noqa: BLE001
        pass

    if missing:
        btilt_miss = [n for n in missing if n in BTILT_REQUIRED]
        if btilt_miss:
            print(
                "ERROR: BTILT_* not on FC. Deploy Lua (upload-lua.py) and reboot, "
                "then export again."
            )
        other = [n for n in missing if n not in BTILT_REQUIRED]
        if other:
            print("ERROR: missing params:", ", ".join(other))
        print("Abort: not writing incomplete file.")
        return 2

    write_param_file(out, aircraft_id, values)
    print(f"Wrote {out} ({len(values)} params)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
