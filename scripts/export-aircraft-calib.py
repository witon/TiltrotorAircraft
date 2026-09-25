#!/usr/bin/env python3
"""Export per-aircraft tilt calibration params from a connected ArduPilot board.

Reads the calib list from params/configs/<id>/config.json, then writes
params/configs/<id>/aircraft/NN.param for: upload-params.py --config ID --aircraft NN

Required names abort without writing if missing. Optional names are skipped.
BTILT_* require the Lua script for that config loaded.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

from config_catalog import (
    DEFAULT_CONFIG_ID,
    format_config_list,
    load_config,
)


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


def _section_comment(name: str) -> str | None:
    if name.startswith("SERVO5_"):
        return "# S5 TiltMotorLeft (FUNCTION 75)"
    if name.startswith("SERVO6_"):
        return "# S6 TiltMotorRight (FUNCTION 76)"
    if name.startswith("BTILT_"):
        return "# Lua FW level"
    if name.startswith("BPIT_"):
        return "# Lua VTOL tail"
    return None


def write_param_file(
    path: Path,
    config_id: str,
    aircraft_id: str,
    values: dict[str, float],
    order: list[str],
) -> None:
    lines = [
        f"# Aircraft {aircraft_id} — tilt servo + FW level calib (config: {config_id})",
        f"# Exported from FC; upload via: upload-params.py --config {config_id} "
        f"--aircraft {aircraft_id}",
        "",
    ]
    last_section: str | None = None
    for name in order:
        section = _section_comment(name)
        if section and section != last_section:
            if last_section is not None:
                lines.append("")
            lines.append(section)
            last_section = section
        elif section is None and last_section is not None:
            lines.append("")
            last_section = None
        lines.append(f"{name},{format_value(values[name])}")
    lines.append("")
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
        "--config",
        default=DEFAULT_CONFIG_ID,
        help=f"Airframe config id under params/configs/ (default: {DEFAULT_CONFIG_ID})",
    )
    parser.add_argument(
        "--list-configs",
        action="store_true",
        help="Print known configs and exit",
    )
    parser.add_argument(
        "--aircraft",
        required=False,
        help="Aircraft id 01..99 (also accepts 1)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output .param path (default: params/configs/<id>/aircraft/NN.param)",
    )
    args = parser.parse_args()

    if args.list_configs:
        print("Known configs:")
        print(format_config_list())
        return 0

    if args.aircraft is None:
        print("--aircraft is required unless --list-configs")
        return 1

    try:
        cfg = load_config(args.config)
        aircraft_id = normalize_aircraft_id(args.aircraft)
    except (FileNotFoundError, ValueError) as exc:
        print(exc)
        return 1

    out = args.output
    if out is None:
        out = cfg.aircraft_dir / f"{aircraft_id}.param"

    print(f"Config: {cfg.config_id} ({cfg.title})")

    master = connect(args.port, args.baud)
    if master is None:
        return 1

    values: dict[str, float] = {}
    missing_required: list[str] = []
    order: list[str] = []

    for name in cfg.calib_required:
        print(f"Reading {name} ...")
        val = read_param(master, name)
        if val is None:
            missing_required.append(name)
            print(f"  MISSING: {name}")
        else:
            values[name] = val
            order.append(name)
            print(f"  {name}={format_value(val)}")

    for name in cfg.calib_optional:
        print(f"Reading {name} (optional) ...")
        val = read_param(master, name)
        if val is None:
            print(f"  skip missing optional: {name}")
        else:
            values[name] = val
            order.append(name)
            print(f"  {name}={format_value(val)}")

    try:
        master.close()
    except Exception:  # noqa: BLE001
        pass

    if missing_required:
        btilt_miss = [n for n in missing_required if n.startswith("BTILT_")]
        if btilt_miss:
            print(
                "ERROR: BTILT_* not on FC. Deploy Lua "
                f"(upload-lua.py --config {cfg.config_id}) and reboot, then export again."
            )
        other = [n for n in missing_required if not n.startswith("BTILT_")]
        if other:
            print("ERROR: missing params:", ", ".join(other))
        print("Abort: not writing incomplete file.")
        return 2

    write_param_file(out, cfg.config_id, aircraft_id, values, order)
    print(f"Wrote {out} ({len(values)} params)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
