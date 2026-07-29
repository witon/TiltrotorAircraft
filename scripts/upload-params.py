#!/usr/bin/env python3
"""Upload a Mission Planner-style .param file to a connected ArduPilot board."""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from pymavlink import mavutil


PARAM_LINE = re.compile(r"^([A-Za-z0-9_]+)\s*,\s*(-?[0-9.eE+-]+)\s*$")


def load_params(path: Path) -> list[tuple[str, float]]:
    params: list[tuple[str, float]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        match = PARAM_LINE.match(line)
        if not match:
            print(f"SKIP unparsed: {line}")
            continue
        params.append((match.group(1), float(match.group(2))))
    return params


def values_close(a: float, b: float) -> bool:
    return abs(a - b) < max(1e-3, abs(b) * 1e-4)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="COM13",
        help="MAVLink serial port (default: COM13)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--param-file",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "params"
        / "matek-h743-mini-bicopter.param",
    )
    parser.add_argument(
        "--no-reboot",
        action="store_true",
        help="Do not reboot after writing parameters",
    )
    args = parser.parse_args()

    if not args.param_file.is_file():
        print(f"Param file not found: {args.param_file}")
        return 1

    params = load_params(args.param_file)
    print(f"Loaded {len(params)} params from {args.param_file.name}")
    print(f"Connecting {args.port} @ {args.baud} ...")

    try:
        master = mavutil.mavlink_connection(args.port, baud=args.baud, autoreconnect=True)
    except Exception as exc:  # noqa: BLE001
        print(f"CONNECT FAIL: {exc}")
        print("请先在 Mission Planner 里断开连接（Disconnect），再重试。")
        return 1

    print("Waiting for heartbeat...")
    try:
        hb = master.wait_heartbeat(timeout=15)
    except Exception as exc:  # noqa: BLE001
        print(f"HEARTBEAT FAIL: {exc}")
        print("请确认 USB 已连接，且 Mission Planner 未占用该串口。")
        return 1

    print(
        f"Connected: sys={master.target_system} "
        f"comp={master.target_component} type={hb.type} autopilot={hb.autopilot}"
    )

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    time.sleep(0.5)
    ver = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
    if ver:
        fw = ver.flight_sw_version
        major = (fw >> 24) & 0xFF
        minor = (fw >> 16) & 0xFF
        patch = (fw >> 8) & 0xFF
        print(f"Firmware version: {major}.{minor}.{patch}")

    ok = 0
    failed: list[str] = []
    for i, (name, value) in enumerate(params, 1):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

        deadline = time.time() + 3.0
        acked = False
        while time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            pname = msg.param_id
            if isinstance(pname, bytes):
                pname = pname.decode("ascii", errors="ignore")
            pname = pname.rstrip("\x00")
            if pname != name:
                continue
            if values_close(msg.param_value, value):
                print(f"[{i:02d}/{len(params)}] OK  {name} = {msg.param_value}")
            else:
                print(
                    f"[{i:02d}/{len(params)}] SET {name} -> requested {value}, "
                    f"board reports {msg.param_value} (may need reboot)"
                )
            acked = True
            ok += 1
            break

        if not acked:
            print(f"[{i:02d}/{len(params)}] FAIL {name} = {value} (no ack)")
            failed.append(name)
        time.sleep(0.05)

    print("---")
    print(f"Done: {ok}/{len(params)} acknowledged")
    if failed:
        print("Failed:", ", ".join(failed))
        return 2

    if not args.no_reboot:
        print("Requesting reboot...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        time.sleep(1.0)
        print("Reboot commanded. Wait ~10s then reconnect in Mission Planner.")
    else:
        print("Skipped reboot. Please reboot the flight controller manually.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
