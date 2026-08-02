#!/usr/bin/env python3
"""Upload Mission Planner-style .param files to a connected ArduPilot board.

Modes:
  incremental (default) — write project config only (--param-file).
  full — write init.param, reboot and wait, then write project config.

Optional --aircraft NN writes params/aircraft/NN.param after the project file
(overlay for per-airframe tilt calib). Export with export-aircraft-calib.py.

Writes use batched PARAM_SET (default 16): send a batch, require PARAM_VALUE
acks whose names and values match; any missing or mismatched ack fails the upload.

Full mode requires init.param with Q_ENABLE=1 so Q_* appear after the mid reboot.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

from pymavlink import mavutil

PARAM_LINE = re.compile(r"^([A-Za-z0-9_]+)\s*,\s*(-?[0-9.eE+-]+)\s*$")
PARAMS_DIR = Path(__file__).resolve().parents[1] / "params"
AIRCRAFT_DIR = PARAMS_DIR / "aircraft"
REBOOT_WAIT_S = 14.0
DEFAULT_BATCH_SIZE = 16


def normalize_aircraft_id(raw: str) -> str:
    """Accept '1' or '01'; return zero-padded two-digit id."""
    s = raw.strip()
    if not s.isdigit():
        raise ValueError(f"aircraft id must be numeric, got: {raw!r}")
    n = int(s)
    if n < 0 or n > 99:
        raise ValueError(f"aircraft id must be 0..99, got: {n}")
    return f"{n:02d}"


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


def print_firmware_version(master) -> None:
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


def decode_param_id(pname) -> str:
    if isinstance(pname, bytes):
        pname = pname.decode("ascii", errors="ignore")
    return pname.rstrip("\x00")


def send_param(master, name: str, value: float) -> None:
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )


def write_params(
    master,
    params: list[tuple[str, float]],
    label: str,
    batch_size: int = DEFAULT_BATCH_SIZE,
) -> tuple[int, list[str]]:
    """Send PARAM_SET in batches; missing or value-mismatched ack fails the write."""
    total = len(params)
    if total == 0:
        return 0, []

    ok = 0
    width = max(2, len(str(total)))
    print(f"{label}: batch_size={batch_size}, total={total}")

    for batch_start in range(0, total, batch_size):
        batch = params[batch_start : batch_start + batch_size]
        pending: dict[str, float] = {name: value for name, value in batch}
        mismatches: list[str] = []

        for name, value in batch:
            send_param(master, name, value)

        deadline = time.time() + max(4.0, 0.25 * len(batch) + 2.0)
        while pending and time.time() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is None:
                continue
            pname = decode_param_id(msg.param_id)
            if pname not in pending:
                continue
            expected = pending.pop(pname)
            if values_close(msg.param_value, expected):
                ok += 1
            else:
                mismatches.append(
                    f"{pname} (want {expected}, got {msg.param_value})"
                )

        missing = sorted(pending.keys())
        end = batch_start + len(batch)
        if missing or mismatches:
            failed = missing + [m.split(" ", 1)[0] for m in mismatches]
            print(
                f"[{label} {batch_start + 1:0{width}d}-{end:0{width}d}/{total}] "
                f"BATCH FAIL: ok-so-far {ok}/{total}"
            )
            if missing:
                print("  missing ack:", ", ".join(missing))
            if mismatches:
                print("  value mismatch:", "; ".join(mismatches))
            return ok, failed

        print(
            f"[{label} {batch_start + 1:0{width}d}-{end:0{width}d}/{total}] "
            f"OK batch ({len(batch)})"
        )

    return ok, []


def reboot_board(master) -> None:
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


def close_connection(master) -> None:
    try:
        master.close()
    except Exception:  # noqa: BLE001
        pass


def wait_reconnect(port: str, baud: int, wait_s: float = REBOOT_WAIT_S):
    """Close is done by caller before reboot settles; open a fresh serial link.

    On Windows the COM device often disappears during FC reboot, so the old
    handle cannot be reused.
    """
    print(f"Waiting {wait_s:.0f}s for board reboot...")
    time.sleep(wait_s)

    deadline = time.time() + 45.0
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        print(f"Reconnect attempt {attempt}...")
        try:
            master = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
        except Exception as exc:  # noqa: BLE001
            print(f"  open fail: {exc}")
            time.sleep(2.0)
            continue
        try:
            hb = master.wait_heartbeat(timeout=8)
        except Exception as exc:  # noqa: BLE001
            print(f"  heartbeat fail: {exc}")
            close_connection(master)
            time.sleep(2.0)
            continue
        print(
            f"Reconnected: sys={master.target_system} "
            f"comp={master.target_component} type={hb.type}"
        )
        return master

    print("RECONNECT FAIL: port did not come back after reboot")
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="COM13",
        help="MAVLink serial port (default: COM13)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--mode",
        choices=("full", "incremental"),
        default="incremental",
        help="full: init then project; incremental: project only (default)",
    )
    parser.add_argument(
        "--param-file",
        type=Path,
        default=PARAMS_DIR / "matek-h743-mini-bicopter.param",
        help="Project config .param (default: params/matek-h743-mini-bicopter.param)",
    )
    parser.add_argument(
        "--init-file",
        type=Path,
        default=PARAMS_DIR / "init.param",
        help="Baseline .param for full mode (default: params/init.param)",
    )
    parser.add_argument(
        "--no-reboot",
        action="store_true",
        help="Skip final reboot after project config (mid full-mode reboot still runs)",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_BATCH_SIZE,
        help=f"PARAM_SET pipeline batch size (default: {DEFAULT_BATCH_SIZE})",
    )
    parser.add_argument(
        "--aircraft",
        default=None,
        help="After project config, write params/aircraft/NN.param overlay (e.g. 01)",
    )
    args = parser.parse_args()

    if args.batch_size < 1:
        print("--batch-size must be >= 1")
        return 1
    if not args.param_file.is_file():
        print(f"Param file not found: {args.param_file}")
        return 1
    if args.mode == "full" and not args.init_file.is_file():
        print(f"Init file not found: {args.init_file}")
        return 1

    aircraft_id: str | None = None
    aircraft_file: Path | None = None
    if args.aircraft is not None:
        try:
            aircraft_id = normalize_aircraft_id(args.aircraft)
        except ValueError as exc:
            print(exc)
            return 1
        aircraft_file = AIRCRAFT_DIR / f"{aircraft_id}.param"
        if not aircraft_file.is_file():
            print(f"Aircraft calib not found: {aircraft_file}")
            print("Export first: python scripts/export-aircraft-calib.py --port COMx "
                  f"--aircraft {aircraft_id}")
            return 1

    master = connect(args.port, args.baud)
    if master is None:
        return 1
    print_firmware_version(master)

    total_ok = 0
    total_n = 0
    all_failed: list[str] = []

    stages_total = 1  # project
    if args.mode == "full":
        stages_total += 1
    if aircraft_file is not None:
        stages_total += 1
    stage_idx = 0

    if args.mode == "full":
        stage_idx += 1
        init_params = load_params(args.init_file)
        print(
            f"\n=== [{stage_idx}/{stages_total}] init: {args.init_file.name} "
            f"({len(init_params)} params) ==="
        )
        ok, failed = write_params(
            master, init_params, "init", batch_size=args.batch_size
        )
        total_ok += ok
        total_n += len(init_params)
        all_failed.extend(failed)
        print(f"init done: {ok}/{len(init_params)} acknowledged")
        if failed:
            print("init failed:", ", ".join(failed))
            return 2

        reboot_board(master)
        close_connection(master)
        master = wait_reconnect(args.port, args.baud)
        if master is None:
            return 1

    project_params = load_params(args.param_file)
    stage_idx += 1
    stage = "project" if args.mode == "full" else "incr"
    print(
        f"\n=== [{stage_idx}/{stages_total}] project: {args.param_file.name} "
        f"({len(project_params)} params) ==="
    )

    ok, failed = write_params(
        master, project_params, stage, batch_size=args.batch_size
    )
    total_ok += ok
    total_n += len(project_params)
    all_failed.extend(failed)
    if failed:
        print("project failed:", ", ".join(failed))
        return 2

    if aircraft_file is not None and aircraft_id is not None:
        aircraft_params = load_params(aircraft_file)
        stage_idx += 1
        label = f"aircraft {aircraft_id}"
        print(
            f"\n=== [{stage_idx}/{stages_total}] {label}: {aircraft_file.name} "
            f"({len(aircraft_params)} params) ==="
        )
        ok, failed = write_params(
            master, aircraft_params, label, batch_size=args.batch_size
        )
        total_ok += ok
        total_n += len(aircraft_params)
        all_failed.extend(failed)
        if failed:
            print(f"{label} failed:", ", ".join(failed))
            return 2

    print("---")
    if args.mode == "full":
        print(f"Done (full): {total_ok}/{total_n} acknowledged")
    else:
        print(f"Done (incremental): {total_ok}/{total_n} acknowledged")
    if all_failed:
        print("Failed:", ", ".join(all_failed))
        return 2

    if not args.no_reboot:
        reboot_board(master)
        print("Reboot commanded. Wait ~10s then reconnect in Mission Planner.")
    else:
        print("Skipped final reboot. Please reboot the flight controller manually.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
