#!/usr/bin/env python3
"""Upload a Lua script to an ArduPilot board via MAVFTP.

Puts the file under APM/scripts/ (created if missing), warns if SCR_ENABLE is
not 1, then reboots so scripting reloads the script (use --no-reboot to skip).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCRIPT = REPO_ROOT / "lua" / "bicopter_fw_tilt_aileron.lua"
DEFAULT_REMOTE_DIR = "APM/scripts"
PUT_TIMEOUT_S = 60.0


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


def mkdir_ok(ret) -> bool:
    return ret.error_code in (FtpError.Success, FtpError.FileExists)


def ensure_remote_dirs(ftp: MAVFTP, remote_dir: str) -> bool:
    parts = [p for p in remote_dir.replace("\\", "/").split("/") if p]
    path = ""
    for part in parts:
        path = f"{path}/{part}" if path else part
        print(f"Ensuring directory: {path}")
        ret = ftp.cmd_mkdir([path])
        if not mkdir_ok(ret):
            ret.display_message()
            print(f"MKDIR FAIL: {path} (error={ret.error_code})")
            return False
        if ret.error_code == FtpError.FileExists:
            print(f"  exists: {path}")
        else:
            print(f"  created: {path}")
    return True


def upload_file(ftp: MAVFTP, local: Path, remote_path: str) -> bool:
    size = local.stat().st_size
    print(f"Uploading {local.name} ({size} bytes) -> {remote_path}")

    done: dict[str, object] = {"ok": False, "bytes": 0}

    def on_done(flen: int) -> None:
        done["ok"] = True
        done["bytes"] = flen

    def on_progress(frac: float) -> None:
        pct = int(max(0.0, min(1.0, frac)) * 100)
        print(f"  progress: {pct}%", end="\r", flush=True)

    start = ftp.cmd_put(
        [str(local), remote_path],
        callback=on_done,
        progress_callback=on_progress,
    )
    if start.error_code != FtpError.Success:
        start.display_message()
        print(f"PUT START FAIL: error={start.error_code}")
        return False

    ret = ftp.process_ftp_reply("CreateFile", timeout=PUT_TIMEOUT_S)
    print()
    if done["ok"]:
        print(f"Upload OK: {done['bytes']} bytes -> {remote_path}")
        return True

    ret.display_message()
    print(f"PUT FAIL: error={ret.error_code}")
    return False


def check_scr_enable(master) -> None:
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        b"SCR_ENABLE",
        -1,
    )
    deadline = time.time() + 5.0
    while time.time() < deadline:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if msg is None:
            continue
        pname = msg.param_id
        if isinstance(pname, bytes):
            pname = pname.decode("ascii", errors="ignore")
        pname = pname.rstrip("\x00")
        if pname != "SCR_ENABLE":
            continue
        value = int(round(msg.param_value))
        if value == 1:
            print("SCR_ENABLE=1")
        else:
            print(
                f"WARNING: SCR_ENABLE={value} (need 1). "
                "Upload params or set SCR_ENABLE=1, then reboot."
            )
        return
    print("WARNING: could not read SCR_ENABLE (param reply timeout)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="COM13",
        help="MAVLink serial port (default: COM13)",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--script",
        type=Path,
        default=DEFAULT_SCRIPT,
        help=f"Local .lua file (default: {DEFAULT_SCRIPT.name})",
    )
    parser.add_argument(
        "--remote-dir",
        default=DEFAULT_REMOTE_DIR,
        help=f"Remote directory on FC SD (default: {DEFAULT_REMOTE_DIR})",
    )
    parser.add_argument(
        "--no-reboot",
        action="store_true",
        help="Skip reboot after upload (script loads on next boot)",
    )
    args = parser.parse_args()

    script = args.script.resolve()
    if not script.is_file():
        print(f"Script not found: {script}")
        return 1
    if script.suffix.lower() != ".lua":
        print(f"WARNING: expected a .lua file, got: {script.name}")

    remote_dir = args.remote_dir.replace("\\", "/").rstrip("/")
    remote_path = f"{remote_dir}/{script.name}"

    master = connect(args.port, args.baud)
    if master is None:
        return 1

    try:
        print("Starting MAVFTP...")
        ftp = MAVFTP(
            master,
            target_system=master.target_system,
            target_component=master.target_component,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"MAVFTP INIT FAIL: {exc}")
        print("确认固件支持 MAVFTP，且链路为 MAVLink2。")
        return 1

    if not ensure_remote_dirs(ftp, remote_dir):
        return 2

    if not upload_file(ftp, script, remote_path):
        return 2

    check_scr_enable(master)

    if not args.no_reboot:
        reboot_board(master)
        print("Reboot commanded. Wait ~10s then reconnect in Mission Planner.")
        print("GCS 应出现类似: BTILT: fw tilt+throttle running")
    else:
        print("Skipped reboot. Reboot the flight controller to load the script.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
