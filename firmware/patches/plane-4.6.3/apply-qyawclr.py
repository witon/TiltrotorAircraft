#!/usr/bin/env python3
"""Clear stored yaw while Lua holds the tilt servos in QSTABILIZE.

Apply on a Plane-4.6.3 tree that already has the TRI thrust-scale patch
(firmware/patches/plane-4.6.3/apply.py). Does not touch AP_MotorsTri.

Edits (unique-string replace, not a brittle unified diff):
  - ArduPlane/version.h: THISFIRMWARE suffix -thstfac-qyawclr
  - libraries/SRV_Channel/SRV_Channel.h: function_override_active()
  - libraries/SRV_Channel/SRV_Channels.cpp: that query
  - ArduPlane/quadplane.cpp: drop yaw target, body rate and I before
    rate_controller_run() while QSTABILIZE and both tilt overrides are live
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

MARKER = "TiltrotorAircraft-qyawclr"
OLD_VER = '#define THISFIRMWARE "ArduPlane V4.6.3-thstfac"'
NEW_VER = '#define THISFIRMWARE "ArduPlane V4.6.3-thstfac-qyawclr"'


def must_replace(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text  # already applied (new may still contain old)
    if MARKER in text and old not in text:
        return text
    if old not in text:
        raise SystemExit(f"{label}: expected snippet not found")
    if text.count(old) != 1:
        raise SystemExit(f"{label}: snippet matched {text.count(old)} times, want 1")
    return text.replace(old, new, 1)


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").replace("\r\n", "\n")


def write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8", newline="\n")


def patch_version(root: Path) -> None:
    path = root / "ArduPlane" / "version.h"
    text = read_text(path)
    if NEW_VER in text:
        print(f"already {path.relative_to(root)}")
        return
    if OLD_VER not in text:
        raise SystemExit(
            f"{path}: need THISFIRMWARE ArduPlane V4.6.3-thstfac "
            "(run apply.py first)"
        )
    if text.count(OLD_VER) != 1:
        raise SystemExit(f"{path}: version line matched {text.count(OLD_VER)} times, want 1")
    write_text(path, text.replace(OLD_VER, NEW_VER, 1))
    print(f"patched {path.relative_to(root)}")


HEADER_OLD = """\
    // find first channel that a function is assigned to
    static bool find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan);
"""

HEADER_NEW = """\
    // TiltrotorAircraft-qyawclr: true while a scripting PWM timeout still holds this function
    static bool function_override_active(SRV_Channel::Aux_servo_function_t function);

    // find first channel that a function is assigned to
    static bool find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan);
"""


def patch_header(root: Path) -> None:
    path = root / "libraries" / "SRV_Channel" / "SRV_Channel.h"
    text = read_text(path)
    updated = must_replace(text, HEADER_OLD, HEADER_NEW, str(path))
    if updated == text:
        print(f"already {path.relative_to(root)}")
        return
    write_text(path, updated)
    print(f"patched {path.relative_to(root)}")


CPP_OLD = """\
#if AP_SCRIPTING_ENABLED && AP_SCHEDULER_ENABLED
// set output value for a specific function channel as a pwm value with loop based timeout
"""

CPP_NEW = r'''// TiltrotorAircraft-qyawclr: scripting PWM timeout still holding this function.
// override_counter stays non-zero for the timeout after the last write;
// override_active is cleared in calc_pwm once that counter hits zero.
bool SRV_Channels::function_override_active(SRV_Channel::Aux_servo_function_t function)
{
    if (_singleton == nullptr || channels == nullptr) {
        return false;
    }
    WITH_SEMAPHORE(_singleton->override_counter_sem);
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].get_function() != function) {
            continue;
        }
        if (override_counter[i] > 0 || channels[i].override_active) {
            return true;
        }
    }
    return false;
}

#if AP_SCRIPTING_ENABLED && AP_SCHEDULER_ENABLED
// set output value for a specific function channel as a pwm value with loop based timeout
'''


def patch_cpp(root: Path) -> None:
    path = root / "libraries" / "SRV_Channel" / "SRV_Channels.cpp"
    text = read_text(path)
    updated = must_replace(text, CPP_OLD, CPP_NEW, str(path))
    if updated == text:
        print(f"already {path.relative_to(root)}")
        return
    write_text(path, updated)
    print(f"patched {path.relative_to(root)}")


QUAD_OLD = """\
        pos_control->set_dt(last_loop_time_s);
        attitude_control->rate_controller_run();
"""

QUAD_NEW = """\
        pos_control->set_dt(last_loop_time_s);
        // TiltrotorAircraft-qyawclr: Lua common-mode tilt override (qhold / qrecover)
        // hides vectored yaw. Drop yaw angle, body rate and I before the rate
        // controller, or the stored command dumps onto the tilts when the
        // override times out (~60 ms after Lua stops writing).
        static_assert(SRV_Channel::k_tiltMotorLeft == 75, "tilt left function");
        static_assert(SRV_Channel::k_tiltMotorRight == 76, "tilt right function");
        if (plane.control_mode == &plane.mode_qstabilize &&
            SRV_Channels::function_override_active(SRV_Channel::k_tiltMotorLeft) &&
            SRV_Channels::function_override_active(SRV_Channel::k_tiltMotorRight)) {
            attitude_control->reset_yaw_target_and_rate(true);
            attitude_control->rate_bf_yaw_target(0);
            attitude_control->get_rate_yaw_pid().reset_I();
        }
        attitude_control->rate_controller_run();
"""


def patch_quadplane(root: Path) -> None:
    path = root / "ArduPlane" / "quadplane.cpp"
    text = read_text(path)
    updated = must_replace(text, QUAD_OLD, QUAD_NEW, str(path))
    if updated == text:
        print(f"already {path.relative_to(root)}")
        return
    write_text(path, updated)
    print(f"patched {path.relative_to(root)}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ardupilot_root", type=Path, help="ArduPilot Plane-4.6.3 tree, thstfac already applied")
    args = parser.parse_args()
    root = args.ardupilot_root.resolve()
    version = root / "ArduPlane" / "version.h"
    if not version.is_file():
        raise SystemExit(f"not an ArduPilot tree: {root}")
    ver_txt = read_text(version)
    if "V4.6.3-thstfac" not in ver_txt:
        raise SystemExit(
            f"{version} is not V4.6.3-thstfac (run firmware/patches/plane-4.6.3/apply.py first)"
        )
    patch_version(root)
    patch_header(root)
    patch_cpp(root)
    patch_quadplane(root)
    print("OK", MARKER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
