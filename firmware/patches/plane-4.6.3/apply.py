#!/usr/bin/env python3
"""Apply TiltrotorAircraft TRI front/rear thrust-scale edits to a Plane-4.6.3 tree.

Edits (unique-string replace, not a brittle unified diff):
  - ArduPlane/version.h: THISFIRMWARE suffix -thstfac
  - libraries/AP_Motors/AP_MotorsTri.h: var_info + Q_M_THST_FRONT/REAR
  - libraries/AP_Motors/AP_MotorsTri.cpp: scale mixer output by those params
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

MARKER = "TiltrotorAircraft-thstfac"


def must_replace(text: str, old: str, new: str, label: str) -> str:
    if MARKER in text and old not in text:
        return text  # already applied
    if old not in text:
        raise SystemExit(f"{label}: expected snippet not found")
    if text.count(old) != 1:
        raise SystemExit(f"{label}: snippet matched {text.count(old)} times, want 1")
    return text.replace(old, new, 1)


def patch_version(root: Path) -> None:
    path = root / "ArduPlane" / "version.h"
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    text = must_replace(
        text,
        '#define THISFIRMWARE "ArduPlane V4.6.3"',
        '#define THISFIRMWARE "ArduPlane V4.6.3-thstfac"',
        str(path),
    )
    path.write_text(text, encoding="utf-8", newline="\n")
    print(f"patched {path.relative_to(root)}")


def patch_header(root: Path) -> None:
    path = root / "libraries" / "AP_Motors" / "AP_MotorsTri.h"
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    text = must_replace(
        text,
        """    // Get the testing order for the motors, this is used for AP_Motors_test
    uint8_t get_motor_test_order(uint8_t i);

protected:""",
        """    // Get the testing order for the motors, this is used for AP_Motors_test
    uint8_t get_motor_test_order(uint8_t i);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:""",
        str(path) + " var_info",
    )
    text = must_replace(
        text,
        """    // parameters

    float           _pivot_angle;                       // Angle of yaw pivot""",
        """    // parameters

    // TiltrotorAircraft: per-end mixer output scale (Q_M_THST_FRONT / Q_M_THST_REAR)
    AP_Float        _thst_front;
    AP_Float        _thst_rear_fac;

    float           _pivot_angle;                       // Angle of yaw pivot""",
        str(path) + " AP_Float",
    )
    path.write_text(text, encoding="utf-8", newline="\n")
    print(f"patched {path.relative_to(root)}")


VAR_INFO = r'''
extern const AP_HAL::HAL& hal;

// TiltrotorAircraft-thstfac: Plane shows these as Q_M_THST_FRONT / Q_M_THST_REAR
const AP_Param::GroupInfo AP_MotorsTri::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: THST_FRONT
    // @DisplayName: Tricopter front motor thrust scale
    // @Description: Multiplier on Motor1/Motor2 mixer output (collective + roll/pitch). 1.0 is stock equal-motor TRI. Lower this when the front motors are stronger than the rear.
    // @Range: 0.2 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("THST_FRONT", 1, AP_MotorsTri, _thst_front, 1.0),
    // @Param: THST_REAR
    // @DisplayName: Tricopter rear motor thrust scale
    // @Description: Multiplier on Motor4 mixer output (collective + pitch). 1.0 is stock equal-motor TRI. Raise this when the rear motor is weaker, until PWM saturates.
    // @Range: 0.2 2.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("THST_REAR", 2, AP_MotorsTri, _thst_rear_fac, 1.0),
    AP_GROUPEND
};

'''


def patch_cpp(root: Path) -> None:
    path = root / "libraries" / "AP_Motors" / "AP_MotorsTri.cpp"
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    text = must_replace(
        text,
        "extern const AP_HAL::HAL& hal;\n",
        VAR_INFO,
        str(path) + " var_info",
    )
    text = must_replace(
        text,
        """    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
}""",
        """    // TiltrotorAircraft-thstfac: scale full mixer output (collective + rpy)
    const float fac_front = constrain_float(_thst_front, 0.2f, 2.0f);
    const float fac_rear = constrain_float(_thst_rear_fac, 0.2f, 2.0f);
    _thrust_right *= fac_front;
    _thrust_left  *= fac_front;
    _thrust_rear  *= fac_rear;

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
}""",
        str(path) + " scale",
    )
    path.write_text(text, encoding="utf-8", newline="\n")
    print(f"patched {path.relative_to(root)}")


def already_patched(root: Path) -> bool:
    version = (root / "ArduPlane" / "version.h").read_text(encoding="utf-8")
    return "V4.6.3-thstfac" in version


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ardupilot_root", type=Path, help="Clone of ArduPilot Plane-4.6.3")
    args = parser.parse_args()
    root = args.ardupilot_root.resolve()
    version = root / "ArduPlane" / "version.h"
    if not version.is_file():
        raise SystemExit(f"not an ArduPilot tree: {root}")
    ver_txt = version.read_text(encoding="utf-8")
    if already_patched(root):
        print(f"already patched: {root}")
        return 0
    if 'THISFIRMWARE "ArduPlane V4.6.3"' not in ver_txt:
        raise SystemExit(
            f"{version} is not stock Plane-4.6.3 (need THISFIRMWARE ArduPlane V4.6.3)"
        )
    patch_version(root)
    patch_header(root)
    patch_cpp(root)
    print("OK", MARKER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
