#!/usr/bin/env python3
"""Overlay faded vectored yaw on Lua's common-mode tilt center.

Apply on a Plane-4.6.3 tree that already has the TRI thrust-scale patch
(firmware/patches/plane-4.6.3/apply.py). Does not touch AP_MotorsTri.

While QSTABILIZE and both tilt servos (functions 75/76) still have a
scripting PWM override, the attitude controller's yaw differential is
added to that center and faded with cos(forward tilt). When the fade
is below 0.05 the yaw target, body rate and yaw-rate I are cleared
before rate_controller_run().

Edits (unique-string replace, not a brittle unified diff):
  - ArduPlane/version.h: THISFIRMWARE suffix -thstfac-qyaw
  - libraries/SRV_Channel/SRV_Channel.h: function_override_active()
  - libraries/SRV_Channel/SRV_Channels.cpp: that query
  - ArduPlane/tiltrotor.h / tiltrotor.cpp: fade, clear, PWM overlay
  - ArduPlane/quadplane.cpp: call those around rate_controller_run()
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

MARKER = "TiltrotorAircraft-qyaw"
OLD_VER = '#define THISFIRMWARE "ArduPlane V4.6.3-thstfac"'
NEW_VER = '#define THISFIRMWARE "ArduPlane V4.6.3-thstfac-qyaw"'


def must_replace(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
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
    // TiltrotorAircraft-qyaw: true while a scripting PWM timeout still holds this function
    static bool function_override_active(SRV_Channel::Aux_servo_function_t function);

    // find first channel that a function is assigned to
    static bool find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan);
"""


def patch_srv_header(root: Path) -> None:
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

CPP_NEW = r'''// TiltrotorAircraft-qyaw: scripting PWM timeout still holding this function.
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


def patch_srv_cpp(root: Path) -> None:
    path = root / "libraries" / "SRV_Channel" / "SRV_Channels.cpp"
    text = read_text(path)
    updated = must_replace(text, CPP_OLD, CPP_NEW, str(path))
    if updated == text:
        print(f"already {path.relative_to(root)}")
        return
    write_text(path, updated)
    print(f"patched {path.relative_to(root)}")


TILT_H_OLD = """\
private:

    // Tiltrotor specific log message
"""

TILT_H_NEW = """\
private:

    // TiltrotorAircraft-qyaw: QSTABILIZE common-mode tilt override
    bool script_yaw_override_active() const;
    bool script_yaw_centers(uint16_t &center_l, uint16_t &center_r) const;
    float script_yaw_fade(uint16_t center_l, uint16_t center_r) const;
    void forget_script_yaw_centers();
    void relax_script_yaw();
    void overlay_script_yaw();
    bool qyaw_have_l = false;
    bool qyaw_have_r = false;
    uint16_t qyaw_last_sent_l = 0;
    uint16_t qyaw_last_sent_r = 0;
    uint16_t qyaw_last_center_l = 0;
    uint16_t qyaw_last_center_r = 0;

    // Tiltrotor specific log message
"""


def patch_tiltrotor_header(root: Path) -> None:
    path = root / "ArduPlane" / "tiltrotor.h"
    text = read_text(path)
    updated = must_replace(text, TILT_H_OLD, TILT_H_NEW, str(path))
    if updated == text:
        print(f"already {path.relative_to(root)}")
        return
    write_text(path, updated)
    print(f"patched {path.relative_to(root)}")


TILT_CPP_OLD = """\
bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

#endif  // HAL_QUADPLANE_ENABLED
"""

TILT_CPP_NEW = r'''bool Tiltrotor::tilt_over_max_angle(void) const
{
    const float tilt_threshold = (max_angle_deg/90.0f);
    return (current_tilt > MIN(tilt_threshold, get_forward_flight_tilt()));
}

// TiltrotorAircraft-qyaw
// Tilt functions use set_range(1000). Below this, yaw differential is gone
// and the attitude memory must be cleared or it winds with no actuator.
static constexpr float QYAW_FADE_OFF = 0.05f;

static float qyaw_scaled_from_pwm(const SRV_Channel &ch, uint16_t pwm)
{
    const uint16_t smin = ch.get_output_min();
    const uint16_t smax = ch.get_output_max();
    if (smax <= smin) {
        return 0.0f;
    }
    float scaled = (float(pwm) - float(smin)) * 1000.0f / float(smax - smin);
    if (ch.get_reversed()) {
        scaled = 1000.0f - scaled;
    }
    return constrain_float(scaled, 0.0f, 1000.0f);
}

static uint16_t qyaw_center_pwm(const SRV_Channel &ch, bool have, uint16_t last_sent, uint16_t last_center)
{
    const uint16_t raw = ch.get_output_pwm();
    if (have && raw == last_sent) {
        return last_center;
    }
    return raw;
}

static float qyaw_tilt_norm(float scaled, float zero_out, float span)
{
    return constrain_float((scaled / 1000.0f - zero_out) / span, 0.0f, 1.0f);
}

bool Tiltrotor::script_yaw_override_active() const
{
    if (type != TILT_TYPE_VECTORED_YAW) {
        return false;
    }
    if (plane.control_mode != &plane.mode_qstabilize) {
        return false;
    }
    static_assert(SRV_Channel::k_tiltMotorLeft == 75, "tilt left function");
    static_assert(SRV_Channel::k_tiltMotorRight == 76, "tilt right function");
    return SRV_Channels::function_override_active(SRV_Channel::k_tiltMotorLeft) &&
           SRV_Channels::function_override_active(SRV_Channel::k_tiltMotorRight);
}

bool Tiltrotor::script_yaw_centers(uint16_t &center_l, uint16_t &center_r) const
{
    SRV_Channel *left = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorLeft);
    SRV_Channel *right = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorRight);
    if (left == nullptr || right == nullptr) {
        return false;
    }
    center_l = qyaw_center_pwm(*left, qyaw_have_l, qyaw_last_sent_l, qyaw_last_center_l);
    center_r = qyaw_center_pwm(*right, qyaw_have_r, qyaw_last_sent_r, qyaw_last_center_r);
    return true;
}

float Tiltrotor::script_yaw_fade(uint16_t center_l, uint16_t center_r) const
{
    SRV_Channel *left = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorLeft);
    SRV_Channel *right = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorRight);
    const float total_angle = 90.0f + tilt_yaw_angle + fixed_angle;
    if (left == nullptr || right == nullptr || !is_positive(total_angle)) {
        return 1.0f;
    }
    const float zero_out = tilt_yaw_angle / total_angle;
    const float level_out = 1.0f - (fixed_angle / total_angle);
    const float span = level_out - zero_out;
    if (!is_positive(span)) {
        return 1.0f;
    }
    const float tilt_norm = 0.5f * (
        qyaw_tilt_norm(qyaw_scaled_from_pwm(*left, center_l), zero_out, span) +
        qyaw_tilt_norm(qyaw_scaled_from_pwm(*right, center_r), zero_out, span));
    return cosf(radians(tilt_norm * 90.0f));
}

void Tiltrotor::forget_script_yaw_centers()
{
    qyaw_have_l = false;
    qyaw_have_r = false;
}

void Tiltrotor::relax_script_yaw()
{
    if (!script_yaw_override_active()) {
        forget_script_yaw_centers();
        return;
    }
    uint16_t center_l = 0;
    uint16_t center_r = 0;
    if (!script_yaw_centers(center_l, center_r)) {
        return;
    }
    if (script_yaw_fade(center_l, center_r) >= QYAW_FADE_OFF) {
        return;
    }
    quadplane.attitude_control->reset_yaw_target_and_rate(true);
    quadplane.attitude_control->rate_bf_yaw_target(0);
    quadplane.attitude_control->get_rate_yaw_pid().reset_I();
}

void Tiltrotor::overlay_script_yaw()
{
    SRV_Channel *left = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorLeft);
    SRV_Channel *right = SRV_Channels::get_channel_for(SRV_Channel::k_tiltMotorRight);
    if (left == nullptr || right == nullptr || motors == nullptr || !script_yaw_override_active()) {
        forget_script_yaw_centers();
        return;
    }

    const uint16_t center_l = qyaw_center_pwm(*left, qyaw_have_l, qyaw_last_sent_l, qyaw_last_center_l);
    const uint16_t center_r = qyaw_center_pwm(*right, qyaw_have_r, qyaw_last_sent_r, qyaw_last_center_r);

    float fade = script_yaw_fade(center_l, center_r);
    if (fade < QYAW_FADE_OFF) {
        fade = 0.0f;
    }

    const float total_angle = 90.0f + tilt_yaw_angle + fixed_angle;
    const float yaw_range = is_positive(total_angle) ? (tilt_yaw_angle / total_angle) : 0.0f;
    const float yaw_out = motors->get_yaw() + motors->get_yaw_ff();
    const float throttle = motors->get_throttle_out();
    const float scale_min = 0.5f;
    const float scale_max = 2.0f;
    float throttle_scaler = scale_max;
    if (is_positive(throttle)) {
        throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, scale_min, scale_max);
    }

    // Yaw term only. Roll stays on the tri mixer while the nacelles are slewing.
    float tilt_scale = throttle_scaler * yaw_out * fade;
    if (fabsf(tilt_scale) > 1.0f) {
        tilt_scale = constrain_float(tilt_scale, -1.0f, 1.0f);
        motors->limit.yaw = true;
    }
    const float tilt_offset = tilt_scale * yaw_range;

    const float left_unlim = qyaw_scaled_from_pwm(*left, center_l) + tilt_offset * 1000.0f;
    const float right_unlim = qyaw_scaled_from_pwm(*right, center_r) - tilt_offset * 1000.0f;
    const float left_scaled = constrain_float(left_unlim, 0.0f, 1000.0f);
    const float right_scaled = constrain_float(right_unlim, 0.0f, 1000.0f);
    if (left_scaled != left_unlim || right_scaled != right_unlim) {
        motors->limit.yaw = true;
    }

    const uint16_t pwm_l = left->pwm_from_scaled_value(left_scaled);
    const uint16_t pwm_r = right->pwm_from_scaled_value(right_scaled);
    left->set_output_pwm(pwm_l, true);
    right->set_output_pwm(pwm_r, true);

    qyaw_last_center_l = center_l;
    qyaw_last_center_r = center_r;
    qyaw_last_sent_l = pwm_l;
    qyaw_last_sent_r = pwm_r;
    qyaw_have_l = true;
    qyaw_have_r = true;
}

#endif  // HAL_QUADPLANE_ENABLED
'''


def patch_tiltrotor_cpp(root: Path) -> None:
    path = root / "ArduPlane" / "tiltrotor.cpp"
    text = read_text(path)
    updated = must_replace(text, TILT_CPP_OLD, TILT_CPP_NEW, str(path))
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
        // TiltrotorAircraft-qyaw: Lua common-mode tilt override hides vectored yaw.
        // Clear yaw memory only once the forward-tilt fade has removed the actuator,
        // then write the controller differential onto the Lua center.
        tiltrotor.relax_script_yaw();
        attitude_control->rate_controller_run();
        tiltrotor.overlay_script_yaw();
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
    patch_srv_header(root)
    patch_srv_cpp(root)
    patch_tiltrotor_header(root)
    patch_tiltrotor_cpp(root)
    patch_quadplane(root)
    print("OK", MARKER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
