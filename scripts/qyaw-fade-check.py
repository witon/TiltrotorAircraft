#!/usr/bin/env python3
"""Sanity-check scheme B yaw fade and PWM overlay (same formula as the firmware patch).

Does not talk to the flight controller. Tilt servos are scaled 0..1000 across MIN..MAX.
"""
from __future__ import annotations

import math

FADE_OFF = 0.05


def scaled_from_pwm(pwm: int, smin: int, smax: int, reversed_: bool) -> float:
    if smax <= smin:
        return 0.0
    scaled = (float(pwm) - float(smin)) * 1000.0 / float(smax - smin)
    if reversed_:
        scaled = 1000.0 - scaled
    return max(0.0, min(1000.0, scaled))


def pwm_from_scaled(scaled: float, smin: int, smax: int, reversed_: bool, high: int = 1000) -> int:
    scaled = max(0.0, min(float(high), scaled))
    if reversed_:
        scaled = high - scaled
    return smin + int((scaled * (smax - smin)) / high)


def geometry(yaw_angle: float, fixed_angle: float) -> tuple[float, float, float]:
    total = 90.0 + yaw_angle + fixed_angle
    zero_out = yaw_angle / total
    level_out = 1.0 - (fixed_angle / total)
    return total, zero_out, level_out - zero_out


def tilt_norm(scaled: float, zero_out: float, span: float) -> float:
    return max(0.0, min(1.0, (scaled / 1000.0 - zero_out) / span))


def fade_from_centers(
    center_l: int,
    center_r: int,
    left: tuple[int, int, bool],
    right: tuple[int, int, bool],
    yaw_angle: float,
    fixed_angle: float,
) -> float:
    _, zero_out, span = geometry(yaw_angle, fixed_angle)
    if span <= 0.0:
        return 1.0
    n_l = tilt_norm(scaled_from_pwm(center_l, *left), zero_out, span)
    n_r = tilt_norm(scaled_from_pwm(center_r, *right), zero_out, span)
    return math.cos(math.radians(0.5 * (n_l + n_r) * 90.0))


def center_pwm(raw: int, have: bool, last_sent: int, last_center: int) -> int:
    if have and raw == last_sent:
        return last_center
    return raw


def overlay(
    center_l: int,
    center_r: int,
    left: tuple[int, int, bool],
    right: tuple[int, int, bool],
    yaw_angle: float,
    fixed_angle: float,
    yaw_out: float,
    throttle_scaler: float,
) -> tuple[int, int, float]:
    fade = fade_from_centers(center_l, center_r, left, right, yaw_angle, fixed_angle)
    if fade < FADE_OFF:
        fade = 0.0
    _, zero_out, _span = geometry(yaw_angle, fixed_angle)
    yaw_range = zero_out
    tilt_scale = max(-1.0, min(1.0, throttle_scaler * yaw_out * fade))
    tilt_offset = tilt_scale * yaw_range
    left_scaled = max(0.0, min(1000.0, scaled_from_pwm(center_l, *left) + tilt_offset * 1000.0))
    right_scaled = max(0.0, min(1000.0, scaled_from_pwm(center_r, *right) - tilt_offset * 1000.0))
    return (
        pwm_from_scaled(left_scaled, *left),
        pwm_from_scaled(right_scaled, *right),
        fade,
    )


def pwm_at_frac(frac: float, smin: int, smax: int, reversed_: bool) -> int:
    """frac is scaled/1000 before the channel's own reverse."""
    return pwm_from_scaled(frac * 1000.0, smin, smax, reversed_)


def main() -> None:
    yaw_angle = 15.0
    fixed_angle = 0.0
    _total, zero_out, span = geometry(yaw_angle, fixed_angle)
    left = (1000, 2000, False)
    right = (1000, 2000, True)

    vert_l = pwm_at_frac(zero_out, *left)
    vert_r = pwm_at_frac(zero_out, *right)
    level_l = pwm_at_frac(zero_out + span, *left)
    level_r = pwm_at_frac(zero_out + span, *right)

    fade_vert = fade_from_centers(vert_l, vert_r, left, right, yaw_angle, fixed_angle)
    fade_level = fade_from_centers(level_l, level_r, left, right, yaw_angle, fixed_angle)
    mid_frac = zero_out + 0.7 * span
    mid_l = pwm_at_frac(mid_frac, *left)
    mid_r = pwm_at_frac(mid_frac, *right)
    fade_mid = fade_from_centers(mid_l, mid_r, left, right, yaw_angle, fixed_angle)
    expect_mid = math.cos(math.radians(0.7 * 90.0))

    print(f"vertical fade={fade_vert:.4f}  level fade={fade_level:.4f}  0.7-span fade={fade_mid:.4f}")
    if abs(fade_vert - 1.0) > 1e-6:
        raise SystemExit("vertical fade should be 1")
    if fade_level >= FADE_OFF:
        raise SystemExit("level fade should be treated as off")
    if abs(fade_mid - expect_mid) > 2e-3:
        raise SystemExit(f"mid fade {fade_mid} != cos(63 deg) {expect_mid}")
    if fade_mid < FADE_OFF:
        raise SystemExit("0.7 of the vertical-to-level span must still keep yaw")

    pwm_l, pwm_r, fade = overlay(vert_l, vert_r, left, right, yaw_angle, fixed_angle, 1.0, 1.0)
    print(f"yaw +1 at vertical: L {vert_l}->{pwm_l}  R {vert_r}->{pwm_r}  fade={fade:.3f}")
    if fade < 0.99:
        raise SystemExit("full yaw fade expected at vertical")
    if pwm_l <= vert_l:
        raise SystemExit("positive yaw should raise left scaled, so left PWM rises")
    if pwm_r <= vert_r:
        raise SystemExit("positive yaw lowers right scaled; reversed right PWM should rise")

    pwm_l_neg, pwm_r_neg, _fade_neg = overlay(vert_l, vert_r, left, right, yaw_angle, fixed_angle, -1.0, 1.0)
    if pwm_l_neg >= vert_l or pwm_r_neg >= vert_r:
        raise SystemExit("negative yaw should move both PWMs the other way")

    # Second pass must reuse the saved center, not add the differential again.
    saved_center_l = vert_l
    raw_l = pwm_l
    again = center_pwm(raw_l, True, pwm_l, saved_center_l)
    if again != vert_l:
        raise SystemExit("center tracking walked off the Lua center")
    pwm_l2, _pwm_r2, _fade2 = overlay(again, center_pwm(pwm_r, True, pwm_r, vert_r), left, right, yaw_angle, fixed_angle, 1.0, 1.0)
    if pwm_l2 != pwm_l:
        raise SystemExit("overlay accumulated across loops")

    new_center = vert_l + 40
    if center_pwm(new_center, True, pwm_l, saved_center_l) != new_center:
        raise SystemExit("a new Lua center was ignored")

    off_l, off_r, off_fade = overlay(level_l, level_r, left, right, yaw_angle, fixed_angle, 1.0, 1.0)
    print(f"yaw +1 at level: L {level_l}->{off_l}  R {level_r}->{off_r}  fade={off_fade:.3f}")
    if off_fade != 0.0 or off_l != level_l or off_r != level_r:
        raise SystemExit("faded-out overlay should write the center unchanged")

    print("OK: fade, sign and center tracking")


if __name__ == "__main__":
    main()
