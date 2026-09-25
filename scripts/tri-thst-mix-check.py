#!/usr/bin/env python3
"""Sanity-check TRI THST_FRONT/REAR scaling (same formula as the firmware patch).

Does not talk to the flight controller. Use after a firmware flash for the
prop-off PWM check documented in docs/ardupilot-setup.md.
"""
from __future__ import annotations


def mix(throttle: float, pitch: float, roll: float, front: float, rear: float) -> tuple[float, float, float]:
    """pitch/roll in [-1,1], throttle in [0,1]. Returns right, left, rear in 0..1."""
    rpy_right = roll * -0.5 + pitch * 0.5
    rpy_left = roll * 0.5 + pitch * 0.5
    rpy_rear = pitch * -0.5
    right = (throttle + rpy_right) * front
    left = (throttle + rpy_left) * front
    rear_t = (throttle + rpy_rear) * rear
    clamp = lambda x: max(0.0, min(1.0, x))
    return clamp(right), clamp(left), clamp(rear_t)


def pwm(thrust: float, mn: int = 1000, mx: int = 2000) -> int:
    return int(round(mn + thrust * (mx - mn)))


def main() -> None:
    cases = [
        ("hover stick, equal", 0.5, 0.0, 0.0, 1.0, 1.0),
        ("hover stick, FRONT=0.6", 0.5, 0.0, 0.0, 0.6, 1.0),
        ("pitch up, equal", 0.5, 0.4, 0.0, 1.0, 1.0),
        ("pitch up, FRONT=0.6", 0.5, 0.4, 0.0, 0.6, 1.0),
    ]
    print(f"{'case':<24} {'S12 R':>6} {'S11 L':>6} {'S8 rear':>8}")
    prev_equal_pitch = None
    scaled_pitch = None
    for name, thr, pit, rol, f, r in cases:
        right, left, rear = mix(thr, pit, rol, f, r)
        print(f"{name:<24} {pwm(right):6d} {pwm(left):6d} {pwm(rear):8d}")
        if name == "pitch up, equal":
            prev_equal_pitch = abs(pwm(right) - pwm(mix(0.5, 0.0, 0.0, 1.0, 1.0)[0]))
        if name == "pitch up, FRONT=0.6":
            hover = mix(0.5, 0.0, 0.0, 0.6, 1.0)
            scaled_pitch = abs(pwm(right) - pwm(hover[0]))

    assert prev_equal_pitch and scaled_pitch
    ratio = scaled_pitch / prev_equal_pitch
    print(f"\npitch-up front PWM delta: equal={prev_equal_pitch}  FRONT0.6={scaled_pitch}  ratio={ratio:.2f} (want ~0.60)")
    if abs(ratio - 0.6) > 0.08:
        raise SystemExit("pitch path did not scale with FRONT")
    hover_eq = pwm(mix(0.5, 0, 0, 1, 1)[0])
    hover_f = pwm(mix(0.5, 0, 0, 0.6, 1)[0])
    print(f"collective front PWM: equal={hover_eq}  FRONT0.6={hover_f}  (FRONT0.6 must be lower)")
    if hover_f >= hover_eq:
        raise SystemExit("collective path did not scale with FRONT")
    print("OK: both climb and pitch paths scale")


if __name__ == "__main__":
    main()
