#!/usr/bin/env python3
"""Analyze ArduPilot .BIN logs and print param / setup suggestions.

Works on a local file, or downloads from the flight controller over USB/MAVFTP
then analyzes. Findings are mapped to this repo's --config (bicopter / tilttri).
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path

from pymavlink.DFReader import DFReader_binary

from config_catalog import (
    DEFAULT_CONFIG_ID,
    REPO_ROOT,
    format_config_list,
    load_config,
)

PARAM_LINE = re.compile(r"^([A-Za-z0-9_]+)\s*,\s*(-?[0-9.eE+-]+)\s*$")

MODE_NAMES = {
    0: "MANUAL",
    2: "STABILIZE",
    5: "FBWA",
    10: "AUTO",
    11: "RTL",
    15: "GUIDED",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
}

POSITION_MODES = {10, 11, 15, 19, 20, 21}

EXPECTED_BTILT_MSG = {
    "bicopter": "BTILT: fw tilt+throttle+vtol tail running",
    "tilttri": "BTILT: tilttri fw differential tilt running",
}

COMPARE_KEYS = (
    "Q_ENABLE",
    "Q_FRAME_CLASS",
    "Q_TILT_TYPE",
    "Q_TILT_MASK",
    "Q_TILT_ENABLE",
    "Q_ASSIST_SPEED",
    "Q_OPTIONS",
    "Q_ANGLE_MAX",
    "Q_M_THST_EXPO",
    "Q_M_SLEW_UP_TIME",
    "Q_M_SPIN_MIN",
    "Q_A_INPUT_TC",
    "Q_A_ANG_RLL_P",
    "Q_A_ANG_PIT_P",
    "AHRS_ORIENTATION",
    "SCR_ENABLE",
    "COMPASS_ENABLE",
    "GPS1_TYPE",
    "AHRS_GPS_USE",
    "ARMING_CHECK",
    "FLTMODE1",
    "FLTMODE2",
    "FLTMODE3",
    "FLTMODE4",
    "FLTMODE5",
    "FLTMODE6",
    "SERVO5_FUNCTION",
    "SERVO6_FUNCTION",
    "SERVO7_FUNCTION",
    "SERVO8_FUNCTION",
    "SERVO11_FUNCTION",
    "SERVO12_FUNCTION",
)

VIBE_CLIP_WARN = 50
VIBE_RMS_WARN = 30.0
ATT_ERR_MEAN_DEG = 6.0
PLACEHOLDER_HORIZ = 1200.0


@dataclass
class Finding:
    severity: str  # error / warn / info
    title: str
    evidence: str
    suggestion: str


@dataclass
class ChanStat:
    n: int = 0
    lo: float = 1e9
    hi: float = -1e9
    sat_lo: int = 0
    sat_hi: int = 0

    def add(self, v: float, lo_lim: float | None, hi_lim: float | None) -> None:
        self.n += 1
        if v < self.lo:
            self.lo = v
        if v > self.hi:
            self.hi = v
        if lo_lim is not None and v <= lo_lim + 5:
            self.sat_lo += 1
        if hi_lim is not None and v >= hi_lim - 5:
            self.sat_hi += 1


@dataclass
class LogStats:
    msgs: list[str] = field(default_factory=list)
    errs: list[str] = field(default_factory=list)
    parms: dict[str, float] = field(default_factory=dict)
    modes: list[tuple[int, str]] = field(default_factory=list)
    mode_counts: Counter = field(default_factory=Counter)
    firmware: str = ""
    frame_msg: str = ""
    t_first_us: int | None = None
    t_last_us: int | None = None
    armed_samples: int = 0
    disarmed_samples: int = 0
    arm_events: int = 0
    disarm_events: int = 0
    att_n: int = 0
    att_roll_abs: float = 0.0
    att_pitch_abs: float = 0.0
    att_roll_err: float = 0.0
    att_pitch_err: float = 0.0
    att_roll_err_max: float = 0.0
    att_pitch_err_max: float = 0.0
    vibe_n: int = 0
    vibe_max: tuple[float, float, float] = (0.0, 0.0, 0.0)
    clip_sum: tuple[int, int, int] = (0, 0, 0)
    rcou: dict[int, ChanStat] = field(default_factory=dict)
    armed: bool = False
    rc3_n: int = 0
    rc3_lo: float = 1e9
    rc3_hi: float = -1e9


def load_param_file(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        match = PARAM_LINE.match(line)
        if match:
            out[match.group(1)] = float(match.group(2))
    return out


def expected_params(config_id: str, aircraft: str | None) -> dict[str, float]:
    cfg = load_config(config_id)
    merged = load_param_file(cfg.project_param)
    if aircraft:
        overlay = cfg.aircraft_dir / f"{aircraft}.param"
        if overlay.is_file():
            merged.update(load_param_file(overlay))
    return merged


def mode_name(num: int) -> str:
    return MODE_NAMES.get(num, f"MODE_{num}")


def _get(msg, *names, default=None):
    for name in names:
        if hasattr(msg, name):
            val = getattr(msg, name)
            if val is not None:
                return val
    return default


def ingest(stats: LogStats, msg) -> None:
    t = msg.get_type()
    tus = _get(msg, "TimeUS")
    if isinstance(tus, (int, float)):
        tus_i = int(tus)
        if stats.t_first_us is None:
            stats.t_first_us = tus_i
        stats.t_last_us = tus_i

    if t == "MSG":
        text = str(_get(msg, "Message", default=""))
        stats.msgs.append(text)
        low = text.lower()
        if "throttle armed" in low:
            stats.armed = True
            stats.arm_events += 1
        elif "throttle disarmed" in low or text.startswith("Disarm"):
            stats.armed = False
            stats.disarm_events += 1
        if text.startswith("ArduPlane") or text.startswith("ArduCopter"):
            stats.firmware = text
        if "QuadPlane Frame" in text:
            stats.frame_msg = text
        return

    if t == "ERR":
        stats.errs.append(str(msg))
        return

    if t == "PARM":
        name = _get(msg, "Name")
        val = _get(msg, "Value")
        if isinstance(name, bytes):
            name = name.decode("ascii", errors="ignore")
        if name:
            name = str(name).rstrip("\x00")
            if val is not None:
                stats.parms[name] = float(val)
        return

    if t == "MODE":
        num = int(_get(msg, "ModeNum", "Mode", default=-1))
        stats.modes.append((num, mode_name(num)))
        stats.mode_counts[num] += 1
        return

    if t == "ARM":
        armed = int(_get(msg, "ArmState", default=0) or 0)
        stats.armed = bool(armed)
        if stats.armed:
            stats.arm_events += 1
        else:
            stats.disarm_events += 1
        return

    if stats.armed:
        stats.armed_samples += 1
    else:
        stats.disarmed_samples += 1

    if t == "ATT" and stats.armed:
        roll = float(_get(msg, "Roll", default=0) or 0)
        pitch = float(_get(msg, "Pitch", default=0) or 0)
        droll = float(_get(msg, "DesRoll", default=roll) or roll)
        dpitch = float(_get(msg, "DesPitch", default=pitch) or pitch)
        er = abs(roll - droll)
        ep = abs(pitch - dpitch)
        stats.att_n += 1
        stats.att_roll_abs += abs(roll)
        stats.att_pitch_abs += abs(pitch)
        stats.att_roll_err += er
        stats.att_pitch_err += ep
        stats.att_roll_err_max = max(stats.att_roll_err_max, er)
        stats.att_pitch_err_max = max(stats.att_pitch_err_max, ep)
        return

    if t == "VIBE":
        vx = float(_get(msg, "VibeX", default=0) or 0)
        vy = float(_get(msg, "VibeY", default=0) or 0)
        vz = float(_get(msg, "VibeZ", default=0) or 0)
        c0 = int(_get(msg, "Clip0", default=0) or 0)
        c1 = int(_get(msg, "Clip1", default=0) or 0)
        c2 = int(_get(msg, "Clip2", default=0) or 0)
        stats.vibe_n += 1
        mx, my, mz = stats.vibe_max
        stats.vibe_max = (max(mx, vx), max(my, vy), max(mz, vz))
        stats.clip_sum = (
            max(stats.clip_sum[0], c0),
            max(stats.clip_sum[1], c1),
            max(stats.clip_sum[2], c2),
        )
        return

    if t == "RCIN" and stats.armed:
        v = _get(msg, "C3")
        if v is not None:
            fv = float(v)
            stats.rc3_n += 1
            if fv < stats.rc3_lo:
                stats.rc3_lo = fv
            if fv > stats.rc3_hi:
                stats.rc3_hi = fv
        return

    if t in ("RCOU", "RCOUT"):
        for ch in (5, 6, 8, 11, 12):
            val = _get(msg, f"C{ch}", f"Chan{ch}")
            if val is None:
                continue
            st = stats.rcou.setdefault(ch, ChanStat())
            lo = stats.parms.get(f"SERVO{ch}_MIN")
            hi = stats.parms.get(f"SERVO{ch}_MAX")
            st.add(float(val), lo, hi)


def analyze_file(path: Path, config_id: str, aircraft: str | None) -> list[Finding]:
    stats = LogStats()
    log = DFReader_binary(str(path))
    while True:
        msg = log.recv_msg()
        if msg is None:
            break
        ingest(stats, msg)
    return findings_from_stats(stats, config_id, aircraft, path)


def findings_from_stats(
    stats: LogStats,
    config_id: str,
    aircraft: str | None,
    path: Path,
) -> list[Finding]:
    findings: list[Finding] = []
    expect = expected_params(config_id, aircraft)
    dur_s = 0.0
    if stats.t_first_us is not None and stats.t_last_us is not None:
        dur_s = max(0.0, (stats.t_last_us - stats.t_first_us) / 1e6)

    findings.append(
        Finding(
            "info",
            f"日志 {path.name}",
            f"固件={stats.firmware or '未知'}; {stats.frame_msg or '无机架MSG'}; "
            f"时长约 {dur_s:.1f}s; 解锁次数={stats.arm_events}; "
            f"模式={_mode_summary(stats)}",
            "台架短日志只能查解锁/脚本/构型，不能代替悬停或过渡试飞。",
        )
    )

    btilt_msgs = [m for m in stats.msgs if "BTILT" in m]
    expected_msg = EXPECTED_BTILT_MSG.get(config_id, "")
    if not btilt_msgs:
        findings.append(
            Finding(
                "error",
                "Lua 脚本未在本日志启动",
                "MSG 中没有 BTILT: 行。",
                f"确认 SCR_ENABLE=1，然后 "
                f"`python scripts/upload-lua.py --port COMx --config {config_id}` 并重启。"
                "起飞前 GCS 必须看到对应 BTILT 运行消息。",
            )
        )
    elif expected_msg and not any(expected_msg in m for m in btilt_msgs):
        findings.append(
            Finding(
                "error",
                "机上 Lua 与 --config 不一致",
                " / ".join(btilt_msgs[:6]),
                f"当前分析按 {config_id}（期望 `{expected_msg}`）。"
                f"切换构型须 full 参数 + `upload-lua.py --config {config_id}`（会删掉另一构型脚本）。",
            )
        )
    else:
        findings.append(
            Finding(
                "info",
                "Lua 已加载",
                " / ".join(btilt_msgs[:4]),
                "脚本在跑；固飞差动仍要台架确认横滚方向（反了改 BTILT_REV=-1）。",
            )
        )

    lua_fail = [
        m
        for m in stats.msgs
        if any(
            k in m.lower()
            for k in ("lua:", "scripting", "add_table", "btile missing", "btile:")
        )
        or "BTILT: missing" in m
        or "BTILT: add_table" in m
    ]
    if lua_fail:
        findings.append(
            Finding(
                "error",
                "脚本运行报错",
                " | ".join(lua_fail[:8]),
                "按报错修 SD 上的 lua 或参数表；bicopter 与 tilttri 脚本不能同时存在。",
            )
        )

    gyro_msgs = [m for m in stats.msgs if "gyro" in m.lower()]
    if gyro_msgs:
        later_armed = stats.arm_events > 0
        sev = "warn" if later_armed else "error"
        findings.append(
            Finding(
                sev,
                "陀螺不一致，拦截过解锁",
                " | ".join(dict.fromkeys(gyro_msgs)),
                "上电后机身静置数秒再解锁；飞控竖装须先设 AHRS_ORIENTATION=16 再做加速度计水平校准。"
                "若一直报：检查减振、USB/线材拉扯、INS 是否在动。项目 ARMING_CHECK 仍包含 IMU，"
                "不要为图省事关掉陀螺检查。",
            )
        )

    accel_msgs = [m for m in stats.msgs if "accel" in m.lower() and "inconsistent" in m.lower()]
    if accel_msgs:
        findings.append(
            Finding(
                "warn",
                "加速度计不一致，拦截过解锁",
                " | ".join(dict.fromkeys(accel_msgs)),
                "上电后放稳再解锁；USB 线不要扯飞控。竖装须 AHRS_ORIENTATION=16 后做过水平校准。",
            )
        )

    prearm = [
        m
        for m in stats.msgs
        if m.startswith("PreArm") or m.startswith("Arm:") or "Arming" in m
    ]
    extra_arm = [
        m
        for m in prearm
        if "gyro" not in m.lower()
        and "accel" not in m.lower()
        and "throttle armed" not in m.lower()
    ]
    if extra_arm:
        findings.append(
            Finding(
                "warn",
                "其它解锁/预解锁提示",
                " | ".join(list(dict.fromkeys(extra_arm))[:12]),
                "对照 docs/ardupilot-setup.md：无 GPS 时不要用 AUTO/QLOITER 等定位模式。",
            )
        )

    if stats.errs:
        findings.append(
            Finding(
                "error",
                "日志 ERR 记录",
                " | ".join(stats.errs[:8]),
                "先处理 ERR（传感器/EKF/FS）；不要在未查清时加大姿态增益。",
            )
        )

    used_pos = [n for n in stats.mode_counts if n in POSITION_MODES]
    gps_off = stats.parms.get("GPS1_TYPE", expect.get("GPS1_TYPE", 0)) == 0
    if used_pos and gps_off:
        names = ", ".join(mode_name(n) for n in used_pos)
        findings.append(
            Finding(
                "error",
                "无 GPS 却进入了定位模式",
                f"出现 {names}",
                "改回 CH8 混控，只保留 QSTABILIZE(17)/STABILIZE(2)/MANUAL(0)。",
            )
        )

    fw_used = any(n in (0, 2) for n in stats.mode_counts)
    q_used = 17 in stats.mode_counts
    if stats.arm_events and not q_used and not fw_used:
        findings.append(
            Finding(
                "warn",
                "解锁了但模式不在本机三档内",
                _mode_summary(stats),
                "检查 FLTMODE1..6 与遥控器混控是否仍为 17,17,2,2,0,0。",
            )
        )

    # Param drift vs repo
    mismatches: list[str] = []
    missing: list[str] = []
    for key in COMPARE_KEYS:
        if key not in expect:
            continue
        if key not in stats.parms:
            missing.append(key)
            continue
        if not _close(stats.parms[key], expect[key]):
            mismatches.append(f"{key} log={stats.parms[key]:g} repo={expect[key]:g}")
    if mismatches:
        findings.append(
            Finding(
                "warn",
                f"与 {config_id} 仓库参数不一致",
                "; ".join(mismatches[:12]),
                f"`python scripts/upload-params.py --port COMx --config {config_id} "
                f"--mode incremental"
                + (f" --aircraft {aircraft}" if aircraft else "")
                + "`。换构型必须 --mode full。",
            )
        )
    if missing and stats.parms:
        findings.append(
            Finding(
                "info",
                "日志未记录部分对比项",
                ", ".join(missing[:10]),
                "BIN 里 PARM 列表不完整时，以飞控 Full Parameter List 为准。",
            )
        )

    horiz_l = stats.parms.get("BTILT_HORIZ_L")
    horiz_r = stats.parms.get("BTILT_HORIZ_R")
    if horiz_l == PLACEHOLDER_HORIZ and horiz_r == PLACEHOLDER_HORIZ:
        findings.append(
            Finding(
                "warn",
                "BTILT_HORIZ_* 仍是占位 1200",
                f"L={horiz_l:g} R={horiz_r:g}",
                "台架把左右外段调到真水平后写入 HORIZ_L/R，再用 "
                f"`export-aircraft-calib.py --config {config_id} --aircraft NN` 入库。",
            )
        )

    s5min = stats.parms.get("SERVO5_MIN")
    s5max = stats.parms.get("SERVO5_MAX")
    if s5min == 1100 and s5max == 2000:
        findings.append(
            Finding(
                "warn",
                "倾转端点仍是 project 占位",
                "SERVO5_MIN/MAX=1100/2000",
                "按 docs/ardupilot-setup.md §4 标定 MIN/TRIM/MAX 与 REVERSED，"
                "不要直接套另一构型的 aircraft overlay。",
            )
        )

    ahrs = stats.parms.get("AHRS_ORIENTATION", expect.get("AHRS_ORIENTATION"))
    if ahrs is not None and not _close(float(ahrs), 16):
        findings.append(
            Finding(
                "error",
                "飞控安装方向参数不是 Roll90",
                f"AHRS_ORIENTATION={ahrs:g}（舱内竖装应为 16）",
                "写入 AHRS_ORIENTATION=16，重启后重新做加速度计水平校准，否则姿态是歪的。",
            )
        )

    scr = stats.parms.get("SCR_ENABLE")
    if scr is not None and int(round(scr)) != 1:
        findings.append(
            Finding(
                "error",
                "脚本功能关闭",
                f"SCR_ENABLE={scr:g}",
                "设 SCR_ENABLE=1 并重启，否则 Lua 不会跑。",
            )
        )

    if stats.vibe_n:
        mx, my, mz = stats.vibe_max
        clips = stats.clip_sum
        if max(mx, my, mz) >= VIBE_RMS_WARN or max(clips) >= VIBE_CLIP_WARN:
            findings.append(
                Finding(
                    "warn",
                    "振动/IMU 削波偏高",
                    f"Vibe max XYZ=({mx:.1f},{my:.1f},{mz:.1f}); Clip={clips}",
                    "先查桨/电机平衡与飞控减振，再考虑 INS_ACCEL_FILTER。"
                    "高振动会误报 Gyros inconsistent，也会让姿态环发抖。",
                )
            )

    if not stats.parms:
        findings.append(
            Finding(
                "info",
                "本日志没有 PARM 快照",
                "无法对比 Q_FRAME_CLASS / SERVO*_FUNCTION / BTILT_*。",
                "新录一段（或 LOG_DISARMED=1 上电即记）再分析参数漂移。",
            )
        )

    if stats.att_n >= 50:
        er = stats.att_roll_err / stats.att_n
        ep = stats.att_pitch_err / stats.att_n
        # Handheld bench spikes max error; use mean so we don't nag pickup motion.
        if er >= ATT_ERR_MEAN_DEG or ep >= ATT_ERR_MEAN_DEG:
            q_mode = 17 in stats.mode_counts
            sug = (
                "垂起：先确认倾转 TRIM 在垂直、Q_OPTIONS bit14 与 Q_ANGLE_MAX=4500；"
                "默认 PID 先别猛加。tilttri 查 Motor4 是否在动；bicopter 查 BPIT_REV/GAIN。"
                if q_mode
                else "固飞：先确认差动方向 BTILT_REV，增益从 0.12 小步加 BTILT_GAIN；"
                "满杆饱和则加大 BTILT_TRAVEL，但先排除机械卡死。"
            )
            findings.append(
                Finding(
                    "warn",
                    "解锁后姿态跟不上期望",
                    f"mean |Roll-Des|={er:.1f}° (max {stats.att_roll_err_max:.1f}°); "
                    f"|Pitch-Des|={ep:.1f}° (max {stats.att_pitch_err_max:.1f}°) n={stats.att_n}",
                    sug,
                )
            )

    _handling_findings(findings, stats)
    _servo_findings(findings, stats, config_id)

    if stats.arm_events == 0:
        findings.append(
            Finding(
                "info",
                "本日志全程未成功解锁",
                "没有 Throttle armed。",
                "先处理 PreArm/Arm 消息；静置过陀螺后再做倾转/电机台架。",
            )
        )
    elif dur_s < 20 and max(stats.mode_counts.values(), default=0) <= 2:
        findings.append(
            Finding(
                "info",
                "更像上电自检，不是试飞",
                f"时长 {dur_s:.1f}s，模式切换很少。",
                "要调增益请录一段：QSTABILIZE 解锁悬停（或绑架）+ 切固飞看 S5/S6 差动。",
            )
        )

    return findings


def _handling_findings(findings: list[Finding], stats: LogStats) -> None:
    p = stats.parms
    if not p:
        return

    slew = p.get("Q_M_SLEW_UP_TIME")
    if slew is not None and slew <= 0.05:
        findings.append(
            Finding(
                "warn",
                "垂起油门上升没有斜率限制",
                f"Q_M_SLEW_UP_TIME={slew:g}（0=瞬时拉满）",
                "QSTABILIZE 油门是直通推力。设 Q_M_SLEW_UP_TIME=0.8，"
                "再把 Q_M_THST_EXPO 提到约 0.80，低油门更细、离地不那么窜。",
            )
        )

    expo = p.get("Q_M_THST_EXPO")
    if expo is not None and expo < 0.75:
        findings.append(
            Finding(
                "warn",
                "推力曲线偏线性，低油门容易离地猛",
                f"Q_M_THST_EXPO={expo:g}（仓库约 0.80）",
                "提到 0.80：同样杆量下底部推力更小。仍猛可再略降 Q_M_SPIN_MIN（默认 0.15→0.12），"
                "怠速不跟转则改回去。",
            )
        )

    tc = p.get("Q_A_INPUT_TC")
    ang_r = p.get("Q_A_ANG_RLL_P")
    ang_p = p.get("Q_A_ANG_PIT_P")
    snappy = (tc is not None and tc <= 0.12) or (
        ang_r is not None and ang_r >= 4.4
    ) or (ang_p is not None and ang_p >= 4.4)
    if snappy:
        findings.append(
            Finding(
                "warn",
                "垂起打舵偏灵敏",
                f"Q_A_INPUT_TC={tc:g} Q_A_ANG_RLL_P={ang_r:g} Q_A_ANG_PIT_P={ang_p:g}",
                "先加滤波、略降角度 P：INPUT_TC=0.25，ANG_RLL/PIT_P=3.5。"
                "不要靠把 Q_ANGLE_MAX 从 45° 砍掉来「变温和」（满杆改出不来）。"
                "仍冲再降 Q_A_RAT_RLL_P / Q_A_RAT_PIT_P，不要先加 D。",
            )
        )

    rc_min = p.get("RC3_MIN")
    if (
        stats.rc3_n >= 20
        and rc_min is not None
        and stats.rc3_lo + 40 < rc_min
    ):
        findings.append(
            Finding(
                "warn",
                "油门校准下限偏高，有效行程被压缩",
                f"RC3_MIN={rc_min:g}，解锁后实际 RC3 最低约 {stats.rc3_lo:.0f}、最高约 {stats.rc3_hi:.0f}",
                "Mission Planner 重新校准遥控：油门最低应对齐真实低位。"
                "MIN 偏高时，杆子先有一段死区，再突然进入映射，离地会更窜。",
            )
        )


def _servo_findings(findings: list[Finding], stats: LogStats, config_id: str) -> None:
    labels = {5: "倾转左 S5", 6: "倾转右 S6", 8: "尾/S8", 11: "S11", 12: "S12"}
    fw = any(n in (0, 2) for n in stats.mode_counts)
    qstab = 17 in stats.mode_counts
    for ch, st in sorted(stats.rcou.items()):
        if st.n < 10:
            continue
        name = labels.get(ch, f"S{ch}")
        if ch in (5, 6) and (st.sat_lo + st.sat_hi) > st.n * 0.4:
            findings.append(
                Finding(
                    "warn",
                    f"{name} PWM 长时间贴端点",
                    f"range {st.lo:.0f}..{st.hi:.0f} µs, sat_lo={st.sat_lo} sat_hi={st.sat_hi} / {st.n}",
                    "机械行程不够或 MIN/MAX 标在结构之外；按 §4.1 重标，避免姿态环饱和。",
                )
            )
        if ch == 8 and config_id == "tilttri" and fw and st.hi > 1100:
            findings.append(
                Finding(
                    "warn",
                    "tilttri 固飞时 S8 不像停转",
                    f"S8 PWM {st.lo:.0f}..{st.hi:.0f}（固飞应靠近 MIN=1000）",
                    "核对 SERVO8_FUNCTION=36 且 MIN/TRIM=1000；固件应在固飞关尾电机。",
                )
            )
        if ch == 8 and config_id == "bicopter" and qstab and stats.arm_events and st.hi <= 1050:
            findings.append(
                Finding(
                    "warn",
                    "bicopter 垂起解锁后尾桨几乎不动",
                    f"S8 PWM {st.lo:.0f}..{st.hi:.0f}",
                    "查 BPIT_ENABLE=1、BPIT_IDLE，以及 Lua 是否加载；电调须单向、TRIM=停转。",
                )
            )
        if ch in (11, 12) and fw and config_id == "bicopter" and st.hi <= 1050 and stats.arm_events:
            findings.append(
                Finding(
                    "warn",
                    "bicopter 固飞油门直通可能没生效",
                    f"{name} 最大 {st.hi:.0f} µs",
                    "见 docs 固飞油门不转：要 BTILT_THR=1 且脚本在跑，否则 stock 会把 73/74 关断。",
                )
            )


def _mode_summary(stats: LogStats) -> str:
    if not stats.modes:
        return "(无 MODE)"
    uniq: list[str] = []
    for _num, name in stats.modes:
        if not uniq or uniq[-1] != name:
            uniq.append(name)
    return " -> ".join(uniq[:20])


def _close(a: float, b: float) -> bool:
    return abs(a - b) < max(1e-3, abs(b) * 1e-4)


def print_report(path: Path, findings: list[Finding]) -> int:
    order = {"error": 0, "warn": 1, "info": 2}
    findings = sorted(findings, key=lambda f: order.get(f.severity, 9))
    print(f"\n======== {path} ========")
    for f in findings:
        tag = {"error": "错误", "warn": "警告", "info": "信息"}.get(f.severity, f.severity)
        print(f"\n[{tag}] {f.title}")
        print(f"  依据: {f.evidence}")
        print(f"  建议: {f.suggestion}")

    print("\n--- 建议优先顺序 ---")
    errors = [f for f in findings if f.severity == "error"]
    warns = [f for f in findings if f.severity == "warn"]
    if not errors and not warns:
        print("未发现必须改的项；若要调手感，录一段解锁后的 QSTABILIZE / 固飞再分析。")
        return 0
    for i, f in enumerate(errors + warns, 1):
        print(f"{i}. {f.title} → {f.suggestion}")
    return 2 if errors else 1


def connect(port: str, baud: int):
    from pymavlink import mavutil

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
        f"comp={master.target_component} type={hb.type}"
    )
    return master


def ftp_session(master):
    from pymavlink.mavftp import MAVFTP

    return MAVFTP(
        master,
        target_system=master.target_system,
        target_component=master.target_component,
    )


def ftp_list_logs(ftp) -> list[tuple[str, int]]:
    ret = ftp.cmd_list(["APM/LOGS"])
    items = getattr(ftp, "list_result", None) or []
    out: list[tuple[str, int]] = []
    for it in items:
        name = getattr(it, "name", "")
        size = int(getattr(it, "size_b", 0) or 0)
        if str(name).upper().endswith(".BIN"):
            out.append((str(name), size))
    out.sort()
    print(f"APM/LOGS list error={ret.error_code} files={len(out)}")
    return out


def ftp_get(ftp, remote: str, local: Path, timeout: float = 900.0) -> bool:
    from pymavlink.mavftp import FtpError

    local.parent.mkdir(parents=True, exist_ok=True)
    done = {"ok": False}

    def on_done(_flen) -> None:
        done["ok"] = True

    def on_progress(frac) -> None:
        if frac is None:
            return
        try:
            pct = int(max(0.0, min(1.0, float(frac))) * 100)
        except (TypeError, ValueError):
            return
        print(f"  {local.name}: {pct}%", end="\r", flush=True)

    print(f"Downloading {remote} -> {local}")
    ret = ftp.cmd_get([remote, str(local)], callback=on_done, progress_callback=on_progress)
    if ret.error_code != FtpError.Success:
        print(f"GET start fail: {ret.error_code}")
        return False
    ftp.process_ftp_reply("OpenFileRO", timeout=timeout)
    print()
    if not done["ok"] or not local.exists() or local.stat().st_size <= 0:
        print("GET fail")
        return False
    print(f"OK {local} ({local.stat().st_size} bytes)")
    return True


def normalize_aircraft_id(raw: str) -> str:
    s = raw.strip()
    if not s.isdigit():
        raise ValueError(f"aircraft id must be numeric, got: {raw!r}")
    n = int(s)
    if n < 0 or n > 99:
        raise ValueError(f"aircraft id must be 0..99, got: {n}")
    return f"{n:02d}"


def parse_log_id(name: str) -> int | None:
    stem = Path(name).stem
    if stem.isdigit():
        return int(stem)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin_files", nargs="*", type=Path, help="Local .BIN paths")
    parser.add_argument("--port", default=None, help="If set, talk to FC on this COM port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_ID,
        help=f"Airframe config id (default: {DEFAULT_CONFIG_ID})",
    )
    parser.add_argument(
        "--aircraft",
        default=None,
        help="Optional NN overlay to compare against (e.g. 01)",
    )
    parser.add_argument(
        "--list-configs",
        action="store_true",
        help="Print known configs and exit",
    )
    parser.add_argument(
        "--list-logs",
        action="store_true",
        help="List APM/LOGS on the FC (--port required)",
    )
    parser.add_argument(
        "--latest",
        type=int,
        default=0,
        metavar="N",
        help="Download and analyze the latest N onboard logs",
    )
    parser.add_argument(
        "--log-id",
        action="append",
        type=int,
        default=[],
        help="Download this log number (repeatable), e.g. --log-id 91",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=REPO_ROOT / "logs_download",
        help="Where to store downloads (default: logs_download/)",
    )
    args = parser.parse_args()

    if args.list_configs:
        print("Known configs:")
        print(format_config_list())
        return 0

    try:
        load_config(args.config)
    except (FileNotFoundError, ValueError) as exc:
        print(exc)
        return 1

    aircraft = None
    if args.aircraft:
        try:
            aircraft = normalize_aircraft_id(args.aircraft)
        except ValueError as exc:
            print(exc)
            return 1

    files: list[Path] = []
    if args.port and (args.list_logs or args.latest or args.log_id):
        master = connect(args.port, args.baud)
        if master is None:
            return 1
        ftp = ftp_session(master)
        listing = ftp_list_logs(ftp)
        if args.list_logs or not listing:
            for name, size in listing:
                print(f"  {name:16} {size:10} bytes")
            if args.list_logs and not args.latest and not args.log_id and not args.bin_files:
                return 0
        wanted: list[str] = []
        if args.log_id:
            ids = set(args.log_id)
            wanted.extend(n for n, _s in listing if parse_log_id(n) in ids)
        if args.latest:
            bins = [n for n, _s in listing]
            wanted.extend(bins[-args.latest :])
        # unique preserve order
        seen: set[str] = set()
        names: list[str] = []
        for n in wanted:
            if n not in seen:
                seen.add(n)
                names.append(n)
        for name in names:
            local = args.out_dir / name
            if local.exists() and local.stat().st_size > 0:
                print(f"Existing {local} ({local.stat().st_size} bytes)")
            elif not ftp_get(ftp, f"APM/LOGS/{name}", local):
                return 2
            files.append(local)

    files.extend(p.resolve() for p in args.bin_files)

    if not files:
        print("没有要分析的文件。传入 .BIN，或加 --port COM4 --latest 1")
        return 1

    worst = 0
    for path in files:
        if not path.is_file():
            print(f"not found: {path}")
            worst = max(worst, 1)
            continue
        print(f"Analyzing {path} ({path.stat().st_size} bytes) config={args.config}"
              + (f" aircraft={aircraft}" if aircraft else ""))
        findings = analyze_file(path, args.config, aircraft)
        worst = max(worst, print_report(path, findings))
    return worst


if __name__ == "__main__":
    sys.exit(main())
