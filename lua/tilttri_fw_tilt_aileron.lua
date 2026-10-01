-- Tilt-Tri fixed-wing differential tilt (vectored yaw QuadPlane)
-- + FW throttle passthrough (no airspeed, no altitude)
--
-- In STABILIZE / MANUAL:
--   - Overrides left/right tilt PWM around per-side HORIZ (equivalent aileron)
--   - On enter FW: ramps tilt from current PWM to HORIZ at Q_TILT_RATE_DN
--     (falls back to Q_TILT_RATE_UP if DN is 0); differential active during ramp
--   - Overrides Motor2/Motor1 (front L/R) from RC throttle (+ yaw differential)
--     so QuadPlane airspeed-wait hover thrust cannot hold the mains high
--   - Holds Motor4 (tail) at SERVO MIN (stopped)
-- On leave FW (e.g. to QSTABILIZE):
--   - Motor overrides stop immediately so the tri mixer owns all three motors
--   - Tilt keeps being overridden and slews from the current PWM back to the
--     captured VTOL PWM at Q_TILT_RATE_UP. Releasing at once would show the
--     firmware's fully-forward output (past HORIZ, further down) for one
--     step, then slew up from there.
-- Steady VTOL: stock firmware owns tilt and all three motors (Motor1/2/4).
--
-- When FLTMODE_CH=0 the script owns the mode from CH8 (same six PWM
-- slots as Plane). A VTOL->FW request stays in QSTABILIZE and slews tilt
-- common-mode only until both sides reach BTILT_QFRAC of the captured
-- start->HORIZ travel; then it enters the selected FW mode, blends motors
-- for a short interval, and the FW ramp above (with roll differential)
-- finishes the tilt. QFRAC=0 skips the hold. Switching back to VTOL enters
-- QSTABILIZE at once and slews tilt back (BTILT: qrecover) before vectored
-- yaw is released. FLTMODE_CH~=0 keeps the legacy path (FW mode
-- means take tilt and motors at once).
--
-- Deploy with: python scripts/upload-lua.py --config tilttri
-- Requires SCR_ENABLE=1. Servo functions: 75/76 tilt (S5/S6),
--   34/33 front motors (S11/S12), 36 tail (S8).
--
-- Script params (GCS):
--   BTILT_HORIZ_L = left tilt PWM at true wing-level (FW center)
--   BTILT_HORIZ_R = right tilt PWM at true wing-level (FW center)
--   BTILT_TRAVEL  = max PWM offset from HORIZ per side at full roll
--   BTILT_GAIN    = 0..1 scale on tilt travel
--   BTILT_REV     = 1 or -1; tilt roll sign
--   BTILT_THR     = 1 enable FW motor override; 0 tilt-only
--   BTILT_YAWDT   = yaw differential gain (-1..1; neg flips sign; ~0.1)
--   BTILT_QFRAC   = 0..1 fraction of start->HORIZ kept in QSTABILIZE
--                   when FLTMODE_CH=0; 0 skips the hold (default 0.7)
--
-- Tilt sign (REV=1): left roll -> left wing decrease AoA, right increase AoA.
-- Right servo is mirrored: both sides use the same PWM offset (horiz_n - delta).
-- Yaw sign (YAWDT>0): right yaw stick -> left thrust up, right thrust down.
-- Throttle uses the mapped RC channel MIN/MAX, not a fixed 1000..2000 span.

local UPDATE_MS = 20
local OVERRIDE_MS = 60
local RAMP_EPS_PWM = 2
-- TRIM <-> HORIZ treated as ~90 deg for rate scaling
local TRIM_HORIZ_DEG = 90.0
local DEFAULT_TILT_RATE_DPS = 40.0

local MODE_MANUAL = 0
local MODE_STABILIZE = 2
-- One table keeps the main chunk under ArduPilot's 100-local limit.
-- SLOT matches Plane::readSwitch (docs/ardupilot-setup.md). FLTMODE1..6 = 17,17,2,2,0,0.
local Q = {
  QSTAB = 17,
  CH = 8,
  DEBOUNCE_MS = 100,
  BLEND_MS = 300,
  SLOT = {17, 17, 2, 2, 0, 0},
  VTOL = 0,
  HOLD = 1,
  FW = 2,
  RECOVER = 3,
  phase = 0,
  start_l = nil,
  start_r = nil,
  desired = nil,
  pending_slot = nil,
  pending_since = 0,
  blend_t0 = nil,
  blend_m1 = nil,
  blend_m2 = nil,
  blend_m4 = nil,
  rec_l = nil,
  rec_r = nil,
  out_l = nil,
  out_r = nil,
}

local K_TILT_LEFT = 75
local K_TILT_RIGHT = 76
local K_AILERON = 4
local K_MOTOR1 = 33
local K_MOTOR2 = 34
local K_MOTOR4 = 36

-- Tilt params use table key 89 (HORIZ_L, TRAVEL, GAIN, REV). Key 101: HORIZ_R.
-- Key 100 (size 2): FW throttle THR / YAWDT. Key 98 (size 1): QFRAC.
-- Do not assert optional tables: a conflict must not kill tilt.
-- Do not register tail tables (102-104).
local PARAM_TABLE_KEY = 89
local PARAM_TABLE_KEY_HR = 101
local PARAM_TABLE_KEY_THR = 100
local PARAM_TABLE_KEY_QFRAC = 98
local PARAM_TABLE_PREFIX = 'BTILT_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'BTILT: add_table 89 failed')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'HORIZ_L', 1200), 'BTILT: HORIZ_L')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TRAVEL', 100), 'BTILT: TRAVEL')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'GAIN', 0.12), 'BTILT: GAIN')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'REV', 1), 'BTILT: REV')

assert(param:add_table(PARAM_TABLE_KEY_HR, PARAM_TABLE_PREFIX, 1), 'BTILT: add_table 101 failed')
assert(param:add_param(PARAM_TABLE_KEY_HR, 1, 'HORIZ_R', 1200), 'BTILT: HORIZ_R')

if param:add_table(PARAM_TABLE_KEY_THR, PARAM_TABLE_PREFIX, 2) then
  param:add_param(PARAM_TABLE_KEY_THR, 1, 'THR', 1)
  param:add_param(PARAM_TABLE_KEY_THR, 2, 'YAWDT', 0.1)
else
  gcs:send_text(4, 'BTILT: throttle table unavailable, using defaults')
end

if param:add_table(PARAM_TABLE_KEY_QFRAC, PARAM_TABLE_PREFIX, 1) then
  param:add_param(PARAM_TABLE_KEY_QFRAC, 1, 'QFRAC', 0.7)
else
  gcs:send_text(4, 'BTILT: qfrac table unavailable, using 0.7')
end

local function bind_param(name)
  local p = Parameter()
  if p:init(name) then
    return p
  end
  return nil
end

local p_horiz_l = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ_L')
local p_horiz_r = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ_R')
local p_travel = Parameter(PARAM_TABLE_PREFIX .. 'TRAVEL')
local p_gain = Parameter(PARAM_TABLE_PREFIX .. 'GAIN')
local p_rev = Parameter(PARAM_TABLE_PREFIX .. 'REV')
local p_thr = bind_param(PARAM_TABLE_PREFIX .. 'THR')
local p_yawdt = bind_param(PARAM_TABLE_PREFIX .. 'YAWDT')
local p_qfrac = bind_param(PARAM_TABLE_PREFIX .. 'QFRAC')

local p_fltmode_ch = Parameter()
local have_fltmode_ch = p_fltmode_ch:init('FLTMODE_CH')

local tilt_left_chan = SRV_Channels:find_channel(K_TILT_LEFT)
local tilt_right_chan = SRV_Channels:find_channel(K_TILT_RIGHT)

if not tilt_left_chan or not tilt_right_chan then
  gcs:send_text(3, 'BTILT: missing SERVO fn 75/76')
  return
end

local mot1_chan = SRV_Channels:find_channel(K_MOTOR1)
local mot2_chan = SRV_Channels:find_channel(K_MOTOR2)
local mot4_chan = SRV_Channels:find_channel(K_MOTOR4)
if not mot1_chan or not mot2_chan or not mot4_chan then
  gcs:send_text(3, 'BTILT: missing SERVO fn 33/34/36 (tilt only)')
end

local roll_rc_chan = 1
local thr_rc_chan = 3
local yaw_rc_chan = 4
do
  local function rcmap(name, default)
    local rmap = Parameter()
    if rmap:init(name) then
      local v = rmap:get()
      if v and v >= 1 and v <= 16 then
        return math.floor(v)
      end
    end
    return default
  end
  roll_rc_chan = rcmap('RCMAP_ROLL', 1)
  thr_rc_chan = rcmap('RCMAP_THROTTLE', 3)
  yaw_rc_chan = rcmap('RCMAP_YAW', 4)
end

local function servo_lim(chan_0based, which, default)
  local p = Parameter()
  local name = string.format('SERVO%u_%s', chan_0based + 1, which)
  if p:init(name) then
    local v = p:get()
    if v then
      return math.floor(v)
    end
  end
  return default
end

local tilt_left_trim = servo_lim(tilt_left_chan, 'TRIM', 1500)
local tilt_right_trim = servo_lim(tilt_right_chan, 'TRIM', 1500)

local function rc_lim(chan_1based, which, default)
  local p = Parameter()
  local name = string.format('RC%u_%s', chan_1based, which)
  if p:init(name) then
    local v = p:get()
    if v then
      return math.floor(v)
    end
  end
  return default
end

local thr_rc_min = rc_lim(thr_rc_chan, 'MIN', 1000)
local thr_rc_max = rc_lim(thr_rc_chan, 'MAX', 2000)
if thr_rc_max <= thr_rc_min then
  thr_rc_min = 1000
  thr_rc_max = 2000
end

local mot1_min, mot1_max = 1000, 2000
local mot2_min, mot2_max = 1000, 2000
local mot4_min = 1000
if mot1_chan then
  mot1_min = servo_lim(mot1_chan, 'MIN', 1000)
  mot1_max = servo_lim(mot1_chan, 'MAX', 2000)
end
if mot2_chan then
  mot2_min = servo_lim(mot2_chan, 'MIN', 1000)
  mot2_max = servo_lim(mot2_chan, 'MAX', 2000)
end
if mot4_chan then
  mot4_min = servo_lim(mot4_chan, 'MIN', 1000)
end

local p_tilt_rate_up = Parameter()
local p_tilt_rate_dn = Parameter()
local have_rate_up = p_tilt_rate_up:init('Q_TILT_RATE_UP')
local have_rate_dn = p_tilt_rate_dn:init('Q_TILT_RATE_DN')

local was_fw = false
local cur_l = tilt_left_trim
local cur_r = tilt_right_trim

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function fw_mode(mode)
  return mode == MODE_MANUAL or mode == MODE_STABILIZE
end

local function tilt_rate_dps()
  local rate = DEFAULT_TILT_RATE_DPS
  if have_rate_dn then
    local dn = p_tilt_rate_dn:get()
    if dn and dn > 0 then
      rate = dn
    elseif have_rate_up then
      local up = p_tilt_rate_up:get()
      if up and up > 0 then
        rate = up
      end
    end
  elseif have_rate_up then
    local up = p_tilt_rate_up:get()
    if up and up > 0 then
      rate = up
    end
  end
  return rate
end

local function read_tilt_pwm(servo_fn, fallback)
  local ok, pwm = pcall(function()
    return SRV_Channels:get_output_pwm(servo_fn)
  end)
  if ok and pwm and type(pwm) == 'number' and pwm > 0 then
    return math.floor(pwm)
  end
  return fallback
end

local function approach(cur, target, max_step)
  local err = target - cur
  if err > max_step then
    return cur + max_step
  end
  if err < -max_step then
    return cur - max_step
  end
  return target
end

local function ramp_max_step(trim_pwm, horiz_pwm, rate_dps)
  local span = math.abs(trim_pwm - horiz_pwm)
  if span < 1 then
    span = 1
  end
  local step = span * (rate_dps / TRIM_HORIZ_DEG) * (UPDATE_MS / 1000.0)
  if step < 0.5 then
    step = 0.5
  end
  return step
end

local function stick_norm(rc_chan)
  local pwm = rc:get_pwm(rc_chan)
  if not pwm then
    return 0
  end
  local norm = (pwm - 1500) / 500.0
  if math.abs(norm) < 0.06 then
    return 0
  end
  return clamp(norm, -1.0, 1.0)
end

local function roll_demand()
  local ok, scaled = pcall(function()
    return SRV_Channels:get_output_scaled(K_AILERON)
  end)
  if ok and scaled then
    return clamp(scaled / 4500.0, -1.0, 1.0)
  end
  return stick_norm(roll_rc_chan)
end

-- Throttle stick 0..1 from the mapped RC channel endpoints.
-- Near the calibrated minimum is zero. No airspeed or altitude.
local function throttle_norm()
  local pwm = rc:get_pwm(thr_rc_chan)
  if not pwm then
    return 0
  end
  local span = thr_rc_max - thr_rc_min
  if span < 1 then
    span = 1
  end
  local norm = (pwm - thr_rc_min) / span
  if norm < 0.02 then
    return 0
  end
  return clamp(norm, 0.0, 1.0)
end

local function is_armed()
  local ok, armed = pcall(function()
    return arming:is_armed()
  end)
  return ok and armed
end

local function write_motor(chan, pwm)
  SRV_Channels:set_output_pwm_chan_timeout(chan, pwm, OVERRIDE_MS)
end

local function pwm_from_norm(lo, hi, norm)
  return math.floor(lo + norm * (hi - lo) + 0.5)
end

-- FW motor targets. nil when override is off (no channels or BTILT_THR=0).
-- Tail is always MIN. Disarmed fronts are MIN. Armed fronts follow throttle
-- plus yaw differential.
local function fw_motor_targets()
  if not mot1_chan and not mot2_chan and not mot4_chan then
    return nil
  end

  if p_thr then
    local thr_en = p_thr:get()
    if not thr_en or thr_en < 0.5 then
      return nil
    end
  end

  local tail = mot4_min
  if not is_armed() then
    return mot2_min, mot1_min, tail
  end

  local yawdt = 0.1
  if p_yawdt then
    local v = p_yawdt:get()
    if v then
      yawdt = v
    end
  end
  yawdt = clamp(yawdt, -1.0, 1.0)

  local base = throttle_norm()
  -- Right yaw (+): left (Motor2) up, right (Motor1) down
  local diff = 0.5 * stick_norm(yaw_rc_chan) * yawdt
  local left = clamp(base + diff, 0.0, 1.0)
  local right = clamp(base - diff, 0.0, 1.0)
  return pwm_from_norm(mot2_min, mot2_max, left),
    pwm_from_norm(mot1_min, mot1_max, right),
    tail
end

local function write_fw_motors(left, right, tail)
  if mot4_chan then
    write_motor(mot4_chan, tail)
  end
  if mot2_chan then
    write_motor(mot2_chan, left)
  end
  if mot1_chan then
    write_motor(mot1_chan, right)
  end
end

-- millis() is uint32 userdata: '/' stays integer and math.floor() rejects it.
local function now_ms()
  return millis():tofloat()
end

local function sample_motors_for_blend()
  Q.blend_m2 = read_tilt_pwm(K_MOTOR2, mot2_min)
  Q.blend_m1 = read_tilt_pwm(K_MOTOR1, mot1_min)
  Q.blend_m4 = read_tilt_pwm(K_MOTOR4, mot4_min)
  Q.blend_t0 = now_ms()
end

-- FW: front motors follow the throttle stick (+ yaw diff). Tail stays at MIN.
-- Disarmed: all three at MIN. BTILT_THR=0: no motor override.
-- After qhold crosses QFRAC, slew from the sampled mixer PWM for Q.BLEND_MS.
local function update_motors()
  local left, right, tail = fw_motor_targets()
  if Q.blend_t0 and (now_ms() - Q.blend_t0) >= Q.BLEND_MS then
    Q.blend_t0 = nil
  end
  if not left then
    return
  end
  if Q.blend_t0 then
    local u = clamp((now_ms() - Q.blend_t0) / Q.BLEND_MS, 0.0, 1.0)
    write_fw_motors(
      math.floor(Q.blend_m2 + (left - Q.blend_m2) * u + 0.5),
      math.floor(Q.blend_m1 + (right - Q.blend_m1) * u + 0.5),
      math.floor(Q.blend_m4 + (tail - Q.blend_m4) * u + 0.5))
    return
  end
  write_fw_motors(left, right, tail)
end

-- with_diff false: common-mode slew only (qhold). Default true.
local function update_tilt(entering_fw, with_diff)
  if with_diff == nil then
    with_diff = true
  end
  local horiz_l = p_horiz_l:get()
  local horiz_r = p_horiz_r:get()
  local travel = p_travel:get()
  local gain = p_gain:get()
  local rev = p_rev:get()

  if not horiz_l or not horiz_r or not travel or not gain or not rev then
    return
  end

  if travel < 0 then travel = 0 end
  gain = clamp(gain, 0.0, 1.0)
  if rev >= 0 then
    rev = 1
  else
    rev = -1
  end

  if entering_fw then
    cur_l = read_tilt_pwm(K_TILT_LEFT, tilt_left_trim)
    cur_r = read_tilt_pwm(K_TILT_RIGHT, tilt_right_trim)
  end

  local rate = tilt_rate_dps()
  local step_l = ramp_max_step(tilt_left_trim, horiz_l, rate)
  local step_r = ramp_max_step(tilt_right_trim, horiz_r, rate)
  cur_l = approach(cur_l, horiz_l, step_l)
  cur_r = approach(cur_r, horiz_r, step_r)

  local at_level = math.abs(cur_l - horiz_l) <= RAMP_EPS_PWM
    and math.abs(cur_r - horiz_r) <= RAMP_EPS_PWM
  if at_level then
    cur_l = horiz_l
    cur_r = horiz_r
  end

  local delta = 0
  if with_diff then
    delta = roll_demand() * rev * travel * gain
  end

  local pwm_l = math.floor(cur_l - delta + 0.5)
  local pwm_r = math.floor(cur_r - delta + 0.5)
  Q.out_l = pwm_l
  Q.out_r = pwm_r

  SRV_Channels:set_output_pwm_chan_timeout(tilt_left_chan, pwm_l, OVERRIDE_MS)
  SRV_Channels:set_output_pwm_chan_timeout(tilt_right_chan, pwm_r, OVERRIDE_MS)
end

local function lua_owns_mode()
  if not have_fltmode_ch then
    return false
  end
  local v = p_fltmode_ch:get()
  return v ~= nil and v == 0
end

local function qfrac_value()
  local q = 0.7
  if p_qfrac then
    local v = p_qfrac:get()
    if v then
      q = v
    end
  end
  return clamp(q, 0.0, 1.0)
end

-- Remaining fraction of the TRIM->HORIZ span. Trim is the ~90 deg yardstick
-- already used for rate scaling; on this airframe it sits near vertical.
local function remain_frac(cur, horiz, trim)
  local span = math.abs(horiz - trim)
  if span < 1 then
    return 0
  end
  return math.abs(horiz - cur) / span
end

local function still_far(cur_l_now, cur_r_now, horiz_l, horiz_r, qfrac)
  local worst = math.max(
    remain_frac(cur_l_now, horiz_l, tilt_left_trim),
    remain_frac(cur_r_now, horiz_r, tilt_right_trim))
  return worst > (1.0 - qfrac)
end

local function side_progress(cur, start_pwm, horiz)
  local span = horiz - start_pwm
  if math.abs(span) < 1 then
    return 1
  end
  return clamp((cur - start_pwm) / span, 0.0, 1.0)
end

-- Plane::readSwitch bands, not a rescale of RC8_MIN/MAX. The transmitter
-- mix is built for these edges (docs/ardupilot-setup.md §5).
local function slot_from_pwm(pwm)
  if pwm > 1230 and pwm <= 1360 then return 2 end
  if pwm > 1360 and pwm <= 1490 then return 3 end
  if pwm > 1490 and pwm <= 1620 then return 4 end
  if pwm > 1620 and pwm <= 1749 then return 5 end
  if pwm >= 1750 then return 6 end
  return 1
end

-- Invalid PWM leaves the last accepted mode unchanged.
local function poll_desired_mode()
  local pwm = rc:get_pwm(Q.CH)
  if not pwm or pwm <= 900 or pwm >= 2200 then
    return Q.desired
  end
  local slot = slot_from_pwm(pwm)
  local now = millis()
  if Q.pending_slot ~= slot then
    Q.pending_slot = slot
    Q.pending_since = now
    return Q.desired
  end
  local dt = now - Q.pending_since
  if dt < 0 or dt >= Q.DEBOUNCE_MS then
    Q.desired = Q.SLOT[slot]
  end
  return Q.desired
end

local function ensure_mode(mode)
  local ok, cur = pcall(function()
    return vehicle:get_mode()
  end)
  if ok and cur == mode then
    return
  end
  pcall(function()
    vehicle:set_mode(mode)
  end)
end

-- Slew tilt back to the VTOL PWM captured at qhold entry. Motors are not
-- overridden here; mode is already QSTABILIZE so the tri mixer runs.
local function update_recover()
  local horiz_l = p_horiz_l:get()
  local horiz_r = p_horiz_r:get()
  local rate = DEFAULT_TILT_RATE_DPS
  if have_rate_up then
    local up = p_tilt_rate_up:get()
    if up and up > 0 then
      rate = up
    end
  end
  local step_l = ramp_max_step(tilt_left_trim, horiz_l or (tilt_left_trim + 1), rate)
  local step_r = ramp_max_step(tilt_right_trim, horiz_r or (tilt_right_trim + 1), rate)
  cur_l = approach(cur_l, Q.rec_l, step_l)
  cur_r = approach(cur_r, Q.rec_r, step_r)
  local pwm_l = math.floor(cur_l + 0.5)
  local pwm_r = math.floor(cur_r + 0.5)
  Q.out_l = pwm_l
  Q.out_r = pwm_r
  SRV_Channels:set_output_pwm_chan_timeout(tilt_left_chan, pwm_l, OVERRIDE_MS)
  SRV_Channels:set_output_pwm_chan_timeout(tilt_right_chan, pwm_r, OVERRIDE_MS)
  if math.abs(cur_l - Q.rec_l) <= RAMP_EPS_PWM
    and math.abs(cur_r - Q.rec_r) <= RAMP_EPS_PWM then
    Q.phase = Q.VTOL
    gcs:send_text(6, 'BTILT: qrecover done')
  end
end

local function start_recover()
  if Q.phase == Q.HOLD then
    gcs:send_text(6, 'BTILT: qhold release')
  end
  gcs:send_text(6, 'BTILT: qrecover')
  if Q.out_l then
    cur_l = Q.out_l
  end
  if Q.out_r then
    cur_r = Q.out_r
  end
  Q.rec_l = Q.start_l or tilt_left_trim
  Q.rec_r = Q.start_r or tilt_right_trim
  Q.phase = Q.RECOVER
  was_fw = false
  Q.blend_t0 = nil
  ensure_mode(Q.QSTAB)
  update_recover()
end

local function enter_fw_now(desired, recapture)
  Q.phase = Q.FW
  was_fw = true
  ensure_mode(desired)
  update_tilt(recapture, true)
  update_motors()
end

local function finish_qhold(desired)
  sample_motors_for_blend()
  Q.phase = Q.FW
  was_fw = true
  gcs:send_text(6, 'BTILT: qhold done')
  ensure_mode(desired)
  -- Common-mode output was already written this tick. Differential
  -- resumes on the next pass; do not step the ramp twice.
  update_motors()
end

local function update_qhold()
  local desired = poll_desired_mode()
  if desired == nil then
    return
  end

  if desired == Q.QSTAB then
    if Q.phase == Q.HOLD or Q.phase == Q.FW then
      start_recover()
    elseif Q.phase == Q.RECOVER then
      update_recover()
    end
    return
  end

  if desired ~= MODE_MANUAL and desired ~= MODE_STABILIZE then
    return
  end

  local kept_pwm = false
  if Q.phase == Q.RECOVER then
    Q.phase = Q.VTOL
    kept_pwm = true
  end

  if Q.phase == Q.FW then
    ensure_mode(desired)
    update_tilt(false, true)
    update_motors()
    return
  end

  if Q.phase == Q.VTOL then
    local qfrac = qfrac_value()
    local horiz_l = p_horiz_l:get()
    local horiz_r = p_horiz_r:get()
    local now_l = kept_pwm and cur_l or read_tilt_pwm(K_TILT_LEFT, tilt_left_trim)
    local now_r = kept_pwm and cur_r or read_tilt_pwm(K_TILT_RIGHT, tilt_right_trim)
    if qfrac <= 0 or not horiz_l or not horiz_r
      or not still_far(now_l, now_r, horiz_l, horiz_r, qfrac) then
      enter_fw_now(desired, true)
      return
    end
    Q.phase = Q.HOLD
    Q.start_l = now_l
    Q.start_r = now_r
    cur_l = now_l
    cur_r = now_r
    gcs:send_text(6, 'BTILT: qhold')
  end

  if Q.phase ~= Q.HOLD or not Q.start_l or not Q.start_r then
    return
  end

  ensure_mode(Q.QSTAB)
  update_tilt(false, false)

  local horiz_l = p_horiz_l:get()
  local horiz_r = p_horiz_r:get()
  if not horiz_l or not horiz_r then
    return
  end
  local prog = math.min(
    side_progress(cur_l, Q.start_l, horiz_l),
    side_progress(cur_r, Q.start_r, horiz_r))
  if prog >= qfrac_value() then
    finish_qhold(desired)
  end
end

local function update()
  if lua_owns_mode() then
    update_qhold()
    return update, UPDATE_MS
  end

  local mode = vehicle:get_mode()
  local in_fw = fw_mode(mode)

  if in_fw then
    local entering_fw = not was_fw
    was_fw = true
    Q.phase = Q.FW
    update_tilt(entering_fw)
    update_motors()
    return update, UPDATE_MS
  end

  was_fw = false
  Q.blend_t0 = nil
  Q.phase = Q.VTOL
  return update, UPDATE_MS
end

gcs:send_text(6, 'BTILT: tilttri fw tilt+throttle running')
if lua_owns_mode() then
  gcs:send_text(6, 'BTILT: qhold on')
end
return update, UPDATE_MS
