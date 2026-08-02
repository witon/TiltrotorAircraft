-- BiCopter fixed-wing differential tilt + FW throttle passthrough
--
-- In STABILIZE / MANUAL:
--   - Overrides left/right tilt PWM around per-side HORIZ (equivalent aileron)
--   - On enter FW: ramps tilt from current/TRIM to HORIZ at Q_TILT_RATE_DN
--     (falls back to Q_TILT_RATE_UP if DN is 0); differential only after level
--   - Overrides ThrottleLeft/Right from RC throttle (+ yaw differential)
--     so stock BiCopter motors SHUT_DOWN / twin-mix fight does not zero S11/S12
-- VTOL modes (e.g. QSTABILIZE) leave stock firmware in control (tilt rate via
-- Q_TILT_RATE_UP when returning to hover).
--
-- Deploy: copy to APM/scripts/ on the FC SD card. Requires SCR_ENABLE=1.
-- Servo functions: 75/76 tilt (S5/S6), 73/74 throttle L/R (S11/S12).
--
-- Script params (GCS):
--   BTILT_HORIZ_L = left tilt PWM at true wing-level (FW center)
--   BTILT_HORIZ_R = right tilt PWM at true wing-level (FW center)
--   BTILT_TRAVEL  = max PWM offset from HORIZ per side at full roll
--   BTILT_GAIN    = 0..1 scale on tilt travel
--   BTILT_REV     = 1 or -1; tilt roll sign
--   BTILT_THR     = 1 enable FW throttle override; 0 tilt-only
--   BTILT_YAWDT   = yaw differential gain (-1..1; neg flips sign; ~0.1 like RUDD_DT)
--
-- Tilt sign (REV=1): left roll -> left wing decrease AoA, right increase AoA.
-- Right servo is mirrored: both sides use the same PWM offset (horiz_n - delta).
-- Yaw sign (YAWDT>0): right yaw stick -> left thrust up, right thrust down (Plane twin mix).

local UPDATE_MS = 20
local OVERRIDE_MS = 60
local RAMP_EPS_PWM = 2
-- TRIM <-> HORIZ treated as ~90 deg for rate scaling (matches BiCopter full stroke)
local TRIM_HORIZ_DEG = 90.0
local DEFAULT_TILT_RATE_DPS = 40.0

local MODE_MANUAL = 0
local MODE_STABILIZE = 2

local K_TILT_LEFT = 75
local K_TILT_RIGHT = 76
local K_THR_LEFT = 73
local K_THR_RIGHT = 74
local K_AILERON = 4

-- Table key 89 (size 4): tilt params. Key 100 (size 2): FW throttle.
-- Key 101 (size 1): right HORIZ (table 89 cannot expand past 4 slots).
-- ArduPilot cannot expand an existing table's slot count.
local PARAM_TABLE_KEY = 89
local PARAM_TABLE_KEY_THR = 100
local PARAM_TABLE_KEY_HR = 101
local PARAM_TABLE_PREFIX = 'BTILT_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'BTILT: add_table 89 failed')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'HORIZ_L', 1200), 'BTILT: HORIZ_L')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TRAVEL', 100), 'BTILT: TRAVEL')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'GAIN', 0.3), 'BTILT: GAIN')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'REV', 1), 'BTILT: REV')

assert(param:add_table(PARAM_TABLE_KEY_THR, PARAM_TABLE_PREFIX, 2), 'BTILT: add_table 100 failed')
assert(param:add_param(PARAM_TABLE_KEY_THR, 1, 'THR', 1), 'BTILT: THR')
assert(param:add_param(PARAM_TABLE_KEY_THR, 2, 'YAWDT', 0.1), 'BTILT: YAWDT')

assert(param:add_table(PARAM_TABLE_KEY_HR, PARAM_TABLE_PREFIX, 1), 'BTILT: add_table 101 failed')
assert(param:add_param(PARAM_TABLE_KEY_HR, 1, 'HORIZ_R', 1200), 'BTILT: HORIZ_R')

local p_horiz_l = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ_L')
local p_horiz_r = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ_R')
local p_travel = Parameter(PARAM_TABLE_PREFIX .. 'TRAVEL')
local p_gain = Parameter(PARAM_TABLE_PREFIX .. 'GAIN')
local p_rev = Parameter(PARAM_TABLE_PREFIX .. 'REV')
local p_thr = Parameter(PARAM_TABLE_PREFIX .. 'THR')
local p_yawdt = Parameter(PARAM_TABLE_PREFIX .. 'YAWDT')

local tilt_left_chan = SRV_Channels:find_channel(K_TILT_LEFT)
local tilt_right_chan = SRV_Channels:find_channel(K_TILT_RIGHT)

if not tilt_left_chan or not tilt_right_chan then
  gcs:send_text(3, 'BTILT: missing SERVO fn 75/76')
  return
end

local thr_left_chan = SRV_Channels:find_channel(K_THR_LEFT)
local thr_right_chan = SRV_Channels:find_channel(K_THR_RIGHT)
local thr_ok = thr_left_chan and thr_right_chan
if not thr_ok then
  gcs:send_text(3, 'BTILT: missing SERVO fn 73/74 (tilt only)')
end

-- RC map (1-based channel numbers)
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

-- SERVOn_MIN/MAX for motor channels (find_channel is 0-based index)
local thr_left_min, thr_left_max = 1000, 2000
local thr_right_min, thr_right_max = 1000, 2000
if thr_ok then
  thr_left_min = servo_lim(thr_left_chan, 'MIN', 1000)
  thr_left_max = servo_lim(thr_left_chan, 'MAX', 2000)
  thr_right_min = servo_lim(thr_right_chan, 'MIN', 1000)
  thr_right_max = servo_lim(thr_right_chan, 'MAX', 2000)
end

local tilt_left_trim = servo_lim(tilt_left_chan, 'TRIM', 1500)
local tilt_right_trim = servo_lim(tilt_right_chan, 'TRIM', 1500)

local p_tilt_rate_up = Parameter()
local p_tilt_rate_dn = Parameter()
local have_rate_up = p_tilt_rate_up:init('Q_TILT_RATE_UP')
local have_rate_dn = p_tilt_rate_dn:init('Q_TILT_RATE_DN')

-- Ramp state (PWM). Reset when leaving FW modes.
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

-- Normalized stick in [-1, 1] from RC PWM (center 1500, deadzone ~30 us)
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

-- Throttle stick 0..1 from RC (min~1000 max~2000); uses RC channel min deadzone
local function throttle_norm()
  local pwm = rc:get_pwm(thr_rc_chan)
  if not pwm then
    return 0
  end
  -- Typical low 1000, high 2000; small deadzone at bottom
  local norm = (pwm - 1000) / 1000.0
  if norm < 0.02 then
    return 0
  end
  return clamp(norm, 0.0, 1.0)
end

-- Normalized roll in [-1, 1]. Prefer mixer aileron scaled output; else RC stick.
local function roll_demand()
  local ok, scaled = pcall(function()
    return SRV_Channels:get_output_scaled(K_AILERON)
  end)
  if ok and scaled then
    return clamp(scaled / 4500.0, -1.0, 1.0)
  end
  return stick_norm(roll_rc_chan)
end

local function is_armed()
  local ok, armed = pcall(function()
    return arming:is_armed()
  end)
  return ok and armed
end

local function update_tilt(entering_fw)
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
  local delta = 0
  if at_level then
    cur_l = horiz_l
    cur_r = horiz_r
    local roll = roll_demand() * rev
    delta = roll * travel * gain
  end

  -- Left roll (+roll with REV=1): both PWM down; left less AoA, right (mirrored) more AoA
  local pwm_l = math.floor(cur_l - delta + 0.5)
  local pwm_r = math.floor(cur_r - delta + 0.5)

  SRV_Channels:set_output_pwm_chan_timeout(tilt_left_chan, pwm_l, OVERRIDE_MS)
  SRV_Channels:set_output_pwm_chan_timeout(tilt_right_chan, pwm_r, OVERRIDE_MS)
end

local function update_throttle()
  if not thr_ok then
    return
  end

  local thr_en = p_thr:get()
  if not thr_en or thr_en < 0.5 then
    return
  end

  local yawdt = p_yawdt:get()
  if not yawdt then
    yawdt = 0
  end
  yawdt = clamp(yawdt, -1.0, 1.0)

  local base = 0.0
  if is_armed() then
    base = throttle_norm()
  end

  -- Right yaw (+): left up, right down (matches Plane servos_twin_engine_mix)
  local yaw = stick_norm(yaw_rc_chan)
  local diff = 0.5 * yaw * yawdt
  local left = clamp(base + diff, 0.0, 1.0)
  local right = clamp(base - diff, 0.0, 1.0)

  local pwm_l = math.floor(thr_left_min + left * (thr_left_max - thr_left_min) + 0.5)
  local pwm_r = math.floor(thr_right_min + right * (thr_right_max - thr_right_min) + 0.5)

  SRV_Channels:set_output_pwm_chan_timeout(thr_left_chan, pwm_l, OVERRIDE_MS)
  SRV_Channels:set_output_pwm_chan_timeout(thr_right_chan, pwm_r, OVERRIDE_MS)
end

local function update()
  local mode = vehicle:get_mode()
  local in_fw = fw_mode(mode)
  if not in_fw then
    was_fw = false
    return update, UPDATE_MS
  end

  local entering_fw = not was_fw
  was_fw = true

  update_tilt(entering_fw)
  update_throttle()

  return update, UPDATE_MS
end

gcs:send_text(6, 'BTILT: fw tilt+throttle running')
return update, UPDATE_MS
