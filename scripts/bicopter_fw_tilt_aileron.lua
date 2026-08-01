-- BiCopter fixed-wing differential tilt + FW throttle passthrough
--
-- In STABILIZE / MANUAL:
--   - Overrides left/right tilt PWM around HORIZ (equivalent aileron)
--   - Overrides ThrottleLeft/Right from RC throttle (+ yaw differential)
--     so stock BiCopter motors SHUT_DOWN / twin-mix fight does not zero S11/S12
-- VTOL modes (e.g. QSTABILIZE) leave stock firmware in control.
--
-- Deploy: copy to APM/scripts/ on the FC SD card. Requires SCR_ENABLE=1.
-- Servo functions: 75/76 tilt (S5/S6), 73/74 throttle L/R (S11/S12).
--
-- Script params (GCS):
--   BTILT_HORIZ  = PWM at true wing-level (FW center)
--   BTILT_TRAVEL = max PWM offset from HORIZ per side at full roll
--   BTILT_GAIN   = 0..1 scale on tilt travel
--   BTILT_REV    = 1 or -1; tilt roll sign
--   BTILT_THR    = 1 enable FW throttle override; 0 tilt-only
--   BTILT_YAWDT  = yaw differential gain (-1..1; neg flips sign; ~0.1 like RUDD_DT)
--
-- Tilt sign (REV=1): left roll -> left wing decrease AoA, right increase AoA.
-- Right servo is mirrored: both sides use the same PWM offset (horiz - delta).
-- Yaw sign (YAWDT>0): right yaw stick -> left thrust up, right thrust down (Plane twin mix).

local UPDATE_MS = 20
local OVERRIDE_MS = 60

local MODE_MANUAL = 0
local MODE_STABILIZE = 2

local K_TILT_LEFT = 75
local K_TILT_RIGHT = 76
local K_THR_LEFT = 73
local K_THR_RIGHT = 74
local K_AILERON = 4

-- Table key 89 (size 4): existing tilt params. Key 100 (size 2): FW throttle.
-- ArduPilot cannot expand an existing table's slot count.
local PARAM_TABLE_KEY = 89
local PARAM_TABLE_KEY_THR = 100
local PARAM_TABLE_PREFIX = 'BTILT_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'BTILT: add_table 89 failed')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'HORIZ', 1200), 'BTILT: HORIZ')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TRAVEL', 100), 'BTILT: TRAVEL')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'GAIN', 0.3), 'BTILT: GAIN')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'REV', 1), 'BTILT: REV')

assert(param:add_table(PARAM_TABLE_KEY_THR, PARAM_TABLE_PREFIX, 2), 'BTILT: add_table 100 failed')
assert(param:add_param(PARAM_TABLE_KEY_THR, 1, 'THR', 1), 'BTILT: THR')
assert(param:add_param(PARAM_TABLE_KEY_THR, 2, 'YAWDT', 0.1), 'BTILT: YAWDT')

local p_horiz = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ')
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

-- SERVOn_MIN/MAX for motor channels (find_channel is 0-based index)
local thr_left_min, thr_left_max = 1000, 2000
local thr_right_min, thr_right_max = 1000, 2000
if thr_ok then
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
  thr_left_min = servo_lim(thr_left_chan, 'MIN', 1000)
  thr_left_max = servo_lim(thr_left_chan, 'MAX', 2000)
  thr_right_min = servo_lim(thr_right_chan, 'MIN', 1000)
  thr_right_max = servo_lim(thr_right_chan, 'MAX', 2000)
end

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function fw_mode(mode)
  return mode == MODE_MANUAL or mode == MODE_STABILIZE
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

local function update_tilt()
  local horiz = p_horiz:get()
  local travel = p_travel:get()
  local gain = p_gain:get()
  local rev = p_rev:get()

  if not horiz or not travel or not gain or not rev then
    return
  end

  if travel < 0 then travel = 0 end
  gain = clamp(gain, 0.0, 1.0)
  if rev >= 0 then
    rev = 1
  else
    rev = -1
  end

  local roll = roll_demand() * rev
  local delta = roll * travel * gain

  -- Left roll (+roll with REV=1): both PWM down; left less AoA, right (mirrored) more AoA
  local pwm_l = math.floor(horiz - delta + 0.5)
  local pwm_r = math.floor(horiz - delta + 0.5)

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
  if not fw_mode(mode) then
    return update, UPDATE_MS
  end

  update_tilt()
  update_throttle()

  return update, UPDATE_MS
end

gcs:send_text(6, 'BTILT: fw tilt+throttle running')
return update, UPDATE_MS
