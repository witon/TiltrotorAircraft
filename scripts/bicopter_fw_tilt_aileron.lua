-- BiCopter fixed-wing differential tilt (equivalent aileron)
--
-- In STABILIZE / MANUAL, overrides left/right tilt PWM around HORIZ so outer
-- panels can move both above and below level. VTOL modes leave stock BiCopter
-- firmware in control.
--
-- Deploy: copy to APM/scripts/ on the FC SD card. Requires SCR_ENABLE=1.
-- Servo functions: 75 = TiltMotorLeft (S5), 76 = TiltMotorRight (S6).
--
-- Script params (GCS): BTILT_HORIZ, BTILT_TRAVEL, BTILT_GAIN, BTILT_REV
--   HORIZ  = PWM at true wing-level (FW center), between SERVO_MIN and TRIM
--   TRAVEL = max PWM offset from HORIZ per side at full roll demand
--   GAIN   = 0..1 scale on travel (start low, e.g. 0.3)
--   REV    = 1 or -1; flip if left-stick roll moves surfaces the wrong way
--
-- Sign (REV=1): left roll demand -> left wing decrease AoA, right increase AoA.

local UPDATE_MS = 20
local OVERRIDE_MS = 60

local MODE_MANUAL = 0
local MODE_STABILIZE = 2

local K_TILT_LEFT = 75
local K_TILT_RIGHT = 76
local K_AILERON = 4

local PARAM_TABLE_KEY = 89
local PARAM_TABLE_PREFIX = 'BTILT_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'BTILT: add_table failed')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'HORIZ', 1200), 'BTILT: HORIZ')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'TRAVEL', 100), 'BTILT: TRAVEL')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'GAIN', 0.3), 'BTILT: GAIN')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'REV', 1), 'BTILT: REV')

local p_horiz = Parameter(PARAM_TABLE_PREFIX .. 'HORIZ')
local p_travel = Parameter(PARAM_TABLE_PREFIX .. 'TRAVEL')
local p_gain = Parameter(PARAM_TABLE_PREFIX .. 'GAIN')
local p_rev = Parameter(PARAM_TABLE_PREFIX .. 'REV')

local tilt_left_chan = SRV_Channels:find_channel(K_TILT_LEFT)
local tilt_right_chan = SRV_Channels:find_channel(K_TILT_RIGHT)

if not tilt_left_chan or not tilt_right_chan then
  gcs:send_text(3, 'BTILT: missing SERVO fn 75/76')
  return
end

local roll_rc_chan = 1
do
  local rmap = Parameter()
  if rmap:init('RCMAP_ROLL') then
    local v = rmap:get()
    if v and v >= 1 and v <= 16 then
      roll_rc_chan = math.floor(v)
    end
  end
end

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function fw_mode(mode)
  return mode == MODE_MANUAL or mode == MODE_STABILIZE
end

-- Normalized roll in [-1, 1]. Prefer mixer aileron scaled output; else RC stick.
local function roll_demand()
  local ok, scaled = pcall(function()
    return SRV_Channels:get_output_scaled(K_AILERON)
  end)
  if ok and scaled then
    return clamp(scaled / 4500.0, -1.0, 1.0)
  end

  local pwm = rc:get_pwm(roll_rc_chan)
  if not pwm then
    return 0
  end
  -- Typical 1000..2000 center 1500; deadzone ~30 us
  local norm = (pwm - 1500) / 500.0
  if math.abs(norm) < 0.06 then
    return 0
  end
  return clamp(norm, -1.0, 1.0)
end

local function update()
  local mode = vehicle:get_mode()
  if not fw_mode(mode) then
    return update, UPDATE_MS
  end

  local horiz = p_horiz:get()
  local travel = p_travel:get()
  local gain = p_gain:get()
  local rev = p_rev:get()

  if not horiz or not travel or not gain or not rev then
    return update, UPDATE_MS
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

  -- Left roll (+roll with REV=1): left PWM down (less AoA), right PWM up (more AoA)
  -- assuming lower PWM = more nose-down / less AoA after bench calibration.
  local pwm_l = math.floor(horiz - delta + 0.5)
  local pwm_r = math.floor(horiz + delta + 0.5)

  SRV_Channels:set_output_pwm_chan_timeout(tilt_left_chan, pwm_l, OVERRIDE_MS)
  SRV_Channels:set_output_pwm_chan_timeout(tilt_right_chan, pwm_r, OVERRIDE_MS)

  return update, UPDATE_MS
end

gcs:send_text(6, 'BTILT: fw differential tilt running')
return update, UPDATE_MS
