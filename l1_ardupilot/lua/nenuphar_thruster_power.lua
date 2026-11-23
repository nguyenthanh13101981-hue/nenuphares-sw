-- nenuphar_thruster_power.lua
-- Tang L1 (Pixhawk 6C / ArduPilot Rover) - CdCT Final 2.0 Muc A.7:
--   1. Tron luc day 4 che do M1-M4 (2 thruster chinh Trai/Phai + 1 du phong duoi).
--   2. Gioi han dong tong: 70 A (M1 binh thuong), 40 A (M2/M3/M4 du phong), <= 30 A/ESC.
--
-- Tham so tiem loi SITL:
--   SCR_USER1: Mat na loi thruster (0=M1 binh thuong, 1=M2 hong Trai, 2=M3 hong Phai, 3=M4 hong ca 2 chinh)
--   SCR_USER2: Dong tong gia lap (A) khi > 0 de kiem thu qua dong tren SITL

local K_LEFT   = 94 -- SERVO1_FUNCTION = Script 1 (Thruster chinh Trai)
local K_RIGHT  = 95 -- SERVO2_FUNCTION = Script 2 (Thruster chinh Phai)
local K_STERN  = 96 -- SERVO3_FUNCTION = Script 3 (Thruster du phong duoi)

local CTRL_THROTTLE = 3
local CTRL_YAW      = 4

local LIMIT_NORMAL_A = 70.0
local LIMIT_BACKUP_A = 40.0
local ESC_MAX_A      = 30.0

local last_telem_ms = 0
local last_mode = 0

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function read_total_current_a()
  local sim_curr = param:get("SCR_USER2")
  if sim_curr and sim_curr > 0.0 then
    return sim_curr
  end
  local batt_curr = battery:current_amps(0)
  if batt_curr then
    return math.max(0.0, batt_curr)
  end
  return 0.0
end

local function update()
  local fault_mask = math.floor((param:get("SCR_USER1") or 0) + 0.5)
  local mix_mode = 1
  if fault_mask == 1 then
    mix_mode = 2
  elseif fault_mask == 2 then
    mix_mode = 3
  elseif fault_mask >= 3 then
    mix_mode = 4
  end

  local limit_a = (mix_mode == 1) and LIMIT_NORMAL_A or LIMIT_BACKUP_A
  local current_a = read_total_current_a()

  -- Nen cong suat muot khi dong tong vuot nguong 70 A / 40 A
  local curr_scale = 1.0
  if current_a > limit_a and current_a > 0.1 then
    curr_scale = clamp(limit_a / current_a, 0.1, 1.0)
  end

  local throttle = vehicle:get_control_output(CTRL_THROTTLE) or 0.0
  local yaw      = vehicle:get_control_output(CTRL_YAW) or 0.0
  local left_out, right_out, stern_out = 0.0, 0.0, 0.0

  if arming:is_armed() then
    if mix_mode == 1 then
      -- M1: 2 thruster chinh Trai + Phai
      left_out  = clamp(throttle + yaw, -1.0, 1.0)
      right_out = clamp(throttle - yaw, -1.0, 1.0)
      stern_out = 0.0
    elseif mix_mode == 2 then
      -- M2: Hong Trai -> Phai + Du phong duoi
      left_out  = 0.0
      right_out = clamp(0.7 * throttle - 0.5 * yaw, -1.0, 1.0)
      stern_out = clamp(0.7 * throttle + 0.5 * yaw, -1.0, 1.0)
    elseif mix_mode == 3 then
      -- M3: Hong Phai -> Trai + Du phong duoi
      left_out  = clamp(0.7 * throttle + 0.5 * yaw, -1.0, 1.0)
      right_out = 0.0
      stern_out = clamp(0.7 * throttle - 0.5 * yaw, -1.0, 1.0)
    else
      -- M4: Hong ca 2 chinh -> Cuu ho bang Du phong duoi (gioi han 30 A/ESC)
      left_out  = 0.0
      right_out = 0.0
      stern_out = clamp(0.6 * throttle, -0.6, 0.6)
    end

    -- Ap dung he so nen dong tong va bao ve tran 30 A/ESC
    local per_esc_cap = (mix_mode == 4) and math.min(1.0, ESC_MAX_A / LIMIT_BACKUP_A) or 1.0
    local final_scale = math.min(curr_scale, per_esc_cap)
    left_out  = left_out  * final_scale
    right_out = right_out * final_scale
    stern_out = stern_out * final_scale
  end

  SRV_Channels:set_output_norm(K_LEFT,  left_out)
  SRV_Channels:set_output_norm(K_RIGHT, right_out)
  SRV_Channels:set_output_norm(K_STERN, stern_out)

  if mix_mode ~= last_mode then
    last_mode = mix_mode
    gcs:send_text(4, string.format("NENUPHAR MIX_MODE=M%d LIMIT=%.0fA", mix_mode, limit_a))
  end

  local now_ms = millis():toint()
  if (now_ms - last_telem_ms) >= 500 then
    last_telem_ms = now_ms
    gcs:send_named_float("MIX_MODE", mix_mode)
    gcs:send_named_float("CURR_LIM_A", limit_a)
    gcs:send_named_float("CURR_SCALE", curr_scale)
    gcs:send_named_float("STERN_OUT", stern_out)
  end

  return update, 20 -- 50 Hz
end

gcs:send_text(6, "nenuphar_thruster_power.lua loaded (M1-M4, 70A/40A)")
return update()
