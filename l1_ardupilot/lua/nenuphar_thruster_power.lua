-- nenuphar_thruster_power.lua
-- Tang L1 (Pixhawk 6C / ArduPilot Rover 4.7.3) - CdCT Final 2.0 Muc A.7 & TR-5.6:
--   1. Tron luc day 4 che do M1-M4 (2 thruster chinh Trai/Phai + 1 du phong duoi).
--      Luu y M4: chi con thruster duoi tren duong tam, KHONG co momen quay lai (day thang thoat cau).
--   2. Loc thong thap EMA + gioi han toc do thay doi (slew-rate) cho nen dong 70 A / 40 A (tranh dao dong 25-50 Hz).
--   3. Phat hien ket/hong ESC qua telemetry + thu toi da 3 chu ky dao chieu go rong (TR-5.6) truoc khi chot loi.
--
-- Tham so tinh chinh / tiem loi SITL:
--   SCR_USER1: Mat na loi thruster thu cong (0=M1, 1=M2 hong Trai, 2=M3 hong Phai, 3=M4 hong ca 2 chinh)
--   SCR_USER2: Dong tong gia lap (A) khi > 0 de kiem thu qua dong tren SITL
--   SCR_USER3: Mat na ket ESC gia lap (bit 0=Trai, bit 1=Phai) de test chu trinh go rong <= 3 lan

local K_LEFT   = 94 -- SERVO1_FUNCTION = Script 1 (Thruster chinh Trai)
local K_RIGHT  = 95 -- SERVO2_FUNCTION = Script 2 (Thruster chinh Phai)
local K_STERN  = 96 -- SERVO3_FUNCTION = Script 3 (Thruster du phong duoi)

local CTRL_THROTTLE = 3
local CTRL_YAW      = 4

local LIMIT_NORMAL_A = 70.0
local LIMIT_BACKUP_A = 40.0
local ESC_MAX_A      = 30.0

-- Bo loc thong thap + slew-rate chong dao dong (20 ms/chu ky = 50 Hz)
local EMA_ALPHA        = 0.25
local SCALE_SLEW_DOWN  = 0.12
local SCALE_SLEW_UP    = 0.03

-- Phat hien ket ESC & quy trinh go rong (TR-5.6)
local MAX_UNJAM_CYCLES = 3
local UNJAM_PULSE_MS   = 200
local STALL_CURRENT_A  = 18.0
local STALL_MIN_RPM    = 120
local STALL_DEBOUNCE_N = 10 -- 200 ms lien tuc

local filtered_current_a = 0.0
local curr_scale = 1.0
local last_telem_ms = 0
local last_mode = 0

-- Trang thai phat hien ket/hong cho 2 thruster chinh (1=Trai, 2=Phai)
local stall_counts = {0, 0}
local unjam_cycles = {0, 0}
local unjam_until_ms = {0, 0}
local esc_latched_fault = {false, false}

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

local function detect_esc_stall(idx, cmd_mag, now_ms)
  -- Ho tro tiem loi SITL qua SCR_USER3 (bit 0=Trai, bit 1=Phai)
  local sim_stall_mask = math.floor((param:get("SCR_USER3") or 0) + 0.5)
  local sim_stalled = (idx == 1 and (sim_stall_mask % 2 == 1)) or
                      (idx == 2 and (math.floor(sim_stall_mask / 2) % 2 == 1))

  local hw_stalled = false
  if esc_telem and esc_telem.get_rpm and esc_telem.get_current and esc_telem.get_voltage then
    local volt = esc_telem:get_voltage(idx - 1)
    if volt and volt > 1.0 then
      local rpm = esc_telem:get_rpm(idx - 1) or 0
      local cur = esc_telem:get_current(idx - 1) or 0
      if cmd_mag > 0.35 and cur > STALL_CURRENT_A and rpm < STALL_MIN_RPM then
        hw_stalled = true
      end
    end
  end

  if sim_stalled or hw_stalled then
    stall_counts[idx] = stall_counts[idx] + 1
  else
    stall_counts[idx] = 0
  end

  if stall_counts[idx] >= STALL_DEBOUNCE_N and not esc_latched_fault[idx] then
    if now_ms >= unjam_until_ms[idx] then
      if unjam_cycles[idx] < MAX_UNJAM_CYCLES then
        unjam_cycles[idx] = unjam_cycles[idx] + 1
        unjam_until_ms[idx] = now_ms + UNJAM_PULSE_MS
        stall_counts[idx] = 0
        gcs:send_text(4, string.format("ESC%d_UNJAM_PULSE %d/%d", idx, unjam_cycles[idx], MAX_UNJAM_CYCLES))
      else
        esc_latched_fault[idx] = true
        gcs:send_text(3, string.format("ESC%d_FAULT_LATCHED after %d unjam cycles", idx, MAX_UNJAM_CYCLES))
      end
    end
  end
end

local function update()
  local now_ms = millis():toint()
  local throttle = vehicle:get_control_output(CTRL_THROTTLE) or 0.0
  local yaw      = vehicle:get_control_output(CTRL_YAW) or 0.0

  if arming:is_armed() then
    detect_esc_stall(1, math.abs(throttle + yaw), now_ms)
    detect_esc_stall(2, math.abs(throttle - yaw), now_ms)
  end

  -- Ket hop mat na loi thu cong (SCR_USER1) va loi phat hien tu ESC telemetry
  local manual_mask = math.floor((param:get("SCR_USER1") or 0) + 0.5)
  local auto_mask = (esc_latched_fault[1] and 1 or 0) + (esc_latched_fault[2] and 2 or 0)
  local fault_mask = (manual_mask > 0) and manual_mask or auto_mask

  local mix_mode = 1
  if fault_mask == 1 then
    mix_mode = 2
  elseif fault_mask == 2 then
    mix_mode = 3
  elseif fault_mask >= 3 then
    mix_mode = 4
  end

  local limit_a = (mix_mode == 1) and LIMIT_NORMAL_A or LIMIT_BACKUP_A
  local raw_curr_a = read_total_current_a()
  filtered_current_a = EMA_ALPHA * raw_curr_a + (1.0 - EMA_ALPHA) * filtered_current_a

  -- Bo nen dong co loc EMA + gioi han slew-rate (tranh dao dong 25-50 Hz)
  local target_scale = 1.0
  if filtered_current_a > limit_a and filtered_current_a > 0.1 then
    target_scale = clamp(limit_a / filtered_current_a, 0.1, 1.0)
  end
  if target_scale < curr_scale then
    curr_scale = math.max(target_scale, curr_scale - SCALE_SLEW_DOWN)
  elseif target_scale > curr_scale then
    curr_scale = math.min(target_scale, curr_scale + SCALE_SLEW_UP)
  end

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
      -- M4: Hong ca 2 chinh -> Day thang thoat cau bang Du phong duoi (khong lai huong)
      left_out  = 0.0
      right_out = 0.0
      stern_out = clamp(0.6 * throttle, -0.6, 0.6)
    end

    -- Xung dao chieu go rong <= 3 lan neu dang trong cua so unjam
    if now_ms < unjam_until_ms[1] and not esc_latched_fault[1] then
      left_out = -0.35
    end
    if now_ms < unjam_until_ms[2] and not esc_latched_fault[2] then
      right_out = -0.35
    end

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

  if (now_ms - last_telem_ms) >= 500 then
    last_telem_ms = now_ms
    gcs:send_named_float("MIX_MODE", mix_mode)
    gcs:send_named_float("CURR_LIM_A", limit_a)
    gcs:send_named_float("CURR_SCALE", curr_scale)
    gcs:send_named_float("STERN_OUT", stern_out)
  end

  return update, 20 -- 50 Hz
end

gcs:send_text(6, "nenuphar_thruster_power.lua loaded (M1-M4, EMA/slew 70A/40A, unjam<=3)")
return update()
