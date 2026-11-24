-- nenuphar_watchdog_bridge.lua
-- Tang L1 (Pixhawk 6C / ArduPilot Rover 4.7.3) - CdCT Final 2.0 Muc A.7:
--   1. Watchdog heartbeat Pi 5 (L2 compid=191): mat > 3.0 s -> chuyen HOLD (mode 4).
--   2. Khoa phep qua cau co han dung TTL (PERMIT_TTL_MS = 15 s):
--      BRIDGE_PERMIT = SCR_USER4, BRIDGE_ZONE = SCR_USER5.
--   3. Khong chong lai quyen gianh dieu khien thu cong cua nguoi van hanh (MANUAL=0, STEERING=3).

local MODE_MANUAL           = 0
local MODE_STEERING         = 3
local MODE_HOLD             = 4
local HEARTBEAT_MSG_ID      = 0
local L2_COMP_ID            = 191
local L2_WD_TIMEOUT_MS      = 3000
local PERMIT_TTL_MS         = 15000
local GCS_TIMEOUT_DEFAULT_S = 5
local GCS_TIMEOUT_BRIDGE_S  = 30

local seen_l2_hb = false
local last_l2_hb_ms = 0
local wd_tripped = false
local bridge_lock_active = false
local last_gcs_timeout = -1
local last_telem_ms = 0

local prev_permit_val = 0
local has_permit_ts = false
local last_permit_ms = 0

mavlink:init(16, 1)
mavlink:register_rx_msgid(HEARTBEAT_MSG_ID)

local function is_human_override(mode)
  return mode == MODE_MANUAL or mode == MODE_STEERING
end

local function extract_compid(raw_msg)
  -- ArduPilot mavlink:receive_chan() returns mavlink_message_t layout:
  -- [1..2]=checksum, [3]=magic (0xFD/0xFE), [7]=seq, [8]=sysid, [9]=compid
  if not raw_msg or #raw_msg < 9 then
    return -1
  end
  local magic = string.byte(raw_msg, 3)
  if magic == 0xFD or magic == 0xFE then
    return string.byte(raw_msg, 9)
  end
  return -1
end

local function poll_l2_heartbeat(now_ms)
  local got_hb = false
  for _ = 1, 8 do
    local msg, _ = mavlink:receive_chan()
    if not msg then
      break
    end
    if extract_compid(msg) == L2_COMP_ID then
      seen_l2_hb = true
      last_l2_hb_ms = now_ms
      got_hb = true
      if wd_tripped then
        wd_tripped = false
        gcs:send_text(6, "L2_WD_RECOVERED: Pi 5 heartbeat restored")
      end
    end
  end
  return got_hb
end

local function update()
  local now_ms = millis():toint()
  local got_hb_this_tick = poll_l2_heartbeat(now_ms)
  local cur_mode = vehicle:get_mode()
  local l2_alive = seen_l2_hb and (now_ms - last_l2_hb_ms) <= L2_WD_TIMEOUT_MS

  -- 1. Watchdog heartbeat Pi 5 (L2 -> L1); khong ep neu dang o MANUAL / STEERING
  if seen_l2_hb and not l2_alive then
    if not wd_tripped then
      wd_tripped = true
      if not is_human_override(cur_mode) then
        vehicle:set_mode(MODE_HOLD)
      end
      gcs:send_text(3, "L2_WD_TRIP: Pi 5 heartbeat lost >3s -> HOLD")
    elseif cur_mode ~= MODE_HOLD and not is_human_override(cur_mode) then
      vehicle:set_mode(MODE_HOLD)
    end
  end

  -- 2. Khoa phep qua cau (BRIDGE_PERMIT) co TTL & noi long GCS timeout trong BRIDGE_ZONE
  local bridge_permit = math.floor((param:get("SCR_USER4") or 0) + 0.5)
  local bridge_zone   = math.floor((param:get("SCR_USER5") or 0) + 0.5)

  if bridge_permit >= 1 then
    if bridge_permit ~= prev_permit_val or got_hb_this_tick then
      has_permit_ts = true
      last_permit_ms = now_ms
    end
  else
    has_permit_ts = false
  end
  prev_permit_val = bridge_permit

  local permit_valid = (bridge_permit >= 1) and has_permit_ts and
                       ((now_ms - last_permit_ms) <= PERMIT_TTL_MS) and
                       (not seen_l2_hb or l2_alive)

  if bridge_zone >= 1 then
    if not permit_valid then
      if not bridge_lock_active then
        bridge_lock_active = true
        gcs:send_text(3, "BRIDGE_PERMIT_DENIED_OR_EXPIRED: holding before bridge")
      end
      if cur_mode ~= MODE_HOLD and not is_human_override(cur_mode) then
        vehicle:set_mode(MODE_HOLD)
      end
    else
      bridge_lock_active = false
      if last_gcs_timeout ~= GCS_TIMEOUT_BRIDGE_S then
        param:set("FS_GCS_TIMEOUT", GCS_TIMEOUT_BRIDGE_S)
        last_gcs_timeout = GCS_TIMEOUT_BRIDGE_S
        gcs:send_text(6, "BRIDGE_ZONE_ACTIVE: FS_GCS_TIMEOUT relaxed to 30s")
      end
    end
  else
    bridge_lock_active = false
    if last_gcs_timeout ~= GCS_TIMEOUT_DEFAULT_S then
      param:set("FS_GCS_TIMEOUT", GCS_TIMEOUT_DEFAULT_S)
      last_gcs_timeout = GCS_TIMEOUT_DEFAULT_S
    end
  end

  if (now_ms - last_telem_ms) >= 500 then
    last_telem_ms = now_ms
    gcs:send_named_float("L2_WD_TRIP", wd_tripped and 1.0 or 0.0)
    gcs:send_named_float("BRG_LOCK", bridge_lock_active and 1.0 or 0.0)
  end

  return update, 100 -- 10 Hz
end

gcs:send_text(6, "nenuphar_watchdog_bridge.lua loaded (L2 WD 3s, bridge TTL, human override safe)")
return update()
