-- nenuphar_watchdog_bridge.lua
-- Tang L1 (Pixhawk 6C / ArduPilot Rover) - CdCT Final 2.0 Muc A.7:
--   1. Watchdog heartbeat Pi 5 (L2 compid=191): mat > 3.0 s -> chuyen HOLD (mode 4).
--   2. Khoa phep qua cau (BRIDGE_PERMIT = SCR_USER4) & noi long FS_GCS_TIMEOUT
--      trong vung cau (BRIDGE_ZONE = SCR_USER5).

local MODE_HOLD             = 4
local HEARTBEAT_MSG_ID      = 0
local L2_COMP_ID            = 191
local L2_WD_TIMEOUT_MS      = 3000
local GCS_TIMEOUT_DEFAULT_S = 5
local GCS_TIMEOUT_BRIDGE_S  = 30

local seen_l2_hb = false
local last_l2_hb_ms = 0
local wd_tripped = false
local bridge_lock_active = false
local last_gcs_timeout = -1
local last_telem_ms = 0

mavlink:init(16, 1)
mavlink:register_rx_msgid(HEARTBEAT_MSG_ID)

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
  for _ = 1, 8 do
    local msg, _ = mavlink:receive_chan()
    if not msg then
      break
    end
    if extract_compid(msg) == L2_COMP_ID then
      seen_l2_hb = true
      last_l2_hb_ms = now_ms
      if wd_tripped then
        wd_tripped = false
        gcs:send_text(6, "L2_WD_RECOVERED: Pi 5 heartbeat restored")
      end
    end
  end
end

local function update()
  local now_ms = millis():toint()
  poll_l2_heartbeat(now_ms)

  -- 1. Watchdog heartbeat Pi 5 (L2 -> L1)
  if seen_l2_hb and (now_ms - last_l2_hb_ms) > L2_WD_TIMEOUT_MS then
    if not wd_tripped then
      wd_tripped = true
      vehicle:set_mode(MODE_HOLD)
      gcs:send_text(3, "L2_WD_TRIP: Pi 5 heartbeat lost >3s -> HOLD")
    elseif vehicle:get_mode() ~= MODE_HOLD then
      vehicle:set_mode(MODE_HOLD)
    end
  end

  -- 2. Khoa phep qua cau (BRIDGE_PERMIT) & noi long GCS timeout trong BRIDGE_ZONE
  local bridge_permit = math.floor((param:get("SCR_USER4") or 0) + 0.5)
  local bridge_zone   = math.floor((param:get("SCR_USER5") or 0) + 0.5)

  if bridge_zone >= 1 then
    if bridge_permit < 1 then
      if not bridge_lock_active then
        bridge_lock_active = true
        gcs:send_text(3, "BRIDGE_PERMIT_DENIED: holding before bridge")
      end
      if vehicle:get_mode() ~= MODE_HOLD then
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

gcs:send_text(6, "nenuphar_watchdog_bridge.lua loaded (L2 WD 3s, bridge lock)")
return update()
