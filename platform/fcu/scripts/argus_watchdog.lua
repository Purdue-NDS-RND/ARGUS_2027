-- ARGUS companion watchdog (PLT-09). ArduCopter 4.5 or later, on the flight controller.
--
-- If the companion computer's setpoints stop for more than a second while the
-- vehicle is flying in GUIDED, switch to LOITER: a vehicle whose companion has
-- died should hold its place for the safety pilot, not fly on its last order.
--
-- "Stop" needs a start. The watchdog counts SET_POSITION_TARGET_LOCAL_NED,
-- SET_POSITION_TARGET_GLOBAL_INT and SET_ATTITUDE_TARGET messages, and is
-- armed only once ARM_AFTER of them have come in this stretch of flying in
-- GUIDED. The count starts again whenever the vehicle is not in GUIDED, is
-- disarmed, is not flying, or is taking off, so a take-off flown by the
-- flight controller itself (no setpoints, as the missions' take-off and the
-- delivery's relaunch are) is left alone, and so is a vehicle standing on the
-- ground between a set-down and a relaunch.
--
-- A mission that goes quiet in the air in GUIDED for more than a second after
-- it has been sending setpoints is treated as a dead companion. Fly each
-- mission with this script loaded in the sim before it is loaded on the
-- aircraft (platform/fcu/README.md).
--
-- Load: SCR_ENABLE 1, reboot, copy this file to APM/scripts on the SD card,
-- reboot. It says "ARGUS watchdog loaded" once and "ARGUS watchdog: ..." when
-- it acts (each under the 50 characters a status text carries). The messages it counts are still handled by the autopilot as usual.

local MODE_GUIDED = 4
local MODE_LOITER = 5
local TIMEOUT_MS = 1000 -- setpoints stopped for longer than this: act
local ARM_AFTER = 5 -- setpoints in this stretch before a silence counts as a stop
local PERIOD_MS = 50
local SET_ATTITUDE_TARGET = 82
local SET_POSITION_TARGET_LOCAL_NED = 84
local SET_POSITION_TARGET_GLOBAL_INT = 86
local SEVERITY_CRITICAL = 2
local SEVERITY_INFO = 6

mavlink:init(20, 3)
mavlink:register_rx_msgid(SET_ATTITUDE_TARGET)
mavlink:register_rx_msgid(SET_POSITION_TARGET_LOCAL_NED)
mavlink:register_rx_msgid(SET_POSITION_TARGET_GLOBAL_INT)

local seen = 0 -- setpoints in this stretch
local last_ms = nil -- when the last one came

local function update()
    local now = millis()
    while true do
        local message = mavlink:receive_chan()
        if not message then
            break
        end
        seen = seen + 1
        last_ms = now
    end

    if vehicle:get_mode() ~= MODE_GUIDED or not arming:is_armed() or not vehicle:get_likely_flying() or vehicle:is_taking_off() then
        seen = 0
        last_ms = nil
        return update, PERIOD_MS
    end

    if seen >= ARM_AFTER and last_ms and (now - last_ms):toint() > TIMEOUT_MS then
        if vehicle:set_mode(MODE_LOITER) then
            gcs:send_text(SEVERITY_CRITICAL, "ARGUS watchdog: setpoints stopped, now LOITER")
        else
            gcs:send_text(SEVERITY_CRITICAL, "ARGUS watchdog: setpoints stopped, LOITER refused")
        end
        seen = 0
        last_ms = nil
    end
    return update, PERIOD_MS
end

gcs:send_text(SEVERITY_INFO, "ARGUS watchdog loaded")
return update, 1000
