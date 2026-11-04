-- This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
-- If a copy of the bCDDL was not distributed with this
-- file, You can obtain one at http://beamng.com/bCDDL-1.1.txt

--Purpose: Controls automatic gear selection and clutch handoffs for dual-clutch transmissions
--
-- phevDct: copy of BeamNG's stock shiftLogic/dctGearbox.lua (BeamNG.drive 0.40.3), modified for the
-- Plug-in Hybrid Swap mod. Changes are marked with "PHEV:". All other code is unchanged.
--   * electric drive-away: when the hybrid module reports launchMode "electric" (motor drives, engine
--     decoupled), the active clutch closes quickly from standstill instead of slipping up from idle
--   * the hybrid module reports a dynamic minimum speed (idleAV): 0 in electric operation, engine idle
--     when the engine is coupled; stall protection uses it and no longer divides by zero
--   * "engine running" follows the hybrid system's ready state, so 0 rpm at standstill is not a stall
--   * engine-led launches: creep rules without throttle; with throttle the slipping clutch closes from the creep
--     point at idle as the shaft revs above idle
--   * synchronised clutch handover for shifts between forward gears (motor matches the shaft speed)
--   * load-based shift points with the engine coupled in D (pedal, not BeamNG's driving aggression)

local M = {}

local max = math.max
local min = math.min
local abs = math.abs

local constants = {rpmToAV = 0.104719755, avToRPM = 9.549296596425384}

local newDesiredGearIndex = 0
local previousGearIndex = 0
local shiftAggression = 1
local gearbox = nil
local engine = nil

local sharedFunctions = nil
local gearboxAvailableLogic = nil
local gearboxLogic = nil

M.gearboxHandling = nil
M.timer = nil
M.timerConstants = nil
M.inputValues = nil
M.shiftPreventionData = nil
M.shiftBehavior = nil
M.smoothedValues = nil

M.currentGearIndex = 0
M.maxGearIndex = 0
M.minGearIndex = 0
M.throttle = 0
M.brake = 0
M.clutchRatio = 0
M.shiftingAggression = 0
M.isArcadeSwitched = false
M.isSportModeActive = false
M.isManualModeActive = false

M.smoothedAvgAVInput = 0
M.rpm = 0
M.idleRPM = 0
M.maxRPM = 0

M.engineThrottle = 0
M.engineLoad = 0
M.engineTorque = 0
M.flywheelTorque = 0
M.gearboxTorque = 0

M.ignition = true
M.isEngineRunning = 0

M.oilTemp = 0
M.waterTemp = 0
M.checkEngine = false

M.energyStorages = {}

local automaticHandling = {
  availableModes = {"P", "R", "N", "D", "S", "1", "2", "M"},
  hShifterModeLookup = {[-1] = "R", [0] = "N", "P", "D", "S", "2", "1", "M1"},
  availableModeLookup = {},
  existingModeLookup = {},
  modeIndexLookup = {},
  modes = {},
  mode = nil,
  modeIndex = 0,
  maxAllowedGearIndex = 0,
  minAllowedGearIndex = 0,
  autoDownShiftInM = true,
  defaultForwardMode = "D"
}

local clutchHandling = {
  clutchLaunchTargetAV = 0,
  clutchLaunchStartAV = 0,
  clutchLaunchIFactor = 0,
  revMatchThrottle = 0.5,
  didRevMatch = false,
  didCutIgnition = false,
  isClutchOpenLaunch = false
}

local dct = {
  access1 = {clutchRatioName = "clutchRatio1", gearIndexName = "gearIndex1", setGearIndexName = "setGearIndex1"},
  access2 = {clutchRatioName = "clutchRatio2", gearIndexName = "gearIndex2", setGearIndexName = "setGearIndex2"},
  primaryAccess = nil,
  secondaryAccess = nil,
  clutchTime = 0
}

--shift LEDs are in use if we are in manual control over the gear selection
local function areShiftLEDsInUse()
  return M.gearboxHandling.behavior ~= "arcade" and M.isManualModeActive
end

local function getGearName()
  local modePrefix = ""
  if automaticHandling.mode == "S" and M.currentGearIndex > 0 then
    modePrefix = "S"
  elseif string.sub(automaticHandling.mode, 1, 1) == "M" then
    modePrefix = "M"
  end
  if M.currentGearIndex ~= 0 then
    return modePrefix ~= "" and modePrefix .. tostring(M.currentGearIndex) or automaticHandling.mode
  else
    return automaticHandling.mode
  end
end

local function getGearPosition()
  return (automaticHandling.modeIndex - 1) / (#automaticHandling.modes - 1), automaticHandling.modeIndex
end

local function applyGearboxModeRestrictions()
  local manualModeIndex
  if string.sub(automaticHandling.mode, 1, 1) == "M" then
    manualModeIndex = string.sub(automaticHandling.mode, 2)
  end
  local maxGearIndex = gearbox.maxGearIndex
  local minGearIndex = gearbox.minGearIndex
  if automaticHandling.mode == "1" then
    maxGearIndex = 1
    minGearIndex = 1
  elseif automaticHandling.mode == "2" then
    maxGearIndex = 2
    minGearIndex = 1
  elseif manualModeIndex then
    maxGearIndex = manualModeIndex
    minGearIndex = manualModeIndex
  end

  automaticHandling.maxGearIndex = maxGearIndex
  automaticHandling.minGearIndex = minGearIndex
end

local function gearboxBehaviorChanged(behavior)
  gearboxLogic = gearboxAvailableLogic[behavior]
  M.updateGearboxGFX = gearboxLogic.inGear
  M.shiftUp = gearboxLogic.shiftUp
  M.shiftDown = gearboxLogic.shiftDown
  M.shiftToGearIndex = gearboxLogic.shiftToGearIndex
end

local function calculateShiftAggression()
  local gearRatioDifference = abs(gearbox.gearRatios[previousGearIndex] - gearbox.gearRatios[newDesiredGearIndex])
  local inertiaCoef = linearScale(engine.inertia, 0.1, 0.5, 0.1, 1)
  local gearRatioCoef = linearScale(gearRatioDifference * inertiaCoef, 0.5, 1, 1, 0.5)
  local aggressionCoef = linearScale(M.smoothedValues.drivingAggression, 0.5, 1, 0.1, 1)

  shiftAggression = clamp(gearRatioCoef * aggressionCoef, 0.1, 1)
  M.shiftingAggression = shiftAggression
  --print(string.format("GR: %.2f, AG: %.2f, IN: %.2f -> %.2f", gearRatioCoef, aggressionCoef, inertiaCoef, shiftAggression))
end

local function applyGearboxMode()
  local autoIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
  if autoIndex then
    automaticHandling.modeIndex = min(max(autoIndex, 1), #automaticHandling.modes)
    automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  end

  if automaticHandling.mode == "P" then
    gearbox:setMode("park")
  elseif automaticHandling.mode == "N" then
    gearbox:setMode("neutral")
  else
    gearbox:setMode("drive")
    local gearIndex = gearbox[dct.primaryAccess.gearIndexName]
    if automaticHandling.mode == "R" and gearbox[dct.access1.gearIndexName] > -1 then
      gearIndex = -1
      dct.primaryAccess = dct.access1
      dct.secondaryAccess = dct.access2
    elseif automaticHandling.mode ~= "R" and gearbox[dct.access1.gearIndexName] < 1 then
      gearIndex = 1
      dct.primaryAccess = dct.access1
      dct.secondaryAccess = dct.access2
    end

    if gearbox[dct.primaryAccess.gearIndexName] ~= gearIndex then
      newDesiredGearIndex = gearIndex
      previousGearIndex = gearbox[dct.primaryAccess.gearIndexName]
      M.timer.shiftDelayTimer = 0
      M.updateGearboxGFX = gearboxLogic.whileShifting
    end
  end

  M.isSportModeActive = automaticHandling.mode == "S"
  M.isManualModeActive = string.sub(automaticHandling.mode, 1, 1) == "M"
end

local function setDefaultForwardMode(mode)
  --todo directly set the active mode as well if we are in forward
  automaticHandling.defaultForwardMode = mode
  if automaticHandling.mode == "D" or automaticHandling.mode == "S" or automaticHandling.mode == "1" or automaticHandling.mode == "2" or string.find(automaticHandling.mode, "M") then
    automaticHandling.mode = mode
    if mode == "M1" then --we just shifted into M1
      --instead of actually using M1, we want to KEEP the current gear, so M<current gear>
      automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
    end
    applyGearboxMode()
    applyGearboxModeRestrictions()
  end
end

local function shiftUp()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  local previousMode = automaticHandling.mode
  automaticHandling.modeIndex = min(automaticHandling.modeIndex + 1, #automaticHandling.modes)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  if automaticHandling.mode == "M1" then --we just shifted into M1
    --instead of actually using M1, we want to KEEP the current gear, so M<current gear>
    automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
  end

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = 0
    if string.find(automaticHandling.mode, "M") then
      local gearIndex = tonumber(string.sub(automaticHandling.mode, 2))
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if tonumber(automaticHandling.mode) then
      local gearIndex = tonumber(automaticHandling.mode)
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      automaticHandling.mode = previousMode
    end
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

local function shiftDown()
  if automaticHandling.mode == "N" then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  local previousMode = automaticHandling.mode
  automaticHandling.modeIndex = max(automaticHandling.modeIndex - 1, 1)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]

  if previousMode == "M1" and electrics.values.wheelspeed > 2 and M.gearboxHandling.gearboxSafety then
    --we just tried to downshift past M1, something that is irritating while racing, so we disallow this shift unless we are really slow
    automaticHandling.mode = previousMode
  end

  if M.gearboxHandling.gearboxSafety then
    local gearRatio = 0
    if string.find(automaticHandling.mode, "M") then
      local gearIndex = tonumber(string.sub(automaticHandling.mode, 2))
      gearRatio = gearbox.gearRatios[gearIndex]
    end

    if tonumber(automaticHandling.mode) then
      local gearIndex = tonumber(automaticHandling.mode)
      gearRatio = gearbox.gearRatios[gearIndex]
    end
    if gearbox.outputAV1 * gearRatio > engine.maxAV then
      automaticHandling.mode = previousMode
    end
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

local function shiftToGearIndex(index)
  local desiredMode = automaticHandling.hShifterModeLookup[index]
  if not desiredMode or not automaticHandling.existingModeLookup[desiredMode] then
    if desiredMode and not automaticHandling.existingModeLookup[desiredMode] then
      guihooks.message({txt = "vehicle.vehicleController.cannotShiftAuto", context = {mode = desiredMode}}, 2, "vehicle.shiftLogic.cannotShift")
    end
    desiredMode = "N"
  end
  automaticHandling.mode = desiredMode

  if automaticHandling.mode == "M1" then --we just shifted into M1
    --instead of actually using M1, we want to KEEP the current gear, so M<current gear>
    automaticHandling.mode = "M" .. tostring(max(gearbox.gearIndex, 1))
  end

  applyGearboxMode()
  applyGearboxModeRestrictions()
end

local function dctPredictNextGear()
  local nextGear = gearbox[dct.secondaryAccess.gearIndexName]
  if M.throttle > 0 and gearbox[dct.primaryAccess.gearIndexName] > 0 and M.smoothedValues.brake <= 0 and (engine.outputAV1 / M.shiftBehavior.shiftUpAV) > 0.9 then
    nextGear = gearbox[dct.primaryAccess.gearIndexName] + sign(gearbox[dct.primaryAccess.gearIndexName])
  elseif gearbox[dct.primaryAccess.gearIndexName] > 0 then
    nextGear = gearbox[dct.primaryAccess.gearIndexName] - sign(gearbox[dct.primaryAccess.gearIndexName])
  end
  --make sure to limit the max gear to whatever is currently on the current secondary shaft (ie max - 1 for the globaly second shaft)
  local maxGearIndexForCurrentSecondary = dct.secondaryAccess == dct.access1 and gearbox.maxGearIndex or gearbox.maxGearIndex - 1
  if gearbox[dct.secondaryAccess.gearIndexName] ~= nextGear and gearbox[dct.secondaryAccess.gearIndexName] < maxGearIndexForCurrentSecondary and M.timer.shiftDelayTimer <= 0 then
    gearbox[dct.secondaryAccess.setGearIndexName](gearbox, nextGear)
    M.timer.shiftDelayTimer = M.timerConstants.shiftDelay
  end
end

local function updateExposedData()
  M.rpm = engine and (engine.outputAV1 * constants.avToRPM) or 0
  M.smoothedAvgAVInput = sharedFunctions.updateAvgAVSingleDevice("gearbox")
  M.waterTemp = (engine and engine.thermals) and (engine.thermals.coolantTemperature or engine.thermals.oilTemperature) or 0
  M.oilTemp = (engine and engine.thermals) and engine.thermals.oilTemperature or 0
  M.checkEngine = engine and engine.isDisabled or false
  M.ignition = electrics.values.ignitionLevel > 1
  M.engineThrottle = (engine and engine.isDisabled) and 0 or M.throttle
  M.engineLoad = engine and (engine.isDisabled and 0 or engine.instantEngineLoad) or 0
  M.running = engine and not engine.isDisabled or false
  M.engineTorque = engine and engine.combustionTorque or 0
  M.flywheelTorque = engine and engine.outputTorque1 or 0
  M.gearboxTorque = gearbox and gearbox.outputTorque1 or 0
  M.isEngineRunning = (engine and engine.starterMaxAV and engine.starterEngagedCoef) and ((engine.outputAV1 > engine.starterMaxAV * 0.8 and engine.starterEngagedCoef <= 0) and 1 or 0) or 1
  -- PHEV: the hybrid system is "running" whenever it is ready, also with motor and engine at standstill
  if engine and engine.systemReady ~= nil then
    M.isEngineRunning = engine.systemReady and 1 or 0
  end
  M.minGearIndex = gearbox.minGearIndex
  M.maxGearIndex = gearbox.maxGearIndex
end

-- PHEV: synchronised clutch handover, like a real DCT, instead of the stock fixed-time clutch ramp.
-- Upshift:   torque phase (the incoming clutch takes over the drive torque, the outgoing one releases), then
--            inertia phase (the incoming clutch slips and holds the wheel torque while the motor brings the
--            input shaft down to the new gear's speed); the clutch locks only once the rpm matches.
-- Downshift: inertia phase first (driving: the outgoing clutch slips and keeps driving; coasting or
--            regenerating: the incoming clutch slips and keeps braking) while the motor speeds the shaft up,
--            then the clutch locks (after a short torque phase when driving).
-- How long the speed change takes follows from the shaft inertia and the motor torque that is free: with the
-- engine decoupled only the light motor shaft changes speed; with K0 closed the engine's inertia and torque come
-- on top, so engine-on shifts take longer and the incoming clutch helps (felt as a short surge).
local phevShift = {phase = "idle", timer = 0, phaseTimer = 0, hold = 0, onGear = 0, offGear = 0}

-- Electric shift points (engine decoupled, launchMode "electric"): upshift once the motor would still turn
-- phevEUpFloorRPM after it, downshift below phevEDownRPM; but no upshift while the pull asked for (the pedal's share
-- of the most wheel torque any gear gives at this speed, as the hybrid controller maps it) is more than
-- phevEPullShare of what the next gear gives, and a downshift where the pedal asks for more than this gear gives and
-- a lower one gives more. Otherwise the aggression-based points apply (they only matter at high demand).
local function phevElectricShiftPoints(gearIndex, outAV)
  if engine.launchMode ~= "electric" or gearIndex < 1 or not engine.motorDriveTorqueAt then return end
  local ratios = gearbox.gearRatios
  local function pull(g)
    local r = ratios[g]
    if not r then return 0 end
    local av = abs(outAV) * r
    if av >= engine.maxAV * 0.98 then return 0 end
    return engine.motorDriveTorqueAt(engine, av) * r
  end
  local best = 0
  for g = 1, gearbox.maxGearIndex do best = max(best, pull(g)) end
  local wanted = max(M.throttle, M.smoothedValues.throttle or 0) * best
  local current = ratios[gearIndex]
  local nextRatio = ratios[gearIndex + 1]
  local upAV = math.huge
  if nextRatio then
    upAV = dct.phevEUpFloorAV * current / nextRatio
    if wanted > dct.phevEPullShare * pull(gearIndex + 1) then upAV = M.shiftBehavior.shiftUpAV end
  end
  local downAV = dct.phevEDownAV
  local lower = ratios[gearIndex - 1]
  if gearIndex > 1 and lower and wanted > pull(gearIndex) and pull(gearIndex - 1) > pull(gearIndex) * 1.05 then
    downAV = max(downAV, M.shiftBehavior.shiftDownAV)
  end
  M.shiftBehavior.shiftUpAV = min(upAV, M.shiftBehavior.shiftUpAV)
  M.shiftBehavior.shiftDownAV = min(downAV, upAV * 0.8)
end

-- PHEV: load-based shift points with the engine coupled in D, like the DCT's shift map. BeamNG's driving aggression
-- (1.333 x pedal, rising fast and falling slowly, so it follows the pedal's peaks) raised the points so much that a
-- steady motorway pedal of about 0.45 with small corrections sat at 6->5 (about 2,500 rpm) and at 50-80 km/h the
-- box stayed in 4th/5th at about 1,700 rpm. Here the points between the light-load (low) and full-load (high) tables
-- follow the pedal itself, low-passed (phevLoadRiseTime up, phevLoadFallTime down), as
-- coef = ((pedal - phevLoadMapStart) / (phevLoadMapFull - phevLoadMapStart))^2, so a part-load cruise keeps the light-load
-- points; kickdown (phevKickdownPedal) gives the full-load points at once. A downshift that only the raised point asks
-- for (shaft above the light-load point) waits phevLoadDownDelay with the demand held, so short pedal corrections don't
-- drop a gear; coasting and lugging downshifts are immediate. Logs of the real car: 6->5 at a median
-- 2,330 rpm with 100-180 Nm, 2,660 rpm above 180 Nm; 5->6 at about 2,050 rpm with light load, 2,900 rpm at medium.
-- S, M and electric operation keep BeamNG's aggression (and the Electric points above).
local phevLoad = {pedal = 0, coef = 0, kickdown = false, downTimer = 0, active = false, lowDownAV = 0, lifted = true, demandDown = false,
  aggression = 0, aggInit = false, lastPedal = 0, fastPedal = 0, gx = 0, gy = 0, downCooldown = 0, outPerMs = 11.2}

-- Driving-style index (like a DCT's sport factor): a slow state 0..1 that rises with sustained pedal, pedal rate,
-- longitudinal and lateral acceleration and kickdown (phevAggRiseTime) and decays slowly (phevAggFallTime) towards the
-- mode's baseline (phevAggBaseHybrid, phevAggBaseBoost). It raises the upshift points (phevAggUpScale of the way to the
-- full-load points), makes kickdown choose a lower gear (phevKickdownForceShare + phevKickdownAggShare x index) and arms
-- the Boost firm-pedal downshift only above phevFirmAggression. A single pedal stab hardly moves it, so calm cruising
-- stays in the high gears. Before, only the low-passed pedal placed the points (0.2 s up, 1.5 s down).
local function phevUpdateAggression(dt, p)
  local phevMode = electrics.values.phevMode == 2
  local base = phevMode and dct.phevAggBaseBoost or dct.phevAggBaseHybrid
  if not phevLoad.aggInit then
    phevLoad.aggression = base
    phevLoad.aggInit = true
  end
  local rate = abs(p - phevLoad.lastPedal) / max(dt, 1e-3)
  phevLoad.lastPedal = p
  phevLoad.fastPedal = phevLoad.fastPedal + (p - phevLoad.fastPedal) * min(1, dt / 0.5)
  local k = min(1, dt / 0.5)
  phevLoad.gx = phevLoad.gx + ((sensors and sensors.gx or 0) - phevLoad.gx) * k
  phevLoad.gy = phevLoad.gy + ((sensors and sensors.gy or 0) - phevLoad.gy) * k
  local s = max(clamp((phevLoad.fastPedal - 0.3) / 0.6, 0, 1), clamp(rate / dct.phevAggPedalRate, 0, 1) * 0.6,
    clamp(abs(phevLoad.gx) / dct.phevAggLongAccel, 0, 1), clamp(abs(phevLoad.gy) / dct.phevAggLatAccel, 0, 1),
    phevLoad.kickdown and 0.8 or 0, base)
  local a = phevLoad.aggression
  if s > a then
    a = a + (s - a) * min(1, dt / max(dct.phevAggRiseTime, 0.1))
  else
    a = a + (max(s, base) - a) * min(1, dt / max(dct.phevAggFallTime, 0.1))
  end
  phevLoad.aggression = clamp(a, 0, 1)
  electrics.values.phevAggression = phevLoad.aggression
end

-- Kickdown target gear: the highest gear that gives at least the force share (phevKickdownForceShare, more with a sporty
-- index) of the most wheel torque any gear gives at this road speed, with the system curve (400 Nm / 160 kW) and at most
-- phevKickdownMaxRPM after the shift. Above about 3,800 rpm the system is power-limited, so every gear there gives about
-- the same force and the highest of them wins. Before, the full-load points calculated from the system curve dropped up
-- to four gears (6-2 at 50-72 km/h, 5-2 at 80 km/h, 6-3 at 93-120 km/h in a logged manual-mode drive).
local function phevKickdownGear(outAV)
  local ratios = gearbox.gearRatios
  local mod = engine
  if not (mod.systemTorqueLimit and ratios) then return nil end
  local function force(g)
    local r = ratios[g]
    if not r or r <= 0 then return 0 end
    local av = abs(outAV) * r
    if av > min(dct.phevKickdownMaxAV, engine.maxAV * 0.98) then return 0 end
    local t = mod.systemTorqueLimit
    if av > 1 and mod.systemPowerLimit then t = min(t, mod.systemPowerLimit / av) end
    return t * r
  end
  local best = 0
  for g = 1, gearbox.maxGearIndex do best = max(best, force(g)) end
  if best <= 0 then return nil end
  local share = clamp(dct.phevKickdownForceShare + dct.phevKickdownAggShare * phevLoad.aggression, 0, 1)
  for g = gearbox.maxGearIndex, 1, -1 do
    if force(g) >= share * best then return g end
  end
  return 1
end

local function phevUpdateLoad(dt)
  local p = clamp(M.throttle or 0, 0, 1)
  local tau = p > phevLoad.pedal and dct.phevLoadRiseTime or dct.phevLoadFallTime
  phevLoad.pedal = phevLoad.pedal + (p - phevLoad.pedal) * min(1, dt / max(tau, 1e-3))
  -- kickdown as the hybrid strategy decided it (keyboard: held floored for a while); own threshold as a fallback
  local kd = electrics.values.phevKickdown
  if kd ~= nil then
    phevLoad.kickdown = kd == 1
  else
    phevLoad.kickdown = p >= dct.phevKickdownPedal
  end
  if phevLoad.kickdown then
    phevLoad.pedal = p
  end
  -- per drive mode (electrics phevMode: 1 Hybrid, 2 Boost): where the pedal starts to raise the points
  local mapStart = electrics.values.phevMode == 2 and dct.phevLoadMapStartBoost or dct.phevLoadMapStart
  local x = clamp((phevLoad.pedal - mapStart) / max(dct.phevLoadMapFull - mapStart, 0.01), 0, 1)
  phevLoad.coef = phevLoad.kickdown and 1 or x * x
  -- S (Boost mode): the same pedal map as D (phevSportLoadFloor, 0 by default). BeamNG's aggression in S kept the
  -- coasting downshift points at 2,000-2,500 rpm, so every lift-off ran down the gears (5-4 at 84, 4-3 at 82 km/h),
  -- each with a large speed match; a floor of 0.12 still held the car in 5th at 70-90 km/h (6th 12 % against 92 %
  -- on the real car over the same stretch). The real car shifts like D at part load and coasts in 6th from 195 to about 110 km/h.
  local sport = automaticHandling.mode == "S"
  if sport then
    phevLoad.coef = max(phevLoad.coef, dct.phevSportLoadFloor)
  end
  phevLoad.lifted = p < dct.phevLiftedPedal
  phevLoad.active = engine.launchMode ~= "electric" and (automaticHandling.mode == "D" or sport)
  phevUpdateAggression(dt, p)
  -- recent pedal low for the firm-pedal press: follows a falling pedal quickly and a held or rising one within about
  -- phevFirmPressTime s; every frame (the downshift gate runs only between shifts). The driver's own pedal
  -- (throttle_input): M.throttle dips during each shift, and its recovery looked like a press. A kickdown serves the
  -- press it came from: after one, a pedal held at 0.90 counted as a fresh press for 2-3 s (the low still rising from
  -- the 0.16 cruise pedal) and the firm rule went 5-4 once the index passed 0.5
  local pedalNow = clamp(electrics.values.throttle_input or p, 0, 1)
  local low = phevLoad.pedalLow or pedalNow
  phevLoad.pedalNow = pedalNow
  -- (a falling pedal within about 0.15 s, so a single-frame dip is no lift: the test rig's step changes passed one
  -- frame of 0, and 0 -> 0.90 counted as a press)
  local tau = pedalNow < low and 0.15 or dct.phevFirmPressTime
  low = low + (pedalNow - low) * min(1, dt / tau)
  phevLoad.pedalLow = phevLoad.kickdown and max(low, pedalNow) or low
  phevLoad.downCooldown = max(phevLoad.downCooldown - dt, 0)
  -- gearbox output speed per m/s of road speed, learnt while the wheels roll freely (for shift decisions with locked or
  -- spinning wheels, phevBodyOutAV)
  local ws = abs(electrics.values.wheelspeed or 0)
  local air = abs(electrics.values.airspeed or ws)
  if phevLoad.lastAir then
    phevLoad.decel = (phevLoad.decel or 0) + ((phevLoad.lastAir - air) / max(dt, 1e-3) - (phevLoad.decel or 0)) * min(1, dt / 0.2)
  end
  phevLoad.lastAir = air
  if ws > 3 and abs(air - ws) < 0.03 * ws and abs(gearbox.outputAV1 or 0) > 1 then
    phevLoad.outPerMs = phevLoad.outPerMs + (abs(gearbox.outputAV1) / ws - phevLoad.outPerMs) * min(1, dt / 2)
  end
end

-- the gearbox output speed the road speed (body) gives: with the front wheels locked under braking the stock logic saw
-- the wheels' speed and ran down the gears to 1st (a logged manual-mode drive: 6-5-4-3-1 within 0.9 s at 52 -> 23 km/h body
-- speed with the wheels at 34 -> 5 km/h)
local function phevBodyOutAV()
  return abs(electrics.values.airspeed or 0) * phevLoad.outPerMs
end

-- Over-rev protection of the launch / in-gear clutch logic: open the clutch only when the wheels would drive the shaft
-- past the limit (a wrong downshift), not when the shaft flares from its own drive torque (wheelspin, a launch). With
-- the stock check (shaft above 1.05 x the 6,000 rpm maxRPM) a flare to 6,300 rpm opened the clutch, the free shaft then
-- ran to the limiter (6,860 rpm) and the box could neither drive nor upshift until the speed fell back: 2 s in 2nd at
-- 80 km/h at full pedal, and 2-3 s at standstill after a brake + throttle start (logged drives).
local function phevOverrev(engineAV, gearIndex)
  local limit = engine.maxAV * 1.05
  if engineAV <= limit then return false end
  local r = gearbox.gearRatios[gearIndex] or 0
  return abs((gearbox.outputAV1 or 0) * r) > limit
end

local function phevLoadShiftPoints(gearIndex, outAV)
  local sv = M.smoothedValues
  local aggression = sv.drivingAggression
  -- the light-load and full-load points of this gear, from the vehicle controller's tables
  sv.drivingAggression = 0.2
  sharedFunctions.selectShiftPoints(gearIndex)
  local lowUp, lowDown = M.shiftBehavior.shiftUpAV, M.shiftBehavior.shiftDownAV
  sv.drivingAggression = 1
  sharedFunctions.selectShiftPoints(gearIndex)
  local highUp, highDown = M.shiftBehavior.shiftUpAV, M.shiftBehavior.shiftDownAV
  -- full-load upshift at most phevFullLoadUpRPM: BeamNG calculates the full-load points from the system curve
  -- (calculateOptimalLoadShiftPoints) and ignores highShiftUpRPM, which put them at 5,850-5,890 rpm (phev_full_accel)
  highUp = min(highUp, dct.phevFullLoadUpAV)
  sv.drivingAggression = aggression
  -- per drive mode: an offset on the light-load points (Hybrid / Boost parameter sets; Electric has its own points, see
  -- phevElectricShiftPoints). Logs of the real car: Boost light-load upshifts (below 60 Nm) at a median 2,180 rpm against about
  -- 2,050 in Hybrid (49 Boost upshifts only); coasting downshifts are rare in Boost and not shifted
  local phevMode = electrics.values.phevMode == 2
  local upOffset = phevMode and dct.phevUpOffsetBoost or dct.phevUpOffsetHybrid
  local downOffset = phevMode and dct.phevDownOffsetBoost or dct.phevDownOffsetHybrid
  lowUp = min(lowUp + upOffset, highUp)
  lowDown = min(max(lowDown + downOffset, 0), lowUp * 0.8)
  local c = phevLoad.coef
  -- the driving-style index raises the upshift points (not the downshift points: those stay demand-based)
  local cUp = max(c, phevLoad.aggression * dct.phevAggUpScale)
  M.shiftBehavior.shiftUpAV = lowUp + (highUp - lowUp) * cUp
  M.shiftBehavior.shiftDownAV = lowDown + (highDown - lowDown) * c
  phevLoad.lowDownAV = lowDown
  if phevLoad.kickdown and outAV then
    -- kickdown: down to the target gear (phevKickdownGear), not to BeamNG's full-load points
    local target = phevKickdownGear(max(abs(outAV), phevBodyOutAV()))
    if target then
      M.shiftBehavior.shiftDownAV = gearIndex > target and math.huge or lowDown
    end
  end
end

local function phevSelectShiftPoints(gearIndex, outAV)
  if phevLoad.active and gearIndex >= 1 then
    phevLoadShiftPoints(gearIndex, outAV)
  else
    sharedFunctions.selectShiftPoints(gearIndex)
    phevLoad.lowDownAV = math.huge
  end
  phevElectricShiftPoints(gearIndex, outAV)
end

-- With the engine coupled (Hybrid, Boost) a load downshift is demand-based: engine and motor share the input shaft, so the
-- box only needs a lower gear when the torque the strategy asks for in this gear exceeds what engine (steady full load)
-- and motor (its curve, within the battery's discharge power) can give together, times phevDemandMargin. The request is
-- the strategy's own: pedal x system torque in Boost, pedal x the engine's full load in Hybrid, at the current shaft
-- speed. Kickdown and the coasting / lugging points (below the light-load point) are unchanged. Before, the pedal load map
-- lowered the downshift points, so every pedal above about 0.6 dropped a gear; the real car held 6th about 90 % of the
-- time at 110-130 km/h with 200+ Nm from the engine for minutes.
local function phevDemandExceeds(engineAV)
  local mod = engine
  local ice = mod.parent
  local av = abs(engineAV)
  if not (ice and mod.motorDriveTorqueAt and mod.systemTorqueLimit) or av < 1 then
    return false
  end
  local cap = mod.systemTorqueLimit
  if mod.systemPowerLimit then
    cap = min(cap, mod.systemPowerLimit / av)
  end
  local iceFull = 0
  local data = ice.torqueData
  if data and data.curves and data.curves[data.finalCurveName] then
    iceFull = max(data.curves[data.finalCurveName].torque[math.floor(av * constants.avToRPM)] or 0, 0)
  end
  local motor = mod.motorDriveTorqueAt(mod, av) * (mod.batteryDriveCoef or 1)
  local dischargeLimit = mod.dischargeLimitNow or mod.batteryMaxDischargePower
  if dischargeLimit then
    motor = min(motor, dischargeLimit * (mod.motorEfficiency or 0.9) / av)
  end
  local available = min(cap, iceFull + motor)
  local scale = (electrics.values.phevMode == 2) and cap or iceFull
  return clamp(M.throttle or 0, 0, 1) * scale > dct.phevDemandMargin * available
end

local function phevDownshiftAllowed(dt, engineAV)
  phevLoad.demandDown = false
  local pedalNow = phevLoad.pedalNow or 0
  electrics.values.phevDownWhy = 0
  if not phevLoad.active or phevLoad.kickdown or engineAV < phevLoad.lowDownAV or engineAV >= M.shiftBehavior.shiftDownAV then
    phevLoad.downTimer = 0
    -- why a downshift may happen (test rig column downWhy): 1 engine decoupled, 2 kickdown, 3 below the light-load point
    electrics.values.phevDownWhy = not phevLoad.active and 1 or phevLoad.kickdown and 2 or engineAV < phevLoad.lowDownAV and 3 or 0
    return true
  end
  -- the raised point only counts while the driver asks for it: after a lift-off the low-passed pedal keeps the
  -- point up for 1.5-3 s, and the box then downshifted while coasting.
  -- Boost mode: a firm pedal (phevFirmPedal, 0.8) asks for the raised point whatever the torque reserve. The real car
  -- drops 1-2 gears at about 80 % pedal at motorway speed; the demand test alone held 6th up to kickdown (0.95).
  -- (only above phevFirmMinKmh and with a driving-style index of at least phevFirmAggression: a logged manual-mode
  -- drive, mostly at 0.8-1.0 pedal, dropped gears constantly)
  -- (and only on a press: the pedal at least phevFirmPress above its recent low. A held pedal let the slowly rising
  -- index arm it mid-acceleration: 0.90 pedal after kickdown went 4-5-4-5-6 at 100-125 km/h)
  local firm = electrics.values.phevMode == 2 and pedalNow >= dct.phevFirmPedal and pedalNow - phevLoad.pedalLow >= dct.phevFirmPress
    and abs(electrics.values.airspeed or 0) * 3.6 >= dct.phevFirmMinKmh and phevLoad.aggression >= dct.phevFirmAggression
  if phevLoad.downCooldown > 0 then
    phevLoad.downTimer = 0
    return false
  end
  if phevLoad.lifted or not (firm or phevDemandExceeds(engineAV)) then
    phevLoad.downTimer = 0
    return false
  end
  phevLoad.downTimer = phevLoad.downTimer + dt
  phevLoad.demandDown = phevLoad.downTimer >= dct.phevLoadDownDelay
  electrics.values.phevDownWhy = phevLoad.demandDown and (firm and 4 or 5) or 0
  if phevLoad.demandDown then
    -- the press is served by this downshift
    phevLoad.pedalLow = pedalNow
    -- one gear per demand, then a pause before the next demand downshift
    phevLoad.downCooldown = dct.phevDemandDownCooldown
  end
  return phevLoad.demandDown
end

local function phevStopSync()
  if engine then
    engine.syncTargetRatio = nil
    engine.syncClutchTorque = 0
  end
  phevShift.phase = "idle"
end

-- the motor synchronises shifts between forward gears (the stock throttle-blip rev match is not used then,
-- also not while the new gear is still being pre-selected: that would open both clutches)
local function phevSyncApplies()
  return engine ~= nil and engine.phevSyncShift == true and gearbox[dct.primaryAccess.gearIndexName] >= 1 and newDesiredGearIndex >= 1
end

local function phevClutchCapacity()
  return max((gearbox.lockTorque or 500) * (gearbox.wearLockTorqueCoef or 1) * (gearbox.damageLockTorqueCoef or 1), 1)
end

-- swaps the clutch roles once the incoming clutch is locked; returns the new primary gear index
local function phevSwapAccess()
  dct.primaryAccess, dct.secondaryAccess = dct.secondaryAccess, dct.primaryAccess
  local primaryGearIndex = gearbox[dct.primaryAccess.gearIndexName]
  -- publish the new ratio at once, so the hybrid's wheel-torque pedal map uses it in this frame
  gearbox.gearRatio = gearbox.gearRatios[primaryGearIndex]
  clutchHandling.didRevMatch = false
  clutchHandling.didCutIgnition = false
  if newDesiredGearIndex == primaryGearIndex then
    M.updateGearboxGFX = gearboxLogic.inGear
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
    M.timer.revMatchTimer = 0
  end
  return primaryGearIndex
end

-- runs one step of the handover; returns true once the incoming clutch is locked
local function phevHandover(dt)
  local onAccess, offAccess = dct.secondaryAccess, dct.primaryAccess
  local onGear, offGear = gearbox[onAccess.gearIndexName], gearbox[offAccess.gearIndexName]
  local onRatio = gearbox.gearRatios[onGear] or 0
  local offRatio = gearbox.gearRatios[offGear] or 0
  if onRatio <= 0 or offRatio <= 0 then
    phevStopSync()
    electrics.values[onAccess.clutchRatioName] = 1
    electrics.values[offAccess.clutchRatioName] = 0
    return true
  end

  local electric = engine.launchMode == "electric"
  local capacity = phevClutchCapacity()
  local shaftAV = engine.outputAV1
  local targetAV = gearbox.outputAV1 * onRatio
  local inertia = engine.shaftInertiaEffective or engine.inertia or 0.1
  local upshift = onRatio < offRatio

  if phevShift.phase == "idle" or phevShift.onGear ~= onGear or phevShift.offGear ~= offGear then
    phevShift.onGear, phevShift.offGear = onGear, offGear
    phevShift.timer = 0
    phevShift.phaseTimer = 0
    -- the torque the box carries into the shift (input-shaft side, negative while regenerating)
    -- (low-passed by the hybrid module: the instantaneous value swung by +-100 Nm in a driveline shuffle)
    phevShift.hold = engine.clutchLoadFiltered or gearbox.torqueDiff or 0
    phevShift.phase = upshift and "torque" or "inertia"
  end
  phevShift.timer = phevShift.timer + dt
  phevShift.phaseTimer = phevShift.phaseTimer + dt

  local aggression = clamp(shiftAggression, 0, 1)
  local torqueTime = electric and dct.phevElectricTorquePhase or (0.12 - 0.08 * aggression)
  local inertiaTime = electric and dct.phevElectricShiftTime or (0.35 - 0.25 * aggression)
  local tolAV = max(25 * constants.rpmToAV, abs(targetAV) * 0.015)
  local hold = phevShift.hold
  local onCmd, offCmd = 0, 0
  local done = false

  if upshift then
    -- the same wheel torque in the new gear
    local driveHold = max(hold, 0) * offRatio / onRatio
    if phevShift.phase == "torque" then
      local p = min(phevShift.phaseTimer / torqueTime, 1)
      onCmd = p * driveHold / capacity
      offCmd = (1 - p) * min(max(hold, 0) * 1.3 / capacity + 0.05, 1)
      if p >= 1 then
        phevShift.phase = "inertia"
        phevShift.phaseTimer = 0
        -- engine coupled: the hybrid module reduces the engine's torque for the speed change in every upshift, like
        -- a DCT (it used to cut the ignition only above 0.5 aggression, and otherwise the motor absorbed the
        -- engine's torque and inertia: -30 to -85 kW pulses)
      end
    else
      local clutchSync = 0
      if not electric then
        -- engine coupled: the incoming clutch also pulls the heavier shaft down
        clutchSync = dct.phevEngineClutchSyncShare * inertia * max(shaftAV - targetAV, 0) / inertiaTime
      end
      local onTorque = driveHold + clutchSync
      -- what the incoming clutch passes on at the new gear's speed stays within the system power (160 kW): at a
      -- full-load upshift the old gear's wheel torque plus the surge gave about 230 kW at the wheels for 0.1-0.15 s
      -- (phev_kickdown 2-3 at 90 km/h), which made the 0.5 s peaks read 166-168 kW system and 127 kW engine. The
      -- engine's torque reduction then gives up the rest of the flywheel energy.
      if engine.systemPowerLimit and targetAV > 1 then
        onTorque = min(onTorque, max(driveHold, engine.systemPowerLimit / targetAV))
      end
      -- and within the dual-clutch gearbox's input rating (400 Nm) x phevShiftClutchCap: in logged drives engine + motor
      -- at the input read 410-450 Nm (90th-99th percentile) in shifts only, never in gear
      if engine.systemTorqueLimit then
        onTorque = min(onTorque, engine.systemTorqueLimit * dct.phevShiftClutchCap)
      end
      onCmd = onTorque / capacity
      offCmd = 0
      engine.syncTargetRatio = onRatio
      engine.syncClutchTorque = onTorque
      engine.syncTau = inertiaTime / 3
      if shaftAV - targetAV < tolAV then
        done = true
      end
    end
  else
    if phevShift.phase == "inertia" then
      local syncClutch = 0
      if hold > 0 then
        -- driving: the outgoing clutch slips and keeps the drive torque while the shaft speeds up
        offCmd = hold / capacity
        syncClutch = hold
      else
        -- coasting or regenerating: the incoming clutch slips and keeps the braking torque
        local brakeHold = -hold * offRatio / onRatio
        onCmd = brakeHold / capacity
        syncClutch = -brakeHold
      end
      engine.syncTargetRatio = onRatio
      engine.syncClutchTorque = syncClutch
      engine.syncTau = inertiaTime / 3
      if targetAV - shaftAV < tolAV then
        if hold > 0 then
          phevShift.phase = "torque"
          phevShift.phaseTimer = 0
        else
          done = true
        end
      end
    else
      -- driving downshift with the shaft in sync: hand the drive torque over to the incoming clutch
      local p = min(phevShift.phaseTimer / torqueTime, 1)
      onCmd = p
      offCmd = (1 - p) * hold / capacity
      engine.syncTargetRatio = onRatio
      engine.syncClutchTorque = hold
      if p >= 1 then
        done = true
      end
    end
  end

  -- safety net: never hang in a shift
  if phevShift.timer > torqueTime + 4 * inertiaTime + 0.5 then
    done = true
  end
  if done then
    onCmd, offCmd = 1, 0
    phevStopSync()
  end
  electrics.values[onAccess.clutchRatioName] = clamp(onCmd, 0, 1)
  electrics.values[offAccess.clutchRatioName] = clamp(offCmd, 0, 1)
  return done
end

local function updateInGearArcade(dt)
  if phevShift.phase ~= "idle" then
    phevStopSync()
  end
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = false

  local gearIndex = gearbox[dct.primaryAccess.gearIndexName]
  local engineAV = engine.outputAV1
  local gearboxBasedEngineAV = gearbox.outputAV1 * gearbox.gearRatios[gearIndex]

  -- driving backwards? - only with automatic shift - for obvious reasons ;)
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.8) or (automaticHandling.mode == "N" and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end
  -- PHEV: the load the shift map follows
  phevUpdateLoad(dt)

  --Arcade mode gets a "rev limiter" in case the engine does not have one
  if engineAV > engine.maxAV and not engine.hasRevLimiter then
    local throttleAdjust = min(max((engineAV - engine.maxAV * 1.02) / (engine.maxAV * 0.03), 0), 1)
    M.throttle = min(max(M.throttle - throttleAdjust, 0), 1)
  end

  if M.timer.gearChangeDelayTimer <= 0 then
    local tmpEngineAV = engineAV
    local relEngineAV = engineAV / gearbox.gearRatios[gearIndex]

    phevSelectShiftPoints(gearIndex, relEngineAV)

    local wheelSlipCanShiftDown = M.shiftPreventionData.wheelSlipShiftDown or M.brake <= 0
    -- PHEV: a load downshift waits for the demand to hold
    local phevCanShiftDown = phevDownshiftAllowed(dt, tmpEngineAV)
    -- downshift decisions on the road speed when the wheels turn slower than the car moves (locked under braking)
    local bodyEngineAV = phevBodyOutAV() * (gearbox.gearRatios[gearIndex] or 0)
    if engine.launchMode ~= "electric" and bodyEngineAV > tmpEngineAV then
      tmpEngineAV = bodyEngineAV
      relEngineAV = tmpEngineAV / gearbox.gearRatios[gearIndex]
    end
    -- braking: decide on the speed the car will have in phevBrakeDownLead s (a DCT downshifts ahead under hard braking;
    -- the box lagged one or two gears behind, 6th at 800 rpm), and don't let the wheel-slip guard block it (the road
    -- speed above doesn't depend on the wheels)
    if M.brake > 0 and (phevLoad.decel or 0) > 1 then
      local air = max(abs(electrics.values.airspeed or 0), 0.5)
      local f = clamp((air - phevLoad.decel * dct.phevBrakeDownLead) / air, 0.3, 1)
      tmpEngineAV = tmpEngineAV * f
      relEngineAV = relEngineAV * f
      wheelSlipCanShiftDown = true
    end
    --shift down?
    while tmpEngineAV < M.shiftBehavior.shiftDownAV and abs(gearIndex) > 1 and wheelSlipCanShiftDown and phevCanShiftDown and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold do
      gearIndex = gearIndex - sign(gearIndex)
      -- a demand downshift goes one gear at a time (the next frame checks the demand again)
      if phevLoad.demandDown then
        phevCanShiftDown = false
      end
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV > engine.maxAV then
        gearIndex = gearIndex + sign(gearIndex)
        break
      end
      phevSelectShiftPoints(gearIndex, relEngineAV)
    end

    local wheelSlipCanShiftUp = M.shiftPreventionData.wheelSlipShiftUp or M.throttle <= 0
    --shift up?
    local isRevLimitReached = engine.revLimiterActive and not (engine.isTempRevLimiterActive or false)
    if (tmpEngineAV >= M.shiftBehavior.shiftUpAV or isRevLimitReached) and M.brake <= 0 and electrics.values[dct.primaryAccess.clutchRatioName] >= 1 and wheelSlipCanShiftUp and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold and gearIndex < gearbox.maxGearIndex and gearIndex > gearbox.minGearIndex and automaticHandling.mode ~= "N" then
      local upFromAV = tmpEngineAV
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV < (engine.launchMode == "electric" and dct.phevEDownAV or engine.idleAV) then
        gearIndex = gearIndex - sign(gearIndex)
      elseif phevLoad.active and not isRevLimitReached and upFromAV < dct.phevFullLoadUpAV and phevDemandExceeds(tmpEngineAV) then
        -- no upshift into a gear whose demand test would at once ask for this one back (0.90 pedal in Boost
        -- after kickdown went 4-5-4-5 at 104-123 km/h); the box holds the gear up to the full-load point instead
        gearIndex = gearIndex - sign(gearIndex)
        tmpEngineAV = upFromAV
      end
      phevSelectShiftPoints(gearIndex, relEngineAV)
    end
  end

  -- neutral gear handling
  local neutralGearChanged = false
  if abs(gearIndex) <= 1 and M.timer.neutralSelectionDelayTimer <= 0 then
    if automaticHandling.mode ~= "P" and abs(M.smoothedValues.avgAV) < M.gearboxHandling.arcadeAutoBrakeAVThreshold and M.throttle <= 0 then
      M.brake = max(M.brake, M.gearboxHandling.arcadeAutoBrakeAmount)
    end

    if M.smoothedValues.throttleInput > 0 and M.smoothedValues.brakeInput <= 0 and M.smoothedValues.avgAV > -1 and automaticHandling.mode ~= automaticHandling.defaultForwardMode then
      gearIndex = 1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = automaticHandling.defaultForwardMode
      neutralGearChanged = true
      applyGearboxMode()
    end

    if M.smoothedValues.brakeInput > 0 and M.smoothedValues.throttleInput <= 0 and M.smoothedValues.avgAV <= 0.15 and automaticHandling.mode ~= "R" then
      gearIndex = -1
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = "R"
      neutralGearChanged = true
      applyGearboxMode()
    end

    if electrics.values.ignitionLevel ~= 2 and automaticHandling.mode ~= "P" then
      gearIndex = 0
      M.timer.neutralSelectionDelayTimer = M.timerConstants.neutralSelectionDelay
      automaticHandling.mode = "P"
      neutralGearChanged = true
      applyGearboxMode()
    end
  end

  if not neutralGearChanged and gearbox[dct.primaryAccess.gearIndexName] ~= gearIndex then
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox[dct.primaryAccess.gearIndexName]
    M.timer.shiftDelayTimer = 0
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  -- PHEV: the clutch of the gear that is still engaged. In the frame a shift is requested gearIndex is already the
  -- new gear: a downshift to 1st applied 1st gear's launch and creep limits to 2nd gear's clutch, which opened to the
  -- creep point (0.02) for the whole pre-selection, so the braking torque was lost and the shift began with almost
  -- no torque to hold (the motor then swung from -316 to +325 Nm at 23 km/h)
  local engagedGear = gearbox[dct.primaryAccess.gearIndexName]
  local clutchGear = (engagedGear ~= gearIndex and engagedGear * gearIndex > 0) and engagedGear or gearIndex
  -- Control clutch to buildup engine RPM
  local dctClutchRatio = 0
  if abs(clutchGear) == 1 and (M.throttle > 0 or max(engineAV, gearboxBasedEngineAV) > clutchHandling.clutchLaunchTargetAV) and not neutralGearChanged then
    local ratio = max((engine.outputAV1 - clutchHandling.clutchLaunchStartAV * (1 + M.throttle)) / (clutchHandling.clutchLaunchTargetAV * (1 + clutchHandling.clutchLaunchIFactor)), 0)
    clutchHandling.clutchLaunchIFactor = min(clutchHandling.clutchLaunchIFactor + dt * 0.5, 1)
    dctClutchRatio = min(max(ratio * ratio, 0), 1)
  elseif abs(clutchGear) > 1 then
    if M.smoothedValues.avgAV * gearbox.gearRatios[clutchGear] * engine.outputAV1 >= 0 then
      dctClutchRatio = 1
    elseif abs(clutchGear) >= 1 and abs(M.smoothedValues.avgAV) > 1 then
      M.brake = M.throttle
      M.throttle = 0
    end
    clutchHandling.clutchLaunchIFactor = 0
  end

  -- PHEV: electric drive-away, the motor can pull from 0 rpm, so close the clutch instead of slipping
  local phevElectric = engine.launchMode == "electric"
  if phevElectric and abs(clutchGear) >= 1 and not neutralGearChanged then
    dctClutchRatio = min((clutchHandling.phevElectricClutchRatio or 0) + dt * (clutchHandling.phevElectricCloseRate or 6), 1)
  end

  if (not phevElectric and engine.outputAV1 < engine.idleAV) or phevOverrev(engine.outputAV1, clutchGear) or automaticHandling.mode == "P" or automaticHandling.mode == "N" then
    --always prevent stalling and overrevving
    dctClutchRatio = 0
  end
  -- PHEV: engine-led launch without throttle. While the engine is being started (K0 open) only the motor holds
  -- the shaft at idle, so the clutch stays open. With the engine running the clutch only creeps: at the kiss
  -- point with the brake held (a DCT's standstill decoupling), a moderate creep without brake.
  if not phevElectric and abs(clutchGear) == 1 and M.throttle <= 0 then
    if engine.k0State ~= "running" then
      dctClutchRatio = 0
    else
      dctClutchRatio = min(dctClutchRatio, M.brake > 0.05 and dct.phevBrakeCreepRatio or dct.phevCreepRatio)
    end
  end
  -- PHEV: engine-led launch with throttle. With the launch constants (clutchLaunchStartRPM 1200, TargetRPM 1500)
  -- the stock formula closes the clutch to about 60 % already at engine idle, so touching the pedal with the
  -- engine running at standstill or crawling (warm-up, charging at standstill, kickdown start) jolted the car and
  -- pulled the shaft down. While the gearbox side is below idle (the clutch has to slip), the clutch starts at the
  -- creep point at idle and closes further only as the shaft revs above idle: the shaft settles where the clutch
  -- carries the drive torque, a few hundred rpm above idle at full load, like a DCT launch.
  if not phevElectric and abs(clutchGear) == 1 and M.throttle > 0 and abs(gearboxBasedEngineAV) < engine.idleAV then
    local aboveIdle = max(engineAV - engine.idleAV, 0)
    local launchCap = dct.phevCreepRatio + (1 - dct.phevCreepRatio) * min(aboveIdle / dct.phevLaunchSlipAV, 1)
    dctClutchRatio = min(dctClutchRatio, launchCap)
  end
  clutchHandling.phevElectricClutchRatio = phevElectric and dctClutchRatio or 0

  --prevent the clutch from engaging while the rpm is still high from just starting the engine
  if engine.idleAVStartOffset > 1 and M.throttle <= 0 then
    dctClutchRatio = 0
  end

  local wasHoldingClutchOpen = clutchHandling.isClutchOpenLaunch
  clutchHandling.isClutchOpenLaunch = false
  if (M.throttle > 0.5 and M.brake > 0.5 and electrics.values.wheelspeed < 2) or gearbox.lockCoef < 1 then
    dctClutchRatio = 0
    clutchHandling.isClutchOpenLaunch = true
  end

  --delay the next shift after having launched via throttle + brake, otherwise it might short-shift
  if wasHoldingClutchOpen and clutchHandling.isClutchOpenLaunch then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  electrics.values[dct.primaryAccess.clutchRatioName] = dctClutchRatio
  electrics.values[dct.secondaryAccess.clutchRatioName] = 0
  M.clutchRatio = 1
  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
  gearbox.gearIndex = gearbox[dct.primaryAccess.gearIndexName] --just so that the DCT can always present the "active" gear/ratio to the outside world
  gearbox.gearRatio = gearbox.gearRatios[gearbox[dct.primaryAccess.gearIndexName]]
  updateExposedData()

  dctPredictNextGear()
end

local function updateWhileShiftingArcade(dt)
  --keep throttle input for upshifts and kill it for downshifts so that rev matching can work properly
  --also make sure to only keep throttle while shifting in the same direction, ie not -1 to 1 or so
  M.throttle = (newDesiredGearIndex > gearbox[dct.primaryAccess.gearIndexName] and newDesiredGearIndex * gearbox[dct.primaryAccess.gearIndexName] > 0) and M.inputValues.throttle or 0
  -- PHEV: a synchronised downshift keeps the driver's pedal. The stock logic drops it for its own throttle blip; with K0
  -- closed that left the engine dragging (-14 to -18 kW) while the motor pulled at 81 kW through a 2 s kickdown
  -- downshift (a logged motorway drive)
  if M.throttle == 0 and phevSyncApplies() and newDesiredGearIndex < gearbox[dct.primaryAccess.gearIndexName] then
    M.throttle = M.inputValues.throttle
  end
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = true

  local gearIndex = gearbox.gearIndex
  if (gearIndex < 0 and M.smoothedValues.avgAV <= 0.15) or (gearIndex <= 0 and M.smoothedValues.avgAV < -1) then
    M.throttle, M.brake = M.brake, M.throttle
    M.isArcadeSwitched = true
  end

  M.currentGearIndex = newDesiredGearIndex

  -- secondary clutch closes while primary opens -> in gear update once fully closed
  local primaryGearIndex = gearbox[dct.primaryAccess.gearIndexName]
  local secondaryGearIndex = gearbox[dct.secondaryAccess.gearIndexName]
  if newDesiredGearIndex ~= secondaryGearIndex and M.timer.shiftDelayTimer <= 0 then
    --find out if our desired gear is actually on the secondary shaft
    local sameShaft = newDesiredGearIndex % 2 == secondaryGearIndex % 2
    --if so, we can directly shift to that desired gear on the secondary shaft
    --if not, we need to use a helper gear first which makes the actually desired gear part of the secondary shaft
    local newGearIndex = sameShaft and newDesiredGearIndex or primaryGearIndex + sign(newDesiredGearIndex - primaryGearIndex)
    if secondaryGearIndex ~= newGearIndex then
      gearbox[dct.secondaryAccess.setGearIndexName](gearbox, newGearIndex)
      M.timer.shiftDelayTimer = M.timerConstants.shiftDelay * 0.5
    end
  end

  local canShift = true
  local isEngineRunning = engine.ignitionCoef >= 1 and not engine.isStalled
  local targetGearRatio = gearbox.gearRatios[newDesiredGearIndex]
  local targetAV = targetGearRatio * gearbox.outputAV1
  local isDownShift = abs(newDesiredGearIndex) < abs(primaryGearIndex)
  -- PHEV: with the synchronised handover the motor matches the speed, no throttle blip needed
  local phevSync = phevSyncApplies()
  -- PHEV: R or N selected in the middle of a synchronised handover: the stock branch takes over, so release the
  -- motor's speed target at once (it would keep chasing the old gear's speed with both clutches open)
  if not phevSync and (phevShift.phase ~= "idle" or engine.syncTargetRatio) then
    phevStopSync()
  end
  if not phevSync and isDownShift and targetAV > engine.outputAV1 and not clutchHandling.didRevMatch and isEngineRunning then
    M.throttle = clutchHandling.revMatchThrottle
    electrics.values[dct.primaryAccess.clutchRatioName] = 0
    electrics.values[dct.secondaryAccess.clutchRatioName] = 0
    M.timer.revMatchTimer = M.timer.revMatchTimer + dt
    local revMatchExpired = M.timer.revMatchTimer > M.timerConstants.revMatchExpired
    canShift = (engine.outputAV1 >= (targetAV - (500 * constants.rpmToAV))) or targetAV > engine.maxAV or revMatchExpired
    clutchHandling.didRevMatch = canShift
  elseif not clutchHandling.didRevMatch then
    clutchHandling.didRevMatch = true
  end

  local adjustedClutchTime = clamp((dct.clutchTime - dct.clutchTime * 10) * shiftAggression + dct.clutchTime * 10, dct.clutchTime, dct.clutchTime * 10)

  if phevSync then
    -- wait (in gear, clutch closed) until the new gear is pre-selected on the other shaft
    if M.timer.shiftDelayTimer <= 0 and gearbox[dct.secondaryAccess.gearIndexName] >= 1 and phevHandover(dt) then
      primaryGearIndex = phevSwapAccess()
    end
  elseif M.timer.shiftDelayTimer <= 0 and canShift then
    if gearbox[dct.primaryAccess.gearIndexName] > 0 and gearbox[dct.primaryAccess.gearIndexName] < gearbox[dct.secondaryAccess.gearIndexName] and M.throttle > 0 and adjustedClutchTime < 0.15 and not clutchHandling.didCutIgnition then
      engine:cutIgnition(adjustedClutchTime)
      clutchHandling.didCutIgnition = true
    end
    if isDownShift and M.brake > 0.5 then
      adjustedClutchTime = 1
    end

    -- PHEV: with the engine decoupled only the light motor shaft changes speed, so the handover is fast and
    -- fixed, and the shift can hardly be felt (also while regenerating)
    if engine.launchMode == "electric" then
      adjustedClutchTime = dct.phevElectricShiftTime
    end

    local clutchRatio = min(electrics.values[dct.secondaryAccess.clutchRatioName] + (1 / adjustedClutchTime) * dt, 1)
    -- PHEV: a 0 rpm minimum (electric operation) needs no stall protection and must not divide by zero
    local stallPrevent = engine.idleAV > 1 and min(max((engine.outputAV1 * 0.9 - engine.idleAV) / (engine.idleAV * 0.1), 0), 1) or 1
    electrics.values[dct.primaryAccess.clutchRatioName] = isDownShift and 0 or min(1 - clutchRatio, stallPrevent * stallPrevent)
    electrics.values[dct.secondaryAccess.clutchRatioName] = min(clutchRatio, stallPrevent * stallPrevent)
    if clutchRatio == 1 or stallPrevent < 1 then
      dct.primaryAccess, dct.secondaryAccess = dct.secondaryAccess, dct.primaryAccess
      primaryGearIndex = gearbox[dct.primaryAccess.gearIndexName]

      if newDesiredGearIndex == primaryGearIndex then
        M.updateGearboxGFX = gearboxLogic.inGear
        M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
        M.timer.revMatchTimer = 0
        clutchHandling.didRevMatch = false
        clutchHandling.didCutIgnition = false
      end
    end
  end
  --print(string.format("Clutch1: %.2f, Clutch 2: %.2f",electrics.values.clutchRatio1,electrics.values.clutchRatio2))

  gearbox.gearIndex = primaryGearIndex --just so that the DCT can always present the "active" gear to the outside world
  updateExposedData()
end

local function updateInGear(dt)
  if phevShift.phase ~= "idle" then
    phevStopSync()
  end
  M.throttle = M.inputValues.throttle
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = false
  -- PHEV: the load the shift map follows
  phevUpdateLoad(dt)

  local gearIndex = gearbox[dct.primaryAccess.gearIndexName]

  local engineAV = engine.outputAV1
  local gearboxBasedEngineAV = gearbox.outputAV1 * gearbox.gearRatios[gearIndex]

  if M.timer.gearChangeDelayTimer <= 0 and gearbox[dct.primaryAccess.clutchRatioName] >= 1 then
    local tmpEngineAV = max(engineAV, gearboxBasedEngineAV)
    local relEngineAV = tmpEngineAV / gearbox.gearRatios[gearIndex]

    phevSelectShiftPoints(gearIndex, relEngineAV)

    local wheelSlipCanShiftDown = M.shiftPreventionData.wheelSlipShiftDown or M.brake <= 0
    -- PHEV: a load downshift waits for the demand to hold
    local phevCanShiftDown = phevDownshiftAllowed(dt, tmpEngineAV)
    -- downshift decisions on the road speed when the wheels turn slower than the car moves (locked under braking)
    local bodyEngineAV = phevBodyOutAV() * (gearbox.gearRatios[gearIndex] or 0)
    if engine.launchMode ~= "electric" and bodyEngineAV > tmpEngineAV then
      tmpEngineAV = bodyEngineAV
      relEngineAV = tmpEngineAV / gearbox.gearRatios[gearIndex]
    end
    -- braking: decide on the speed the car will have in phevBrakeDownLead s (a DCT downshifts ahead under hard braking;
    -- the box lagged one or two gears behind, 6th at 800 rpm), and don't let the wheel-slip guard block it (the road
    -- speed above doesn't depend on the wheels)
    if M.brake > 0 and (phevLoad.decel or 0) > 1 then
      local air = max(abs(electrics.values.airspeed or 0), 0.5)
      local f = clamp((air - phevLoad.decel * dct.phevBrakeDownLead) / air, 0.3, 1)
      tmpEngineAV = tmpEngineAV * f
      relEngineAV = relEngineAV * f
      wheelSlipCanShiftDown = true
    end
    while tmpEngineAV < M.shiftBehavior.shiftDownAV and abs(gearIndex) > 1 and wheelSlipCanShiftDown and phevCanShiftDown and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold do
      gearIndex = gearIndex - sign(gearIndex)
      -- a demand downshift goes one gear at a time (the next frame checks the demand again)
      if phevLoad.demandDown then
        phevCanShiftDown = false
      end
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV > engine.maxAV then
        gearIndex = gearIndex + sign(gearIndex)
        break
      end
      phevSelectShiftPoints(gearIndex, relEngineAV)
    end

    local wheelSlipCanShiftUp = M.shiftPreventionData.wheelSlipShiftUp or M.throttle <= 0
    --shift up?
    local isRevLimitReached = engine.revLimiterActive and not (engine.isTempRevLimiterActive or false)
    if (tmpEngineAV >= M.shiftBehavior.shiftUpAV or isRevLimitReached) and M.brake <= 0 and wheelSlipCanShiftUp and abs(M.throttle - M.smoothedValues.throttle) < M.smoothedValues.throttleUpShiftThreshold and gearIndex < gearbox.maxGearIndex and gearIndex > gearbox.minGearIndex and automaticHandling.mode ~= "N" then
      local upFromAV = tmpEngineAV
      gearIndex = gearIndex + sign(gearIndex)
      tmpEngineAV = relEngineAV * (gearbox.gearRatios[gearIndex] or 0)
      if tmpEngineAV < (engine.launchMode == "electric" and dct.phevEDownAV or engine.idleAV) then
        gearIndex = gearIndex - sign(gearIndex)
      elseif phevLoad.active and not isRevLimitReached and upFromAV < dct.phevFullLoadUpAV and phevDemandExceeds(tmpEngineAV) then
        -- no upshift into a gear whose demand test would at once ask for this one back (0.90 pedal in Boost
        -- after kickdown went 4-5-4-5 at 104-123 km/h); the box holds the gear up to the full-load point instead
        gearIndex = gearIndex - sign(gearIndex)
        tmpEngineAV = upFromAV
      end
      phevSelectShiftPoints(gearIndex, relEngineAV)
    end
  end

  local isManualMode = string.sub(automaticHandling.mode, 1, 1) == "M"
  --enforce things like L and M modes
  gearIndex = min(max(gearIndex, automaticHandling.minGearIndex), automaticHandling.maxGearIndex)
  if isManualMode and gearIndex > 1 and engineAV < engine.idleAV * 1.2 and M.shiftPreventionData.wheelSlipShiftDown and automaticHandling.autoDownShiftInM then
    gearIndex = gearIndex - 1
  end

  if gearbox[dct.primaryAccess.gearIndexName] ~= gearIndex then
    newDesiredGearIndex = gearIndex
    previousGearIndex = gearbox[dct.primaryAccess.gearIndexName]
    M.timer.shiftDelayTimer = 0
    calculateShiftAggression()
    M.updateGearboxGFX = gearboxLogic.whileShifting
  end

  -- PHEV: the clutch of the gear that is still engaged. In the frame a shift is requested gearIndex is already the
  -- new gear: a downshift to 1st applied 1st gear's launch and creep limits to 2nd gear's clutch, which opened to the
  -- creep point (0.02) for the whole pre-selection, so the braking torque was lost and the shift began with almost
  -- no torque to hold (the motor then swung from -316 to +325 Nm at 23 km/h)
  local engagedGear = gearbox[dct.primaryAccess.gearIndexName]
  local clutchGear = (engagedGear ~= gearIndex and engagedGear * gearIndex > 0) and engagedGear or gearIndex
  -- Control clutch to buildup engine RPM
  local dctClutchRatio = 0
  if abs(clutchGear) == 1 and (M.throttle > 0 or max(engineAV, gearboxBasedEngineAV) > clutchHandling.clutchLaunchTargetAV) then
    local ratio = max((max(engineAV, gearboxBasedEngineAV) - clutchHandling.clutchLaunchStartAV * (1 + M.throttle)) / (clutchHandling.clutchLaunchTargetAV * (1 + clutchHandling.clutchLaunchIFactor)), 0)
    clutchHandling.clutchLaunchIFactor = min(clutchHandling.clutchLaunchIFactor + dt * 0.5, 1)
    dctClutchRatio = min(max(ratio * ratio, 0), 1)
  elseif abs(clutchGear) > 1 then
    if gearbox.outputAV1 * gearbox.gearRatios[clutchGear] * engineAV >= 0 then
      dctClutchRatio = 1
    end
    clutchHandling.clutchLaunchIFactor = 0
  end

  -- PHEV: electric drive-away, the motor can pull from 0 rpm, so close the clutch instead of slipping
  local phevElectric = engine.launchMode == "electric"
  if phevElectric and abs(clutchGear) >= 1 then
    dctClutchRatio = min((clutchHandling.phevElectricClutchRatio or 0) + dt * (clutchHandling.phevElectricCloseRate or 6), 1)
  end

  if (not phevElectric and engineAV < engine.idleAV * 0.5) or phevOverrev(engineAV, clutchGear) or automaticHandling.mode == "P" or automaticHandling.mode == "N" then
    --always prevent stalling
    dctClutchRatio = 0
  end
  -- PHEV: engine-led launch without throttle. While the engine is being started (K0 open) only the motor holds
  -- the shaft at idle, so the clutch stays open. With the engine running the clutch only creeps: at the kiss
  -- point with the brake held (a DCT's standstill decoupling), a moderate creep without brake.
  if not phevElectric and abs(clutchGear) == 1 and M.throttle <= 0 then
    if engine.k0State ~= "running" then
      dctClutchRatio = 0
    else
      dctClutchRatio = min(dctClutchRatio, M.brake > 0.05 and dct.phevBrakeCreepRatio or dct.phevCreepRatio)
    end
  end
  -- PHEV: engine-led launch with throttle. With the launch constants (clutchLaunchStartRPM 1200, TargetRPM 1500)
  -- the stock formula closes the clutch to about 60 % already at engine idle, so touching the pedal with the
  -- engine running at standstill or crawling (warm-up, charging at standstill, kickdown start) jolted the car and
  -- pulled the shaft down. While the gearbox side is below idle (the clutch has to slip), the clutch starts at the
  -- creep point at idle and closes further only as the shaft revs above idle: the shaft settles where the clutch
  -- carries the drive torque, a few hundred rpm above idle at full load, like a DCT launch.
  if not phevElectric and abs(clutchGear) == 1 and M.throttle > 0 and abs(gearboxBasedEngineAV) < engine.idleAV then
    local aboveIdle = max(engineAV - engine.idleAV, 0)
    local launchCap = dct.phevCreepRatio + (1 - dct.phevCreepRatio) * min(aboveIdle / dct.phevLaunchSlipAV, 1)
    dctClutchRatio = min(dctClutchRatio, launchCap)
  end
  clutchHandling.phevElectricClutchRatio = phevElectric and dctClutchRatio or 0

  if engine.idleAVStartOffset > 1 and M.throttle <= 0 then
    dctClutchRatio = 0
  end

  local wasHoldingClutchOpen = clutchHandling.isClutchOpenLaunch
  clutchHandling.isClutchOpenLaunch = false
  if (M.throttle > 0.5 and M.brake > 0.5 and electrics.values.wheelspeed < 2) or gearbox.lockCoef < 1 then
    dctClutchRatio = 0
    clutchHandling.isClutchOpenLaunch = true
  end

  --delay the next shift after having launched via throttle + brake, otherwise it might short-shift
  if wasHoldingClutchOpen and clutchHandling.isClutchOpenLaunch then
    M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
  end

  electrics.values[dct.primaryAccess.clutchRatioName] = dctClutchRatio
  electrics.values[dct.secondaryAccess.clutchRatioName] = 0
  M.clutchRatio = 1
  M.currentGearIndex = (automaticHandling.mode == "N" or automaticHandling.mode == "P") and 0 or gearIndex
  gearbox.gearIndex = gearbox[dct.primaryAccess.gearIndexName] --just so that the DCT can always present the "active" gear/ratio to the outside world
  gearbox.gearRatio = gearbox.gearRatios[gearbox[dct.primaryAccess.gearIndexName]]

  if isManualMode then
    automaticHandling.mode = "M" .. gearIndex
    automaticHandling.modeIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
    applyGearboxModeRestrictions()
  end

  updateExposedData()
  dctPredictNextGear()
end

local function updateWhileShifting(dt)
  --keep throttle input for upshifts and kill it for downshifts so that rev matching can work properly
  --also make sure to only keep throttle while shifting in the same direction, ie not -1 to 1 or so
  M.throttle = (newDesiredGearIndex > gearbox[dct.primaryAccess.gearIndexName] and newDesiredGearIndex * gearbox[dct.primaryAccess.gearIndexName] > 0) and M.inputValues.throttle or 0
  -- PHEV: a synchronised downshift keeps the driver's pedal. The stock logic drops it for its own throttle blip; with K0
  -- closed that left the engine dragging (-14 to -18 kW) while the motor pulled at 81 kW through a 2 s kickdown
  -- downshift (a logged motorway drive)
  if M.throttle == 0 and phevSyncApplies() and newDesiredGearIndex < gearbox[dct.primaryAccess.gearIndexName] then
    M.throttle = M.inputValues.throttle
  end
  M.brake = M.inputValues.brake
  M.isArcadeSwitched = false
  M.isShifting = true

  M.currentGearIndex = newDesiredGearIndex

  -- secondary clutch closes while primary opens -> in gear update once fully closed
  local primaryGearIndex = gearbox[dct.primaryAccess.gearIndexName]
  local secondaryGearIndex = gearbox[dct.secondaryAccess.gearIndexName]

  if newDesiredGearIndex ~= secondaryGearIndex and M.timer.shiftDelayTimer <= 0 then
    --find out if our desired gear is actually on the secondary shaft
    local sameShaft = newDesiredGearIndex % 2 == secondaryGearIndex % 2
    --if so, we can directly shift to that desired gear on the secondary shaft
    --if not, we need to use a helper gear first which makes the actually desired gear part of the secondary shaft
    local newGearIndex = sameShaft and newDesiredGearIndex or primaryGearIndex + sign(newDesiredGearIndex - primaryGearIndex)
    if secondaryGearIndex ~= newGearIndex then
      gearbox[dct.secondaryAccess.setGearIndexName](gearbox, newGearIndex)
      M.timer.shiftDelayTimer = M.timerConstants.shiftDelay * 0.5
    end
  end

  local canShift = true
  local isEngineRunning = engine.ignitionCoef >= 1 and not engine.isStalled
  local targetGearRatio = gearbox.gearRatios[newDesiredGearIndex]
  local targetAV = targetGearRatio * gearbox.outputAV1
  local isDownShift = abs(newDesiredGearIndex) < abs(primaryGearIndex)
  -- PHEV: with the synchronised handover the motor matches the speed, no throttle blip needed
  local phevSync = phevSyncApplies()
  -- PHEV: R or N selected in the middle of a synchronised handover: the stock branch takes over, so release the
  -- motor's speed target at once (it would keep chasing the old gear's speed with both clutches open)
  if not phevSync and (phevShift.phase ~= "idle" or engine.syncTargetRatio) then
    phevStopSync()
  end
  if not phevSync and isDownShift and targetAV > engine.outputAV1 and not clutchHandling.didRevMatch and isEngineRunning then
    M.throttle = clutchHandling.revMatchThrottle
    electrics.values[dct.primaryAccess.clutchRatioName] = 0
    electrics.values[dct.secondaryAccess.clutchRatioName] = 0
    M.timer.revMatchTimer = M.timer.revMatchTimer + dt
    local revMatchExpired = M.timer.revMatchTimer > M.timerConstants.revMatchExpired
    canShift = (engine.outputAV1 >= (targetAV - (500 * constants.rpmToAV))) or targetAV > engine.maxAV or revMatchExpired
    if canShift then
      M.throttle = 0
    end
    clutchHandling.didRevMatch = canShift
  elseif not clutchHandling.didRevMatch then
    clutchHandling.didRevMatch = true
  end

  local adjustedClutchTime = clamp((dct.clutchTime - dct.clutchTime * 10) * shiftAggression + dct.clutchTime * 10, dct.clutchTime, dct.clutchTime * 10)
  if phevSync then
    -- wait (in gear, clutch closed) until the new gear is pre-selected on the other shaft
    if M.timer.shiftDelayTimer <= 0 and gearbox[dct.secondaryAccess.gearIndexName] >= 1 and phevHandover(dt) then
      primaryGearIndex = phevSwapAccess()
    end
  elseif M.timer.shiftDelayTimer <= 0 and canShift then
    if gearbox[dct.primaryAccess.gearIndexName] > 0 and gearbox[dct.primaryAccess.gearIndexName] < gearbox[dct.secondaryAccess.gearIndexName] and M.throttle > 0 and adjustedClutchTime < 0.15 and not clutchHandling.didCutIgnition then
      engine:cutIgnition(adjustedClutchTime)
      clutchHandling.didCutIgnition = true
    end
    if isDownShift and M.brake > 0.5 then
      adjustedClutchTime = 0.5
    end

    -- PHEV: with the engine decoupled only the light motor shaft changes speed, so the handover is fast and
    -- fixed, and the shift can hardly be felt (also while regenerating)
    if engine.launchMode == "electric" then
      adjustedClutchTime = dct.phevElectricShiftTime
    end
    local clutchRatio = min(electrics.values[dct.secondaryAccess.clutchRatioName] + (1 / adjustedClutchTime) * dt, 1)
    -- PHEV: a 0 rpm minimum (electric operation) needs no stall protection and must not divide by zero
    local stallPrevent = engine.idleAV > 1 and min(max((engine.outputAV1 * 0.9 - engine.idleAV) / (engine.idleAV * 0.1), 0), 1) or 1
    electrics.values[dct.primaryAccess.clutchRatioName] = isDownShift and 0 or min(1 - clutchRatio, stallPrevent * stallPrevent)
    electrics.values[dct.secondaryAccess.clutchRatioName] = min(clutchRatio, stallPrevent * stallPrevent)
    if clutchRatio == 1 or stallPrevent < 1 then
      dct.primaryAccess, dct.secondaryAccess = dct.secondaryAccess, dct.primaryAccess
      primaryGearIndex = gearbox[dct.primaryAccess.gearIndexName]
      clutchHandling.didRevMatch = false
      clutchHandling.didCutIgnition = false

      if newDesiredGearIndex == primaryGearIndex then
        M.updateGearboxGFX = gearboxLogic.inGear
        M.timer.gearChangeDelayTimer = M.timerConstants.gearChangeDelay
        M.timer.revMatchTimer = 0
      end
    end
  end

  --If we are currently in the wrong sign of gear (ie trying to drive backwards while technically a forward gear is still selected),
  --do not ever close the primary clutch to prevent the car from driving in the wrong direction
  if newDesiredGearIndex * gearbox[dct.primaryAccess.gearIndexName] <= 0 then
    electrics.values[dct.primaryAccess.clutchRatioName] = 0
  end

  gearbox.gearIndex = newDesiredGearIndex --just so that the DCT can always present the "active" gear to the outside world
  updateExposedData()
end

local function sendTorqueData()
  if engine then
    engine:sendTorqueData()
  end
end

local function init(jbeamData, sharedFunctionTable)
  sharedFunctions = sharedFunctionTable
  engine = powertrain.getDevice("mainEngine")
  gearbox = powertrain.getDevice("gearbox")
  newDesiredGearIndex = 0
  previousGearIndex = 0

  M.currentGearIndex = 0
  M.maxGearIndex = 0
  M.minGearIndex = 0
  M.throttle = 0
  M.brake = 0
  M.clutchRatio = 0
  M.shiftingAggression = 0
  M.isArcadeSwitched = false
  M.isSportModeActive = false
  M.isManualModeActive = false

  M.smoothedAvgAVInput = 0
  M.rpm = 0
  M.idleRPM = 0
  M.maxRPM = 0

  M.engineThrottle = 0
  M.engineLoad = 0
  M.engineTorque = 0
  M.flywheelTorque = 0
  M.gearboxTorque = 0

  M.ignition = true
  M.isEngineRunning = 0

  M.oilTemp = 0
  M.waterTemp = 0
  M.checkEngine = false

  gearboxAvailableLogic = {
    arcade = {
      inGear = updateInGearArcade,
      whileShifting = updateWhileShiftingArcade,
      shiftUp = sharedFunctions.warnCannotShiftSequential,
      shiftDown = sharedFunctions.warnCannotShiftSequential,
      shiftToGearIndex = sharedFunctions.switchToRealisticBehavior
    },
    realistic = {
      inGear = updateInGear,
      whileShifting = updateWhileShifting,
      shiftUp = shiftUp,
      shiftDown = shiftDown,
      shiftToGearIndex = shiftToGearIndex
    }
  }

  clutchHandling.didRevMatch = false
  clutchHandling.didCutIgnition = false
  clutchHandling.isClutchOpenLaunch = false
  clutchHandling.clutchLaunchTargetAV = (jbeamData.clutchLaunchTargetRPM or 3000) * constants.rpmToAV * 0.5
  -- PHEV: the engine's idle (the hybrid reports 0 while electric, which it is at init)
  clutchHandling.clutchLaunchStartAV = ((jbeamData.clutchLaunchStartRPM or 2000) * constants.rpmToAV - (engine.iceIdleAV or engine.idleAV)) * 0.5
  clutchHandling.clutchLaunchIFactor = 0
  clutchHandling.revMatchThrottle = jbeamData.revMatchThrottle or 0.5

  automaticHandling.availableModeLookup = {}
  for _, v in pairs(automaticHandling.availableModes) do
    automaticHandling.availableModeLookup[v] = true
  end

  automaticHandling.modes = {}
  automaticHandling.modeIndexLookup = {}
  local modes = jbeamData.automaticModes or "PRNDS21M"
  local modeCount = #modes
  local modeOffset = 0
  for i = 1, modeCount do
    local mode = modes:sub(i, i)
    if automaticHandling.availableModeLookup[mode] then
      if mode ~= "M" then
        automaticHandling.modes[i + modeOffset] = mode
        automaticHandling.modeIndexLookup[mode] = i + modeOffset
        automaticHandling.existingModeLookup[mode] = true
      else
        for j = 1, gearbox.maxGearIndex, 1 do
          local manualMode = "M" .. tostring(j)
          local manualModeIndex = i + j - 1
          automaticHandling.modes[manualModeIndex] = manualMode
          automaticHandling.modeIndexLookup[manualMode] = manualModeIndex
          automaticHandling.existingModeLookup[manualMode] = true
          modeOffset = j - 1
        end
      end
    else
      print("unknown auto mode: " .. mode)
    end
  end

  local defaultMode = jbeamData.defaultAutomaticMode or "N"
  automaticHandling.modeIndex = string.find(modes, defaultMode)
  automaticHandling.mode = automaticHandling.modes[automaticHandling.modeIndex]
  automaticHandling.maxGearIndex = gearbox.maxGearIndex
  automaticHandling.minGearIndex = gearbox.minGearIndex
  automaticHandling.autoDownShiftInM = jbeamData.autoDownShiftInM == nil and true or jbeamData.autoDownShiftInM

  dct.clutchTime = jbeamData.dctClutchTime or 0.05
  -- PHEV: clutch handover time while the engine is decoupled
  dct.phevElectricShiftTime = jbeamData.phevElectricShiftTime or 0.04
  dct.phevElectricTorquePhase = jbeamData.phevElectricTorquePhase or 0.02
  dct.phevEUpFloorAV = (jbeamData.phevEUpFloorRPM or 1000) * constants.rpmToAV
  dct.phevEDownAV = (jbeamData.phevEDownRPM or 800) * constants.rpmToAV
  dct.phevEPullShare = jbeamData.phevEPullShare or 0.9
  -- creep clutch with the engine running and no throttle: brake held / released (share of clutch capacity)
  dct.phevBrakeCreepRatio = jbeamData.phevBrakeCreepRatio or 0.02
  dct.phevCreepRatio = jbeamData.phevCreepRatio or 0.1
  -- engine-led launch: shaft speed above idle at which the clutch may be fully closed while it slips
  dct.phevLaunchSlipAV = max(jbeamData.phevLaunchSlipRPM or 500, 50) * constants.rpmToAV
  -- share of the speed change the incoming clutch provides with the engine coupled (the rest is the motor)
  dct.phevEngineClutchSyncShare = jbeamData.phevEngineClutchSyncShare or 0.5
  -- load-based shift points with the engine coupled in D (see phevLoadShiftPoints)
  dct.phevLoadMapStart = jbeamData.phevLoadMapStart or 0.3
  dct.phevLoadMapStartBoost = jbeamData.phevLoadMapStartBoost or dct.phevLoadMapStart
  dct.phevLoadMapFull = jbeamData.phevLoadMapFull or 0.95
  -- per-mode offsets on the light-load points (rpm): Hybrid (D) and Boost (S)
  dct.phevUpOffsetHybrid = (jbeamData.phevUpOffsetHybridRPM or 0) * constants.rpmToAV
  dct.phevDownOffsetHybrid = (jbeamData.phevDownOffsetHybridRPM or 0) * constants.rpmToAV
  dct.phevUpOffsetBoost = (jbeamData.phevUpOffsetBoostRPM or 0) * constants.rpmToAV
  dct.phevDownOffsetBoost = (jbeamData.phevDownOffsetBoostRPM or 0) * constants.rpmToAV
  dct.phevLoadRiseTime = jbeamData.phevLoadRiseTime or 0.2
  dct.phevLoadFallTime = jbeamData.phevLoadFallTime or 1.5
  dct.phevLoadDownDelay = jbeamData.phevLoadDownDelay or 0.4
  dct.phevKickdownPedal = jbeamData.phevKickdownPedal or 0.95
  -- S: lowest place on the load map (0 = light-load points, 1 = full-load points); the pedal below which a load
  -- downshift doesn't happen (lifted)
  dct.phevSportLoadFloor = jbeamData.phevSportLoadFloor or 0
  dct.phevLiftedPedal = jbeamData.phevLiftedPedal or 0.05
  -- demand-based load downshifts with the engine coupled: margin on engine + motor in the current gear
  dct.phevDemandMargin = jbeamData.phevDemandMargin or 0.9
  dct.phevFirmPedal = jbeamData.phevFirmPedal or 0.8
  dct.phevFirmMinKmh = jbeamData.phevFirmMinKmh or 80
  dct.phevFirmAggression = jbeamData.phevFirmAggression or 0.5
  dct.phevFirmPress = jbeamData.phevFirmPress or 0.08
  dct.phevFirmPressTime = jbeamData.phevFirmPressTime or 1.5
  dct.phevDemandDownCooldown = jbeamData.phevDemandDownCooldown or 3
  -- driving-style index (phevUpdateAggression)
  dct.phevAggBaseHybrid = jbeamData.phevAggBaseHybrid or 0
  dct.phevAggBaseBoost = jbeamData.phevAggBaseBoost or 0.15
  dct.phevAggRiseTime = jbeamData.phevAggRiseTime or 6
  dct.phevAggFallTime = jbeamData.phevAggFallTime or 45
  dct.phevAggPedalRate = jbeamData.phevAggPedalRate or 3
  dct.phevAggLongAccel = jbeamData.phevAggLongAccel or 4
  dct.phevAggLatAccel = jbeamData.phevAggLatAccel or 6
  dct.phevAggUpScale = jbeamData.phevAggUpScale or 0.5
  -- kickdown target gear (phevKickdownGear)
  dct.phevKickdownForceShare = jbeamData.phevKickdownForceShare or 0.65
  dct.phevKickdownAggShare = jbeamData.phevKickdownAggShare or 0.2
  dct.phevKickdownMaxAV = (jbeamData.phevKickdownMaxRPM or 5600) * constants.rpmToAV
  -- braking downshifts: look-ahead (s); clutch torque in an upshift as a share of the system torque (400 Nm)
  dct.phevBrakeDownLead = jbeamData.phevBrakeDownLead or 0.5
  dct.phevFullLoadUpAV = (jbeamData.phevFullLoadUpRPM or 5200) * constants.rpmToAV
  dct.phevShiftClutchCap = jbeamData.phevShiftClutchCap or 1.1
  phevLoad.pedal, phevLoad.coef, phevLoad.kickdown, phevLoad.downTimer, phevLoad.active = 0, 0, false, 0, false
  phevLoad.aggInit, phevLoad.downCooldown, phevLoad.lastPedal, phevLoad.fastPedal, phevLoad.gx, phevLoad.gy = false, 0, 0, 0, 0, 0
  phevStopSync()

  M.maxRPM = engine.maxRPM
  M.idleRPM = engine.idleRPM
  M.maxGearIndex = automaticHandling.maxGearIndex
  M.minGearIndex = abs(automaticHandling.minGearIndex)
  -- PHEV: the fuel gauge shows the 1.4 turbo's tank (the hybrid module itself only links the battery)
  local fuelEngine = (engine.parent and engine.parent.type == "combustionEngine") and engine.parent or engine
  M.energyStorages = sharedFunctions.getEnergyStorages({fuelEngine})

  dct.primaryAccess = dct.access1
  dct.secondaryAccess = dct.access2

  local defaultForwardMode = jbeamData.defaultForwardMode or "D"
  setDefaultForwardMode(defaultForwardMode)
  applyGearboxMode()
end

local function getState()
  local data = {grb_mde = automaticHandling.mode}

  return tableIsEmpty(data) and nil or data
end

local function setState(data)
  if data.grb_mde then
    automaticHandling.mode = data.grb_mde
    automaticHandling.modeIndex = automaticHandling.modeIndexLookup[automaticHandling.mode]
    applyGearboxMode()
    applyGearboxModeRestrictions()
  end
end

M.init = init

M.gearboxBehaviorChanged = gearboxBehaviorChanged
M.shiftUp = shiftUp
M.shiftDown = shiftDown
M.shiftToGearIndex = shiftToGearIndex
M.updateGearboxGFX = nop
M.getGearName = getGearName
M.getGearPosition = getGearPosition
M.setDefaultForwardMode = setDefaultForwardMode
M.sendTorqueData = sendTorqueData

M.areShiftLEDsInUse = areShiftLEDsInUse

M.getState = getState
M.setState = setState

return M
