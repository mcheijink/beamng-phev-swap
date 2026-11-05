-- phev.lua
-- Operating strategy ("hybrid ECU") for the Plug-in Hybrid Swap mod.
--
-- Decides per frame:
--   * the driving mode (EV, Hybrid, Boost) and the Charge mode function
--   * whether the 1.4 turbo should run (the hybrid module performs the start/stop sequence)
--   * how the requested torque is split between engine and motor (motor demand and engine throttle factor)
--
-- Mode behaviour follows the design requirements; thresholds are calibrated on the
-- drive logs of the real car. In Hybrid the engine covers the driver's demand while it can do
-- so without (much) turbo boost; above that the motor fills in, so the engine stays in its efficient band.
--
-- Original code written for this add-on. License: MIT, (c) 2026 mcheijink.

local M = {}
M.type = "auxiliary"
-- runs after the vehicle controller (order 500), which sets the brake and throttle values each frame:
-- the brake blending below replaces part of the friction-brake demand by regeneration
M.defaultOrder = 600

local min = math.min
local max = math.max
local abs = math.abs
local floor = math.floor
local smoothmin = smoothmin

local avToRPM = 9.549296585513721

local MODE_ELECTRIC, MODE_HYBRID, MODE_BOOST = 0, 1, 2
local modeNames = {[MODE_ELECTRIC] = "Electric", [MODE_HYBRID] = "Hybrid", [MODE_BOOST] = "Boost"}
local chargeName = "Charge mode"

local module = nil
local ice = nil
local gearbox = nil
local cfg = {}

local mode = MODE_HYBRID
local keepModeAfterReset = false   -- set by reset(): the next READY keeps the mode instead of starting in Hybrid
local holdTarget = 0
local chargeFunction = false
local iceWanted = false
local iceOnTimer = 0
local iceOffTimer = 0
local lowDemandTimer = 0
local coastTimer = 0
local standstillTimer = 0
local warmingUp = false
local standstillChargeWanted = false
local warmedUp = false
local engineShare = 1
local fillSmoother = nil
local lastReady = false
local kickdownActive = false
-- hydrolock protection (water.update): water.block is true while the 1.4's intake is under water, then no engine start
local water = {block = false, timer = 0}
local kbdFloorTimer = 0
local brakeDevices = {}
-- brake devices of the axle the motor regenerates on: only the driven wheels (regenAxle "driven", the default), or all wheels
-- (regenAxle "all"): series blending then lowers the friction brake of that axle only
local regenBrakeDevices = {}
local drivenBrakeDevices = {}
-- share of the driven axle's friction brake that is left (regenAxle "driven", the default), applied in updateWheelsIntermediate
local axleBrakeScale = 1
local finalDriveRatio = 3.75
local coastWheelTorque = 0
local vehicleMass = 1800
local wheelRadius = 0.334
local kickdownEnabled = true
local speedLimitSmoother = nil
-- top-speed limiter coefficient of this frame (1 below the band): also applied to kickdown (updateGFX)
local speedLimitCoef = 1
-- charging at a standstill (Charge mode, Hybrid charge sustaining): fade-in 0..1
local standstillChargeCoef = 0
-- raised idle the strategy asks the module for this frame (standstill charging, brake-held launch, warm-up coasting):
-- the engine's idle controller holds idleReq.av with at most idleReq.throttle (module chargeIdleAV)
local idleReq = {av = 0, throttle = 0}
-- warm-up coasting: the engine keeps firing and the motor puts its power into the battery (fade 0..1)
local warmCoastCoef = 0
local warmCoastTouched = false
local frameDt = 1 / 60
-- B position (stronger lift-off regeneration), brake lights from regeneration and the emergency stop signal
local bMode = false
local lights = {regenOn = false, ess = false, essTimer = 0, hazard = false, lastAir = nil, decel = 0}
-- automatic headlights: the game's time of day (0..1, asked from the game engine), poll timer, whether the low beam was
-- switched on by this controller (and to which state), and the driver's override
local autoLights = {tod = nil, timer = 0, set = false, setState = 0, override = false, lastReady = false}
-- driven (front) wheels, watched for slip under regeneration
local drivenWheels = {}
-- brake torque vectoring (updateWheelsIntermediate): the inner and outer front wheel and the rear wheels by side, the brake applied (Nm)
local tv = {frontL = nil, frontR = nil, rearL = nil, rearR = nil, brakeL = 0, brakeR = 0, active = 0}
local regenSlipHold = 0
local regenSlipCoef = 1
-- pedal filter and lift-off state (with hysteresis), see updatePedalState
local pedalFiltered = 0
local coasting = true
-- Boost / Hybrid charging from the engine: faded in while the driver keeps a steady demand, out on lift-off or slowing
local chargeCoef = 0
local accelSmoothed = 0
local lastSpeedMs = nil
-- engine drag torque seen through a K0 that is opening or closing (low-passed, Nm)
local k0DragFiltered = 0

local function clampValue(x, lo, hi)
  if x < lo then
    return lo
  elseif x > hi then
    return hi
  end
  return x
end

local function message(text)
  guihooks.message({txt = text}, 3, "vehicle.phev.mode")
end

local function usableSoc()
  return module and module.usableSoc or 0
end

-- Hydrolock protection: BeamNG's combustion engine floods (floodLevel rises with crank speed, 0.9 locks it up for good)
-- while ALL of its waterDamage nodes are under water. The crank only turns under water when this controller starts the
-- engine (Electric with the engine off does not flood it: tested), so no engine request is made while the nodes are
-- submerged, and for waterBlockHold s after the last wave, so spray at the edge of a ford does not flicker the request.
-- Switched off by the jbeam option hydrolockProtection = false.
function water.update(dt)
  local submerged = cfg.hydrolockProtection and ice and ice.canFlood and type(ice.waterDamageNodes) == "table" or false
  if submerged then
    for _, n in ipairs(ice.waterDamageNodes) do
      if not obj:inWater(n) then
        submerged = false
        break
      end
    end
  end
  water.timer = submerged and cfg.waterBlockHold or max(water.timer - dt, 0)
  local blocked = water.timer > 0
  if blocked ~= water.block then
    water.block = blocked
    message(blocked and "PHEV: engine intake under water, engine start blocked" or "PHEV: engine start released")
  end
end

-- BeamNG drive-mode selector (controller "driveModes", a part of the host car): its modes are Electric / Hybrid /
-- Boost (keys electric, hybrid, boost). The selector hands its choice over through setParameters; changes the car
-- makes itself (kickdown, empty battery, mode keys, monitor app) are reported back so both always agree.
local DRIVE_MODE_KEYS = {[MODE_ELECTRIC] = "electric", [MODE_HYBRID] = "hybrid", [MODE_BOOST] = "boost"}
local MODE_BY_PARAMETER = {Electric = MODE_ELECTRIC, Hybrid = MODE_HYBRID, Boost = MODE_BOOST}
local syncingDriveModes = false

local function syncDriveModes()
  if syncingDriveModes then
    return
  end
  local dm = controller.getController("driveModes")
  local key = DRIVE_MODE_KEYS[mode]
  if dm and dm.getCurrentDriveModeKey and dm.getDriveModeData and dm.getDriveModeData(key) and dm.getCurrentDriveModeKey() ~= key then
    syncingDriveModes = true
    dm.setDriveMode(key, true)
    syncingDriveModes = false
  end
end

local function setMode(newMode, quiet)
  if not module then
    return
  end
  if newMode == MODE_ELECTRIC and module.socValid and usableSoc() <= cfg.evMinUsableSoc and not water.block then
    message("PHEV: battery empty, " .. modeNames[MODE_ELECTRIC] .. " not available")
    newMode = MODE_HYBRID
  end
  if newMode == MODE_HYBRID and newMode ~= mode then
    -- Hybrid uses the battery to assist and save fuel: it runs it down to the reserve (holdFloor) and only
    -- keeps it there; it doesn't hold the charge it had when selected (as on the real car)
    holdTarget = cfg.holdFloor
  elseif newMode == MODE_BOOST then
    -- Boost keeps the battery at least half full (half of the usable window, not 50 % raw charge):
    -- above that the motor uses the battery freely, below it the engine charges
    holdTarget = cfg.boostHoldTarget
  end
  if newMode == MODE_BOOST and mode ~= MODE_BOOST then
    -- selecting Boost starts the engine at once; a cold engine then runs until it is warm
    iceWanted = true
    iceOnTimer = 0
    iceOffTimer = 0
  end
  mode = newMode
  electrics.values.phevMode = mode
  syncDriveModes()
  if not quiet then
    message("PHEV: " .. modeNames[mode] .. (mode == MODE_HYBRID and chargeFunction and " (" .. chargeName .. ")" or ""))
  end
end

-- Electric button: toggles between Electric and Hybrid
local function toggleElectricMode()
  setMode(mode == MODE_ELECTRIC and MODE_HYBRID or MODE_ELECTRIC)
end

-- Boost button: toggles between Boost and Hybrid
local function toggleBoostMode()
  setMode(mode == MODE_BOOST and MODE_HYBRID or MODE_BOOST)
end

local function cycleMode()
  setMode((mode + 1) % 3)
end

-- Charge mode function (part of Hybrid): the engine charges the battery up to cfg.chargeTarget
local function toggleChargeMode()
  chargeFunction = not chargeFunction
  if chargeFunction then
    setMode(MODE_HYBRID, true)
    message(string.format("PHEV: %s on (to %d%%)", chargeName, floor(cfg.chargeTarget * 100 + 0.5)))
  else
    holdTarget = mode == MODE_BOOST and cfg.boostHoldTarget or cfg.holdFloor
    message("PHEV: " .. chargeName .. " off")
  end
end

local function getMode()
  return mode
end

-- called by the drive-mode selector with {phevMode = "Electric" | "Hybrid" | "Boost"}; the selector shows its own message
local function setParameters(parameters)
  local wanted = parameters and MODE_BY_PARAMETER[parameters.phevMode]
  if wanted and not syncingDriveModes and module then
    syncingDriveModes = true
    setMode(wanted, true)
    syncingDriveModes = false
    -- a refused change (Electric with an empty battery) moves the selector back
    if mode ~= wanted then
      syncDriveModes()
    end
  end
end

-- B position (DCT selector pulled back from D): stronger lift-off regeneration, coastDecelB
local function toggleBMode()
  bMode = not bMode
  electrics.values.phevBMode = bMode and 1 or 0
  message(bMode and "PHEV: B (stronger regeneration)" or "PHEV: D")
end

local function toggleKickdown()
  kickdownEnabled = not kickdownEnabled
  message(kickdownEnabled and "PHEV: kickdown detection on" or "PHEV: kickdown detection off")
end

------------------------------------------------------------------------------------------------

-- full-load torque of the 1.4 turbo engine (incl. turbocharger) at the current shaft speed, as BeamNG computes it
local function iceFullTorqueAt(av)
  local data = ice and ice.torqueData
  local rpm = floor(abs(av) * avToRPM)
  if data and data.curves and data.curves[data.finalCurveName] then
    local t = data.curves[data.finalCurveName].torque[rpm]
    if t then
      return max(t, 0)
    end
  end
  return ice and ice.torqueCurve and (ice.torqueCurve[rpm] or 0) or 0
end

-- Net torque of the 1.4 turbo for a throttle, as BeamNG's combustion engine computes it (combustionEngine.lua
-- updateTorque): the base curve with the live turbo boost and BeamNG's throttle map, less friction and the engine
-- brake. The throttle map blends the pedal with a constant-power map, so part throttle gives far more than its
-- share of the curve (30 % throttle at 2,300 rpm: about 120 Nm, not 75).
local function iceNetTorqueFor(throttle, av)
  local base = (ice.torqueCurve[floor(av * avToRPM)] or 0) * (ice.intakeAirDensityCoef or 1)
  local full = base * (ice.forcedInductionCoef or 1)
  local map = throttle + throttle * (ice.maxPowerThrottleMap or 0) / (full * av + 1e-30) * (1 - throttle)
  map = smoothmin(max(map, 0), 1, max((1 - throttle) * 0.8, 1e-6))
  local gross = full * map
  local friction = (ice.friction or 0) * (ice.wearFrictionCoef or 1) * (ice.damageFrictionCoef or 1)
    + (ice.dynamicFriction or 0) * (ice.wearDynamicFrictionCoef or 1) * (ice.damageDynamicFrictionCoef or 1) * av
  local load = clampValue(gross / max((base - friction) * (ice.forcedInductionCoef or 1), 1e-3), 0, 1)
  return gross - friction - (ice.engineBrakeTorque or 0) * (1 - load)
end

-- the throttle at which the engine delivers this net torque (the inverse of iceNetTorqueFor, by bisection:
-- the torque rises with the throttle); nil when the engine's data isn't there
local function iceThrottleFor(torque, av)
  if not (ice and ice.torqueCurve) or av < 1 then
    return nil
  end
  if torque >= iceNetTorqueFor(1, av) then
    return 1
  end
  local lo, hi = 0, 1
  for _ = 1, 12 do
    local mid = (lo + hi) * 0.5
    if iceNetTorqueFor(mid, av) < torque then
      lo = mid
    else
      hi = mid
    end
  end
  return (lo + hi) * 0.5
end

local function systemTorqueAt(av)
  local t = module.systemTorqueLimit
  if av > 1 then
    t = min(t, module.systemPowerLimit / av)
  end
  return t
end

-- 1 below the Electric speed limit, fading to 0 at the limit
local function evSpeedCoef(speedKmh)
  return clampValue((cfg.evMaxSpeedKmh - speedKmh) / cfg.evSpeedTaperKmh, 0, 1)
end

local function gearEngaged()
  return (electrics.values.gearIndex or 0) ~= 0
end

-- total friction-brake torque the wheels (all brake devices, or the given list) would apply for a brake input, computed
-- like wheels.lua does
local function frictionBrakeTorqueFor(brakeInput, devices)
  local sum = 0
  for _, wd in ipairs(devices or brakeDevices) do
    if not wd.isBroken then
      local split = wd.brakeInputSplit or 1
      local splitCoef = wd.brakeSplitCoef or 1
      sum = sum + (wd.brakeTorque or 0) * (min(brakeInput, split) + max(brakeInput - split, 0) * splitCoef)
    end
  end
  return sum
end

-- motor (shaft) torque to wheel torque in the current gear
local function wheelTorqueRatio()
  local gearRatio = gearbox and abs(gearbox.gearRatio or 0) or 0
  return gearRatio * finalDriveRatio * cfg.drivelineEfficiency
end

local function flagOn(v)
  return v == true or (tonumber(v) or 0) > 0
end

-- Front-wheel slip under regeneration. BeamNG's ABS only evaluates a wheel while its brake input is above 0.1,
-- and the series blending lowers the friction-brake input towards 0 when the motor brakes, so a front wheel
-- locked by regeneration on low grip (or on lift-off, with no pedal at all) never raises absActive. Watch the
-- driven wheels directly: one turning clearly slower than the car moves (ground speed, which a locked wheel
-- doesn't affect) cuts regeneration at once, the friction brakes take over (blendBrakes), and regeneration
-- returns gradually after the wheels have rolled freely for a moment.
local function updateRegenSlip(dt)
  local vehicleSpeed = electrics.values.airspeed or 0
  local slipping = false
  if vehicleSpeed > cfg.regenSlipMinSpeed then
    local allowed = max(cfg.regenSlipRatio * vehicleSpeed, cfg.regenSlipMinDiff)
    for _, wd in ipairs(drivenWheels) do
      if not wd.isBroken and vehicleSpeed - abs(wd.wheelSpeed or vehicleSpeed) > allowed then
        slipping = true
        break
      end
    end
  end
  -- ABS, traction or stability control intervening (the flags are booleans or numbers, depending on the system):
  -- the same cut, hold and gradual return. Cutting regeneration only in the frames with the flag set switched the
  -- motor between 0 and full regeneration (about -250 Nm) every 2-3 frames while ABS cycled, and each step shook
  -- the driveline (braking from 60 km/h with the engine coupled)
  local e = electrics.values
  if flagOn(e.absActive) or flagOn(e.tcsActive) or flagOn(e.escActive) then
    slipping = true
  end
  if slipping then
    regenSlipHold = cfg.regenSlipHoldTime
    regenSlipCoef = 0
  elseif regenSlipHold > 0 then
    regenSlipHold = max(regenSlipHold - dt, 0)
  else
    regenSlipCoef = min(regenSlipCoef + dt / cfg.regenSlipRecoverTime, 1)
  end
  electrics.values.phevRegenSlipCut = 1 - regenSlipCoef
end

-- Regeneration on lift-off and braking. Limited only by the motor's generator torque at its current speed,
-- the 40 kW battery limit and the battery window (inside the module); there is no separate deceleration cap.
-- Lift-off: a target deceleration (it feels the same at every speed) that includes the drag of a coupled engine
-- in fuel cut. The motor regenerates what the drag leaves (none in a low gear where the drag alone exceeds the
-- target), so the deceleration doesn't change when the engine decouples or stops: the regeneration rises as the
-- drag through K0 fades. Measured on the real car: with the engine coupled in
-- overrun the car slows 0.3-0.4 m/s2 more than road load and the charge rises 6-15 kW; with the engine off
-- 0.2-0.3 m/s2 and 4-7 kW. coastCoef (0..1) blends the lift-off target in over the pedal's coast band.
-- Regeneration hands over to the friction brakes by road speed: full above 10 km/h, none below 5 km/h.
-- dragShaft: the coupled engine's drag torque at the input shaft (Nm, >= 0).
local function regenDemand(coastCoef, brake, speedKmh, coastDecelG, dragShaft)
  coastWheelTorque = 0
  local gearIndex = electrics.values.gearIndex or 0
  -- during a synchronised DCT shift the clutch slips (ratio below 0.9) while the module sets the motor: keep the
  -- demand, so the motor returns to it when the shift ends (it returned to 0 for a frame, then stepped to full
  -- regeneration: a 200-300 Nm step after every coasting downshift)
  local clutchClosed = gearbox and ((gearbox.clutchRatio or 0) >= 0.9 or module.syncTargetRatio ~= nil)
  if gearIndex <= 0 or not clutchClosed or speedKmh <= cfg.regenFadeEndKmh or (coastCoef <= 0 and brake <= 0.01) then
    return 0
  end
  -- not while a front wheel slips or ABS / TC / ESC intervene (updateRegenSlip)
  if regenSlipCoef <= 0 then
    return 0
  end
  -- the motor takes as much of the braking the driver asks for as it can (series blending);
  -- the friction brakes only add the rest (see blendBrakes)
  local ratio = wheelTorqueRatio()
  local fullCapacity = module.motorRegenTorqueAt(module, module.outputAV1) * ratio
  if fullCapacity < 1 then
    return 0
  end
  local fade = clampValue((speedKmh - cfg.regenFadeEndKmh) / max(cfg.regenFadeStartKmh - cfg.regenFadeEndKmh, 0.1), 0, 1)
  local capacity = fullCapacity * fade * regenSlipCoef
  local coastTarget = max(coastDecelG * vehicleMass * 9.81 * wheelRadius - (dragShaft or 0) * ratio, 0) * clampValue(coastCoef, 0, 1)
  local wanted = coastTarget
  if brake > 0.01 then
    wanted = wanted + frictionBrakeTorqueFor(brake, regenBrakeDevices)
  end
  wanted = min(wanted, capacity)
  coastWheelTorque = min(coastTarget, capacity)
  return -wanted / fullCapacity
end

-- Electric pedal map on wheel torque, like the maker's driver-demand torque: the pedal asks for a share of the most
-- torque the drivetrain could put on the wheels at this road speed in its best gear. The motor torque is that
-- divided by the current gear ratio, so an upshift doesn't change the pull and electric shifts can hardly be
-- felt. torqueAt gives the shaft torque curve the pedal scales (motor curve, or the system curve in Boost).
local function electricDriveDemand(pedal, torqueAt, coupledMap)
  local shaftAV = module.outputAV1
  local avail = module.motorDriveTorqueAt(module, shaftAV)
  local ratios = gearbox and gearbox.gearRatios
  local current = gearbox and abs(gearbox.gearRatio or 0) or 0
  local gearIndex = electrics.values.gearIndex or 0
  local share
  if avail < 1 or not ratios or current < 0.01 or gearIndex <= 0 then
    share = min(pedal * torqueAt(shaftAV) / max(avail, 1), 1)
  else
    local outAV = abs(gearbox.outputAV1 or 0)
    local best = 0
    for g = 1, gearbox.maxGearIndex or 0 do
      local r = ratios[g]
      if r and r > 0 and outAV * r < module.motorMaxAV then
        best = max(best, torqueAt(outAV * r) * r)
      end
    end
    share = best > 0 and min(pedal * best / current / avail, 1) or min(pedal * torqueAt(shaftAV) / avail, 1)
  end
  -- Hybrid and Boost (coupledMap): the engine can join at any moment, so from coupledMapFromKmh (blended over
  -- coupledMapBlendKmh) the pedal follows the map the car drives with the engine coupled: the current gear, scaled by
  -- the engine's full load (Hybrid) or the system torque (Boost). Before, engine off used the motor in its best gear
  -- and engine on the engine in the current gear: at 70 km/h in 6th the same pedal gave 19 kW electric and 11 kW with
  -- the engine running, and each start and stop changed the pull by about 40 % (in simulation the car surged
  -- every 2-4 s once the engine started at 15 kW)
  if coupledMap and avail >= 1 then
    local kmh = abs(electrics.values.wheelspeed or 0) * 3.6
    local f = clampValue((kmh - cfg.coupledMapFromKmh) / max(cfg.coupledMapBlendKmh, 1), 0, 1)
    if f > 0 then
      local av = max(abs(shaftAV), module.iceIdleAV or 84)
      local ref = coupledMap == 'boost' and systemTorqueAt(av) or iceFullTorqueAt(av)
      share = share + (min(pedal * ref / avail, 1) - share) * f
    end
  end
  return share
end

local function motorCurveAt(av)
  return module.motorDriveTorqueAt(module, av)
end

-- Regeneration torque the motor really puts on the wheels (Nm, >= 0), the part that may replace friction. The motor's own
-- torque times the ratio is only an estimate: the shaft's inertia torque while it slows down and the gearbox losses come off
-- it, and the friction brake was cut by the full estimate (the deceleration per pedal fell short of the car without
-- regeneration, game test: 6-30 % less). The gearbox output torque is what reaches the wheels: the estimate is lowered to it
-- (never raised: the K0 drag of a coupled engine and the shaft drag belong to the engine braking, they are extra on top of the
-- pedal, as in a car without regeneration), but not below cfg.regenMeasureFloor of it (guards a bad reading). During a
-- synchronised DCT shift the motor is busy matching the shaft speed and the slipping clutch carries the braking, so the
-- gearbox output torque is used as it is (forward gears only: in P the park lock torque must not count).
local function regenWheelTorque()
  local ratio = wheelTorqueRatio()
  local estimate = max(-(module.motorTorque or 0), 0) * ratio
  if not gearbox or (electrics.values.gearIndex or 0) <= 0 then
    return estimate
  end
  -- (the gearbox output torque is already after the gearbox: no driveline efficiency on it, that factor is part of the estimate)
  local passed = max(-(gearbox.outputTorque1 or 0), 0) * finalDriveRatio
  if module.syncTargetRatio then
    return passed
  end
  local delivered = passed - max(-(module.k0Torque or 0), 0) * abs(gearbox.gearRatio or 0) * finalDriveRatio
  return clampValue(delivered, estimate * cfg.regenMeasureFloor, estimate)
end

-- brake input that gives a given total friction torque on the wheels (the inverse of frictionBrakeTorqueFor: the torque
-- rises with the input, piecewise linear through each wheel's brakeInputSplit), by bisection
local function brakeInputForFriction(target)
  local lo, hi = 0, 1
  if frictionBrakeTorqueFor(1) <= target then
    return 1
  end
  for _ = 1, 14 do
    local mid = (lo + hi) * 0.5
    if frictionBrakeTorqueFor(mid) < target then
      lo = mid
    else
      hi = mid
    end
  end
  return (lo + hi) * 0.5
end

-- Brake blending: the pedal sets the total braking (BeamNG's normal pedal feel, the same as the car without regeneration);
-- the regeneration the motor actually delivers at the wheels (regenWheelTorque) replaces exactly that much of the friction
-- brake. Friction takes over by itself when regeneration fades (low speed, full battery, DCT shift, ABS / ESC / TC
-- intervention).
--
-- regenAxle "driven" (the default; front-wheel drive: the front axle, rear-wheel drive: the rear): the regeneration brakes
-- the driven axle only, so only that axle's friction brake is lowered (axleBrakeScale, applied to its wheels in
-- updateWheelsIntermediate); the pedal, and with it the other axle's brakes and the front / rear brake balance, stay as
-- they are. regenDemand limits the regeneration to the friction torque the driven axle would have applied at this pedal, so
-- the total braking still follows the pedal. regenAxle "all" (the maker's brake servo lowers the pressure of all four brakes):
-- the friction of every wheel is lowered by the same share, which moves the balance towards the driven axle.
local function blendBrakes(driverBrake)
  electrics.values.phevDriverBrake = driverBrake
  if driverBrake <= 0.01 then
    axleBrakeScale = 1
    electrics.values.phevFrictionBrake = 0
    electrics.values.phevRegenBrakeShare = 0
    return
  end
  -- coast regeneration is extra deceleration on top of the pedal, so only the pedal part replaces friction
  local regenForPedal = max(regenWheelTorque() - coastWheelTorque, 0)
  if cfg.regenAxle ~= "all" then
    local axleRequested = frictionBrakeTorqueFor(driverBrake, drivenBrakeDevices)
    axleBrakeScale = 1
    if axleRequested > 1 then
      axleBrakeScale = clampValue(1 - regenForPedal / axleRequested, 0, 1)
    end
    local total = frictionBrakeTorqueFor(driverBrake)
    local share = total > 1 and clampValue(axleRequested * (1 - axleBrakeScale) / total, 0, 1) or 0
    electrics.values.brakelights = 1
    electrics.values.phevFrictionBrake = driverBrake * (1 - share)
    electrics.values.phevRegenBrakeShare = share
    return
  end
  local requested = frictionBrakeTorqueFor(driverBrake)
  local scale = 1
  if requested > 1 then
    scale = clampValue(1 - regenForPedal / requested, 0, 1)
  end
  -- the wheels' own input curve (brakeInputSplit / brakeSplitCoef) is not linear: scaling the input by the share would
  -- brake the rear wheels more than the share says, so the input is solved for the reduced total torque
  local input = scale >= 1 and driverBrake or brakeInputForFriction(requested * scale)
  electrics.values.brake = input
  electrics.values.brakelights = 1
  electrics.values.phevFrictionBrake = input
  electrics.values.phevRegenBrakeShare = 1 - scale
end

-- electric creep: with the clutch closed at standstill the motor provides the creep an automatic has
local function creepDemand(pedal, brake, speedKmh, shaftAV)
  if module.launchMode ~= "electric" or not gearEngaged() or pedal > 0.02 or brake > 0.05 then
    return 0
  end
  local coef = clampValue((cfg.creepMaxSpeedKmh - speedKmh) / 2, 0, 1)
  local avail = module.motorDriveTorqueAt(module, shaftAV)
  if avail < 1 then
    return 0
  end
  return coef * cfg.creepTorque / avail
end

local function iceCoolant()
  local thermals = ice and ice.thermals
  return thermals and (thermals.coolantTemperature or thermals.oilTemperature) or 90
end

local function iceIsCold()
  local thermals = ice and ice.thermals
  local coolant = thermals and (thermals.coolantTemperature or thermals.oilTemperature)
  return coolant ~= nil and coolant < cfg.warmUpTemperature
end

------------------------------------------------------------------------------------------------

-- linear interpolation in a {{x, y}, ...} table, clamped at the ends
local function interpolate(points, x)
  if x <= points[1][1] then
    return points[1][2]
  end
  for i = 2, #points do
    if x <= points[i][1] then
      local x0, y0, x1, y1 = points[i - 1][1], points[i - 1][2], points[i][1], points[i][2]
      return y0 + (y1 - y0) * (x - x0) / max(x1 - x0, 1e-9)
    end
  end
  return points[#points][2]
end

-- a power threshold in W: a flat kW number, or a [km/h, kW] table interpolated by road speed
local function kwBySpeed(v, speedKmh)
  if type(v) == 'table' then return interpolate(v, speedKmh) * 1000 end
  return (v or 0) * 1000
end

-- The driver's pedal for the strategy: low-passed over cfg.pedalFilterTau, with a coast band at the bottom. Below
-- cfg.coastPedal the pedal blends the lift-off deceleration in (coastCoef 1 at 0, 0 at coastPedal); above it the
-- drive demand starts from 0 (drivePedal), so the torque at the wheels runs continuously from coasting to driving.
-- Lift-off (engine stop rules) has hysteresis: it begins at or below coastPedal and ends at cfg.coastPedalExit.
-- With a plain 0.02 threshold a recorded pedal hovering at 0.01-0.04 switched the Boost powertrain every 0.5-2 s between
-- "engine +21 to +46 kW, motor charging" and "engine drag, motor -29 to -44 kW coast regeneration".
local function updatePedalState(dt, pedal)
  pedalFiltered = pedalFiltered + (pedal - pedalFiltered) * min(1, dt / max(cfg.pedalFilterTau, 1e-3))
  local p = pedalFiltered
  if coasting then
    if p >= cfg.coastPedalExit then
      coasting = false
    end
  elseif p <= cfg.coastPedal then
    coasting = true
  end
  local drivePedal = clampValue((p - cfg.coastPedal) / (1 - cfg.coastPedal), 0, 1)
  local coastCoef = clampValue(1 - p / cfg.coastPedal, 0, 1)
  return drivePedal, coastCoef
end

-- Charging from the engine's spare torque (Boost below its hold level, Hybrid charge sustaining): faded in over
-- cfg.chargeRampTime while the driver keeps a demand, faded out quickly on lift-off and while the car slows down
-- faster than it would coast (smoothed deceleration beyond the road-load coast-down, from chargeDecelStart to
-- chargeDecelFull m/s2). Without it the engine pushed charging power into the battery in every pedal pulse of a
-- gentle slow-down (100 -> 55 km/h: engine +40 kW, motor -10 to -20 kW, then engine drag when the pedal lifted).
-- A light pedal at motorway speed that only slows the car more gently than coasting still charges (the real car's
-- charge rose 0.5-1.7 % per minute between accelerations at 130-150 km/h on a logged drive).
local function updateChargeCoef(dt, drivePedal)
  local v = abs(electrics.values.wheelspeed or 0)
  if lastSpeedMs then
    local a = (v - lastSpeedMs) / max(dt, 1e-3)
    accelSmoothed = accelSmoothed + (a - accelSmoothed) * min(1, dt / 1.0)
  end
  lastSpeedMs = v
  -- road-load coast-down deceleration at this speed
  local coastDown = (315 + 0.356 * v * v) * 0.93 / max(vehicleMass, 500)
  local excess = -accelSmoothed - coastDown
  local slowing = clampValue((excess - cfg.chargeDecelStart) / max(cfg.chargeDecelFull - cfg.chargeDecelStart, 0.01), 0, 1)
  local wanted = (drivePedal > 0 and not coasting) and (1 - slowing) or 0
  if wanted > chargeCoef then
    chargeCoef = min(chargeCoef + dt / max(cfg.chargeRampTime, 0.05), wanted)
  else
    chargeCoef = max(chargeCoef - dt / 0.3, wanted)
  end
  electrics.values.phevChargeCoef = chargeCoef
end

-- Drag torque of the coupled engine in fuel cut at the input shaft (Nm, >= 0): BeamNG's friction and engine brake
-- (iceNetTorqueFor at zero throttle) while K0 is closed; the low-passed K0 drag while K0 opens or closes.
local function engineDragTorque(dt)
  local k0Drag = max(-(module.k0Torque or 0), 0)
  k0DragFiltered = k0DragFiltered + (k0Drag - k0DragFiltered) * min(1, dt / 0.15)
  if module.iceCoupled and ice and ice.torqueCurve then
    local av = abs(ice.outputAV1 or 0)
    if av > 1 then
      return max(-iceNetTorqueFor(0, av), 0)
    end
    return 0
  end
  if (module.k0Ratio or 0) > 0.01 then
    return k0DragFiltered
  end
  return 0
end

-- Engine on/off for Hybrid and Boost, calibrated on the logs of the real car:
--   * the engine stops when the load falls away (lift-off, braking, light load), at any speed
--   * it starts on requested wheel power: speed dependent in Hybrid, eagerly in Boost
local function updateEngineRequest(dt, pedal, brake, speedKmh, shaftAV, isBoost)
  local soc = usableSoc()
  local target = chargeFunction and cfg.chargeTarget or holdTarget
  local deficit = target - soc

  -- requested power: the pedal share of what the drivetrain can deliver at this shaft speed
  local refTorque = max(module.motorDriveTorqueAt(module, shaftAV), iceFullTorqueAt(shaftAV))
  local demandPower = pedal * refTorque * max(shaftAV, 0)

  local onPower
  if isBoost then
    onPower = kwBySpeed(cfg.boostOnPowerKW, speedKmh)
  else
    onPower = interpolate(cfg.hybridOnPower, speedKmh) * 1000
    if soc < target + cfg.socMarginForPower then
      local x = clampValue((soc - target) / cfg.socMarginForPower, 0, 1)
      onPower = onPower * (cfg.hybridLowSocFactor + (1 - cfg.hybridLowSocFactor) * x)
    end
  end
  local offPower = isBoost and kwBySpeed(cfg.boostOffPowerKW, speedKmh) or onPower * cfg.hybridOffPowerRatio

  -- Charge mode "always charges" (as on the real car): below its target the engine runs and charges, also at a standstill and in P
  local chargeWanted = chargeFunction and not isBoost and deficit > cfg.sustainBand

  local kickdown = kickdownActive
  -- lift-off with hysteresis on the filtered pedal (updatePedalState): a pedal hovering at the threshold no
  -- longer toggles between the lift-off and the driving rules
  local liftOff = coasting or brake > 0.05
  -- above the Electric range the engine drives in Hybrid as soon as the driver actually pulls
  local beyondEV = not isBoost and speedKmh > cfg.hybridEngineSpeedKmh and demandPower > offPower
  -- Boost's battery target (half the usable window) is kept while driving, never by idling at standstill;
  -- Hybrid's battery hold may charge at standstill (below)
  local standing = speedKmh < 1 and pedal < 0.05
  local socRequest = not (isBoost and standing)
  local wantOn = demandPower > onPower or kickdown or beyondEV or (socRequest and deficit > cfg.deficitStart)
    or (soc <= 0 and not liftOff) or chargeWanted
  -- Warm-up is graded (measured on the real car, 35 cold starts): below about 50 C coolant the engine never stops, not even
  -- while slowing down or at standstill; between 50 and 70 C it stays on while driving but may stop when
  -- slowing down below about 10 km/h or standing (stops in the logs: median 63 C, median 4 km/h).
  local coolant = iceCoolant()
  local hardWarmUp = warmingUp and coolant < cfg.warmUpHardTemperature
  local softWarmUpHold = warmingUp and not ((liftOff or standing) and speedKmh < cfg.warmUpReleaseKmh)
  local holdOn = hardWarmUp or softWarmUpHold or (socRequest and deficit > cfg.deficitStart * 2) or kickdown or chargeWanted

  -- reasons to stop: lift-off / braking (any speed), light load, standstill
  if liftOff then
    coastTimer = coastTimer + dt
  else
    coastTimer = 0
  end
  -- light load with the pedal still pressed (lift-off has its own delay, liftOffDelay*: before, this 2 s timer also ran on
  -- lift-off and stopped the engine after 2 s at any speed)
  if demandPower < offPower and not liftOff then
    lowDemandTimer = lowDemandTimer + dt
  else
    lowDemandTimer = 0
  end
  if speedKmh < 1 and pedal < 0.05 then
    standstillTimer = standstillTimer + dt
  else
    standstillTimer = 0
  end
  local standstillCharge = not isBoost and standstillTimer > 0 and (deficit >= cfg.standstillChargeDeficit or chargeWanted)
  standstillChargeWanted = standstillCharge
  -- Lift-off stop delay by mode and road speed (measured on the real car): in Hybrid-like
  -- stretches a warm engine stops after a median 2 s of lift-off at 60-100 km/h, 4 s at 100-140 and 6 s above 140 km/h
  -- (stops up to about 210 km/h); in Boost-like stretches the engine stays on longer (2 / 4 / 14 s, and it stopped in only
  -- 1 of 20 lift-offs above 140 km/h). On the real car Boost keeps it running longer than Hybrid.
  local liftOffDelay = interpolate(isBoost and cfg.liftOffDelayBoost or cfg.liftOffDelayHybrid, speedKmh)
  local wantOff = (coastTimer > liftOffDelay or lowDemandTimer > cfg.lowDemandOffTime) and not standstillCharge

  if iceWanted then
    iceOnTimer = iceOnTimer + dt
    if wantOff and not holdOn and not wantOn and iceOnTimer > cfg.minOnTime then
      iceWanted = false
      iceOffTimer = 0
    end
  else
    iceOffTimer = iceOffTimer + dt
    if wantOn and (iceOffTimer > cfg.minOffTime or kickdown or beyondEV) then
      iceWanted = true
      iceOnTimer = 0
      lowDemandTimer = 0
      coastTimer = 0
    end
  end
  return deficit, kickdown
end

-- Hybrid with the engine coupled: the driver's demand is pedal x the engine's full-load torque. The engine is
-- given its share as a torque (iceThrottleFor turns it into a throttle, relative to the pedal) and the motor fills
-- what the engine doesn't deliver.
--
-- What the engine delivers, for the motor fill (Hybrid and Boost): the engine's own net torque (BeamNG's
-- combustionTorque: combustion minus friction and engine braking, so turbo lag and the warm-up limit show), low-passed
-- over cfg.fillEngineTorqueTau, while K0 is closed. The torque measured at K0 also carries the engine's inertia
-- torque and the K0 spring's swing in a driveline shuffle; filling against it once per graphics frame drove the
-- shuffle (Hybrid warm-up at 8-18 km/h in 1st, steady pedal: motor 0 <-> 3-11 kW and engine -5 <-> 21 kW from one
-- 0.2 s sample to the next). With K0 not fully closed the K0 torque is used, as before.
local deliveredEngineTorque = 0
local engineTorqueFiltered = nil
local function updateDeliveredEngineTorque(dt)
  local net = ice and ice.combustionTorque
  if type(net) ~= "number" or not module.iceCoupled or (module.k0Ratio or 0) < 0.99 then
    engineTorqueFiltered = nil
    deliveredEngineTorque = max(module.k0Torque or 0, 0)
    return
  end
  if engineTorqueFiltered and (module.syncTargetRatio or module.syncIceLimit) then
    -- a DCT shift: the module sets the motor and reduces the engine's torque for the speed change; keep the
    -- estimate from before, so the fill doesn't rise against the reduction and fall back after it
    deliveredEngineTorque = max(engineTorqueFiltered, 0)
    return
  end
  if engineTorqueFiltered then
    engineTorqueFiltered = engineTorqueFiltered + (net - engineTorqueFiltered) * min(1, dt / cfg.fillEngineTorqueTau)
  else
    engineTorqueFiltered = net
  end
  deliveredEngineTorque = max(engineTorqueFiltered, 0)
end

-- charge sustaining with a band (cfg.sustainBand either side of the target): without it the motor switched
-- several times a second between charging and filling for a capped cold engine, at a hair below or above target
local sustainCharging = false

-- throttle factor (relative to the pedal) for an engine torque; 1 without the engine's data
local function iceFactorFor(torque, pedal, shaftAV)
  local throttle = iceThrottleFor(torque, abs(ice.outputAV1 or shaftAV))
  if not throttle then
    return 1
  end
  return throttle / max(pedal, 0.01)
end

-- throttle for an engine torque, as a factor on the raw pedal the engine reads (iceFactorAbsolute); a raw pedal of
-- 0 gives no throttle at all (fuel cut)
local function iceFactorAbs(torque, rawPedal, shaftAV)
  local throttle = iceThrottleFor(torque, abs(ice.outputAV1 or shaftAV))
  if not throttle then
    return 1
  end
  return throttle / max(rawPedal, 1e-3)
end

-- motor demand (fraction of the drive or the generator curve) for a motor torque
local function motorFraction(torque, shaftAV)
  if torque >= 0 then
    local avail = module.motorDriveTorqueAt(module, shaftAV)
    return avail > 1 and clampValue(torque / avail, 0, 1) or 0
  end
  local avail = module.motorRegenTorqueAt(module, shaftAV)
  return avail > 1 and clampValue(torque / avail, -1, 0) or 0
end

-- Lift-off / braking with the engine coupled (Hybrid, Boost, warm-up): the engine goes to fuel cut and drags; between
-- pedal 0 and coastPedal it is given part of its drag back as torque (net -drag x coastCoef), so the wheel torque
-- runs continuously into the drive range. The motor regenerates the rest of the lift-off target (regenDemand).
-- Returns motor demand, engine throttle factor (absolute).
local function coupledCoast(coastCoef, brake, speedKmh, decelG, shaftAV, rawPedal, drag)
  engineShare = 1
  fillSmoother:set(0)
  -- Warm-up coasting: while the engine is held on to warm up (coolant below warmUpTemperature) and the driver coasts
  -- (pedal fully up, no brake), the engine keeps firing at about warmCoastPowerKW instead of dragging in fuel cut, and
  -- the motor puts that power into the battery on top of the normal lift-off regeneration, so the wheels feel the same
  -- deceleration. The engine's torque comes through its idle controller (target a little above its speed, throttle
  -- capped at what the torque needs, module chargeIdleAV), as the pedal is 0. The real car charges while
  -- coasting during warm-up; logs: a mean 14 kW of engine power until 70 C.
  warmCoastTouched = true
  local warmCoast = warmingUp and cfg.warmCoastPowerKW > 0 and coastCoef > 0.98 and brake <= 0.05
    and speedKmh > cfg.regenFadeStartKmh and usableSoc() < cfg.warmCoastMaxSoc and (electrics.values.gearIndex or 0) >= 1
  if warmCoast then
    warmCoastCoef = min(warmCoastCoef + frameDt / max(cfg.warmCoastRampTime, 0.05), 1)
  else
    warmCoastCoef = 0
  end
  electrics.values.phevWarmCoast = warmCoastCoef
  if warmCoastCoef > 0 then
    local av = abs(ice.outputAV1 or shaftAV)
    local cap = interpolate(cfg.warmUpTorqueCap, iceCoolant()) * iceFullTorqueAt(av)
    local torque = min(cfg.warmCoastPowerKW * 1000 / max(av, 50), cfg.warmCoastMaxTorque, cap) * warmCoastCoef
    local throttle = iceThrottleFor(torque, av)
    if throttle then
      idleReq.av = av + cfg.warmCoastLeadRPM / avToRPM
      idleReq.throttle = throttle
    end
    -- the lift-off target without engine drag (the engine fires), plus what the engine delivers, into the battery
    local regen = regenDemand(coastCoef, brake, speedKmh, decelG, 0)
    return clampValue(regen + motorFraction(-deliveredEngineTorque, shaftAV), -1, 0), 0
  end
  local factor = 0
  if rawPedal > 1e-3 and coastCoef < 1 then
    factor = iceFactorAbs(-drag * coastCoef, rawPedal, shaftAV)
  end
  return regenDemand(coastCoef, brake, speedKmh, decelG, drag), factor
end

-- Charging at a standstill (Charge mode below its target, Hybrid charge sustaining at a standstill): the engine idles
-- up (in P / N to standstillChargeRPM through its idle controller, in a drive gear at its normal idle, so the creep
-- doesn't rise) and the motor generates standstillChargeTorque (P / N) or standstillChargeTorqueD (drive gear), faded in
-- over standstillChargeRampTime and drooped when the engine sags below its idle target, so it can't be stalled. Before,
-- the engine ran at a standstill in Charge mode but nothing loaded it (the lift-off branch regenerates only while
-- rolling). Real car: "not full power, but always a bit"; no standstill charge was logged. Provisional figures,
-- to be measured on the real car: about 11 kW in P (60 Nm at 1,800 rpm); in D 2 kW (25 Nm at idle, assumed).
local function standstillChargeDemand(dt, wanted, shaftAV)
  local gearIndex = electrics.values.gearIndex or 0
  local parked = gearIndex == 0
  if wanted then
    standstillChargeCoef = min(standstillChargeCoef + dt / max(cfg.standstillChargeRampTime, 0.05), 1)
  else
    standstillChargeCoef = max(standstillChargeCoef - dt / 0.3, 0)
  end
  local idleRPM = parked and cfg.standstillChargeRPM or ((module.iceIdleAV or 84) * avToRPM)
  if wanted and parked then
    -- BeamNG's idle controller is proportional only (throttle = P x speed error), so under the generator load it settles
    -- short of its target (about 1,680 rpm for 1,800 with 60 Nm), and the droop below then takes the load down. The
    -- target is lifted by the error the load needs: the throttle for the torque the motor takes now, over P. The motor's
    -- own torque is used, not the demand, so a pack that accepts less (cold, the top of the window) leaves no extra
    -- throttle to rev the engine past the target.
    local targetAV = cfg.standstillChargeRPM / avToRPM
    local throttle = iceThrottleFor(max(-(module.motorTorque or 0), 0), targetAV) or 0
    idleReq.av = targetAV + min(throttle / max(ice.idleControllerP or 0.01, 1e-4), cfg.standstillChargeLeadMaxRPM / avToRPM)
    idleReq.throttle = cfg.standstillChargeThrottle
  end
  if standstillChargeCoef <= 0 then
    return 0
  end
  local iceRPM = abs(ice.outputAV1 or 0) * avToRPM
  local droop = clampValue((iceRPM - (idleRPM - cfg.standstillChargeDroopRPM)) / max(cfg.standstillChargeDroopRPM * 0.6, 1), 0, 1)
  local torque = (parked and cfg.standstillChargeTorque or cfg.standstillChargeTorqueD) * standstillChargeCoef * droop
  -- towards the Charge mode target the charging tapers (chargeTaperBand), down to chargeTaperMin
  if chargeFunction then
    torque = torque * clampValue((cfg.chargeTarget - usableSoc()) / max(cfg.chargeTaperBand, 1e-3), cfg.chargeTaperMin, 1)
  end
  return motorFraction(-torque, shaftAV)
end

local function hybridCoupledSplit(dt, pedal, rawPedal, coastCoef, brake, speedKmh, shaftAV, deficit, kickdown, drag)
  local standstillWanted = pedal <= 0 and not kickdown and speedKmh < cfg.standstillChargeMaxKmh
    and (standstillChargeWanted or (chargeFunction and deficit > cfg.sustainBand))
  local standstillMotor = standstillChargeDemand(dt, standstillWanted, shaftAV)
  if standstillWanted or standstillMotor < 0 then
    engineShare = 1
    fillSmoother:set(0)
    return min(standstillMotor, 0), 0
  end
  if pedal <= 0 and not kickdown then
    return coupledCoast(coastCoef, brake, speedKmh, bMode and cfg.coastDecelB or cfg.coastDecelD, shaftAV, rawPedal, drag)
  end

  local engineTorque = deliveredEngineTorque

  if kickdown then
    -- kickdown releases the full motor on top of the engine (the module caps it at system power); at the top speed the
    -- limiter scales both (before, kickdown bypassed it, and keyboard throttle is always kickdown)
    engineShare = 1
    fillSmoother:set(speedLimitCoef)
    local engineTorque = speedLimitCoef >= 0.999 and 1e6 or speedLimitCoef * iceFullTorqueAt(shaftAV)
    return speedLimitCoef, iceFactorAbs(engineTorque, rawPedal, shaftAV)
  end

  local fullTorque = iceFullTorqueAt(shaftAV)
  local demandTorque = pedal * fullTorque

  if deficit > cfg.sustainBand then
    sustainCharging = true
  elseif deficit < -cfg.sustainBand then
    sustainCharging = false
  end
  if sustainCharging and pedal < 0.9 then
    -- charge sustaining / Charge mode: the motor generates and the engine covers the demand plus that (faded in
    -- while the demand holds, out on lift-off and while slowing: chargeCoef)
    engineShare = 1
    local regenAvail = module.motorRegenTorqueAt(module, shaftAV)
    local genTorque = min(chargeFunction and cfg.chargeTorque or cfg.sustainTorque, regenAvail) * chargeCoef
    local fill = max(demandTorque + genTorque - engineTorque, 0) - genTorque
    return fillSmoother:get(motorFraction(fill, shaftAV), dt), iceFactorAbs(demandTorque + genTorque, rawPedal, shaftAV)
  end

  -- The engine takes the demand up to the torque at which the real car's turbo starts to build boost (logs: from
  -- about 160 Nm before friction, cfg.hybridEngineTorque net); of the demand above that the motor fills
  -- cfg.hybridFillShare and the engine the rest. The real car assists sparingly: with the engine running on the
  -- motorway its charge falls a median 0.12-0.15 % (raw) per km, and the engine torque keeps rising with the demand
  --. A throttle trim against BeamNG's live boost (which builds at cruising load) cycled every
  -- few seconds and left most of the cruise to the motor (1.3 % per km).
  -- The fill fades out over the last socMarginForPower above the hold target: at the reserve the motor filled the
  -- whole gap (33 kW at 133 km/h) until sustaining began 0.5 % below it, so it alternated draining and charging.
  -- Cruise assist: between cfg.hybridAssistTorque and cfg.hybridEngineTorque the motor takes cfg.hybridAssistShare of
  -- the demand above cfg.hybridAssistTorque, faded in with road speed (cfg.hybridAssistSpeed). The real car takes
  -- about 2.4 kWh/100 km from the battery at a steady 130 km/h in Hybrid (6.7 L/100 km), about 2.1 kW at the shaft
  -- once the 0.7 kW of auxiliaries are taken out; the logs give -0.15 % (raw) per km above 130 km/h, -0.12 % at
  -- 110-130 and +0.04 % at 80-110 km/h. Without it the engine carried the whole steady cruise (motor 0.02 kW at 130).
  local excess = max(demandTorque - cfg.hybridEngineTorque, 0)
  local assistBand = clampValue(demandTorque - cfg.hybridAssistTorque, 0, max(cfg.hybridEngineTorque - cfg.hybridAssistTorque, 0))
  local assist = assistBand * cfg.hybridAssistShare * interpolate(cfg.hybridAssistSpeed, speedKmh)
  local socFade = clampValue(-deficit / cfg.socMarginForPower, 0, 1)
  local engineTarget = demandTorque - (excess * cfg.hybridFillShare + assist) * socFade
  -- a cold engine's torque limit (warm-up); the motor fills the rest
  engineTarget = min(engineTarget, interpolate(cfg.warmUpTorqueCap, iceCoolant()) * fullTorque)
  engineShare = demandTorque > 1 and engineTarget / demandTorque or 1
  local fill = max(demandTorque - engineTorque, 0)
  return fillSmoother:get(motorFraction(fill, shaftAV), dt), iceFactorAbs(engineTarget, rawPedal, shaftAV)
end

-- Boost with the engine coupled and the pedal in the drive range: the pedal asks for a share of the system torque
-- (400 Nm / 160 kW). The engine is given its part as a torque (iceThrottleFor), up to its full load; the motor
--   * fills what the engine doesn't deliver of that part (turbo lag, a cold engine's load limit),
--   * boosts above the engine's full load, faded in with the pedal from the point where the engine alone reaches
--     full load to kickdown: ((pedal - pFull) / (kickdown - pFull))^boostExponent; full boost at kickdown,
--   * below the Boost hold level generates from the engine's spare torque (boostChargeTorque, faded by chargeCoef).
-- Before, the engine got the raw pedal (BeamNG's throttle map gives more torque than the pedal share at low and mid
-- pedal, less at high rpm) and the charging torque was turned into a throttle as if torque were linear in throttle:
-- 3 % pedal gave the engine 20-45 kW, and at high speed the motor filled the gap between pedal x system torque and
-- the throttle map's torque at every steady pedal above about 0.6 (in simulation: 47 -> 38 % at 140-200 km/h, the
-- real car stayed at 45-47 %).
local function phevCoupledDrive(dt, pedal, rawPedal, shaftAV, kickdown)
  local sys = systemTorqueAt(shaftAV)
  local target = (kickdown and speedLimitCoef or pedal) * sys
  local fullIce = max(iceFullTorqueAt(shaftAV), 20)
  local engineTarget = min(target, fullIce)
  local boost = 0
  if target > fullIce then
    local pFull = fullIce / max(sys, 1)
    local w = 1
    if not kickdown then
      w = clampValue((pedal - pFull) / max(cfg.kickdownPedal - pFull, 0.05), 0, 1) ^ cfg.boostExponent
    end
    boost = (target - fullIce) * w
  end
  local gen = 0
  local deficit = holdTarget - usableSoc()
  if deficit > 0 and not kickdown then
    local warmCap = interpolate(cfg.warmUpTorqueCap, iceCoolant())
    local regenAvail = module.motorRegenTorqueAt(module, shaftAV)
    gen = min(cfg.boostChargeTorque * clampValue(deficit / 0.01, 0, 1) * chargeCoef, max(fullIce * warmCap - engineTarget, 0), regenAvail)
  end
  local motorTorque = boost + max(engineTarget + gen - deliveredEngineTorque, 0) - gen
  engineShare = target > 1 and engineTarget / target or 1
  if kickdown then
    fillSmoother:set(motorFraction(motorTorque, shaftAV))
  end
  return fillSmoother:get(motorFraction(motorTorque, shaftAV), dt), iceFactorAbs(engineTarget + gen, rawPedal, shaftAV)
end

-- Brake lights from regeneration (see the comment inside); runs each frame from updateGFX
local function updateRegenLights()
  -- Brake lights from regeneration without the brake pedal (UN R13-H 5.2.22.4: not below 0.7 m/s2, may between 0.7 and
  -- 1.3, shall above 1.3 m/s2): the regenerative deceleration (motor braking torque at the wheels / radius / mass) lights
  -- them from regenLightOnDecel and they go out below regenLightOffDecel. Before: above 15 kW of regeneration.
  local regenDecel = max(-(module.motorTorque or 0), 0) * wheelTorqueRatio() / max(wheelRadius, 0.2) / max(vehicleMass, 500)
  if regenDecel >= cfg.regenLightOnDecel then
    lights.regenOn = true
  elseif regenDecel < cfg.regenLightOffDecel then
    lights.regenOn = false
  end
  electrics.values.phevRegenDecel = regenDecel
  if lights.regenOn then
    electrics.values.brakelights = 1
  end
end

-- Emergency stop signal; runs each frame after the brake blending
local function updateEmergencyStop(dt, brake)
  -- Emergency stop signal (UN R48 6.23 / R13-H 5.2.23): above essMinKmh, at a deceleration of essOnDecel or more (or with
  -- ABS cycling under the brake) the stop lamps flash at essFlashHz; they stop below essOffDecel. If the braking goes on
  -- below essHazardKmh while they flash, the hazard lights come on (essHazardAtStop), until the car moves off again.
  local air = abs(electrics.values.airspeed or 0)
  if lights.lastAir then
    local a = (lights.lastAir - air) / max(dt, 1e-3)
    lights.decel = lights.decel + (a - lights.decel) * min(1, dt / 0.15)
  end
  lights.lastAir = air
  local absCycling = flagOn(electrics.values.absActive) and brake > 0.5
  if not lights.ess and air * 3.6 > cfg.essMinKmh and (lights.decel >= cfg.essOnDecel or absCycling) then
    lights.ess = true
    lights.essTimer = 0
  elseif lights.ess and cfg.essHazardAtStop and air * 3.6 < cfg.essHazardKmh then
    -- still braking below essHazardKmh: the hazard lights come on and the stop lamps burn steadily (the car's manual)
    if not lights.hazard then
      lights.hazard = true
      pcall(function() electrics.set_warn_signal(true) end)
    end
    lights.ess = false
  elseif lights.ess and lights.decel < cfg.essOffDecel and not absCycling then
    lights.ess = false
  end
  if lights.hazard and air > 3 and (electrics.values.throttle or 0) > 0.05 then
    lights.hazard = false
    pcall(function() electrics.set_warn_signal(false) end)
  end
  if lights.ess then
    lights.essTimer = lights.essTimer + dt
    electrics.values.brakelights = (lights.essTimer * cfg.essFlashHz) % 1 < 0.5 and 1 or 0
  end
  electrics.values.phevEmergencyStop = lights.ess and 1 or 0
end

-- brake torque vectoring (the maker's "extended electronic differential lock" on the open front differential): in a corner under drive the inner
-- front wheel is braked a little, so the open differential passes more torque to the outer wheel and the car turns in
-- with less understeer. Runs at the physics rate between the wheel speed update and the wheel torque update (BeamNG
-- controller hook updateWheelsIntermediate, as brakeControl and electronicDiffLock do), adding to the wheel's
-- desiredBrakingTorque. The turn direction and lateral acceleration come from the undriven rear wheels: in a turn of
-- radius R the outer rear wheel turns faster by track / R, so a_y = v x (v_outer - v_inner) / track. Braking torque on
-- the inner front wheel: tvTorqueShare x the drive torque per front wheel x ramp(a_y from tvMinLatAccel to
-- tvFullLatAccel), at most tvMaxBrakeShare x the wheel's brake torque, between tvMinKmh and tvMaxKmh, only while
-- driving (no brake pedal, no ABS / ESC intervention), rate-limited by tvRate (Nm/s). BeamNG's brake-based traction
-- control already brakes a spinning driven wheel (EDS); this adds the cornering part (provisional values).
local function updateWheelsIntermediate(dt)
  if cfg.regenAxle ~= "all" and axleBrakeScale < 0.999 and (electrics.values.parkingbrake or 0) < 0.05 then
    -- regeneration replaces part of the driven axle's friction brake (blendBrakes). Brake torque added by the stability
    -- systems would be scaled too, but regeneration is cut while they intervene (updateRegenSlip), so the scale is 1
    for _, wd in ipairs(drivenBrakeDevices) do
      if not wd.isBroken then
        wd.desiredBrakingTorque = (wd.desiredBrakingTorque or 0) * axleBrakeScale
      end
    end
  end
  if not cfg.tvEnabled or not module or not tv.frontL or not tv.frontR or not tv.rearL or not tv.rearR then
    return
  end
  local v = abs(electrics.values.airspeed or 0)
  local kmh = v * 3.6
  local driveShaft = (module.motorTorque or 0) + (module.k0Torque or 0)
  -- the driver's pedal, not the friction input after the blending (that falls to 0 while the motor brakes: no torque
  -- vectoring under braking)
  local brakeIn = electrics.values.phevDriverBrake or electrics.values.brake or 0
  local wanted = 0
  local innerLeft = false
  if kmh > cfg.tvMinKmh and kmh < cfg.tvMaxKmh and driveShaft > 5 and brakeIn < 0.02 and (electrics.values.gearIndex or 0) >= 1
    and (electrics.values.parkingbrake or 0) < 0.05
    and not flagOn(electrics.values.absActive) and not flagOn(electrics.values.escActive) then
    local dv = (tv.rearR.wheelSpeed or 0) - (tv.rearL.wheelSpeed or 0)
    local ay = v * abs(dv) / max(cfg.tvTrack, 0.5)
    innerLeft = dv > 0
    local ramp = clampValue((ay - cfg.tvMinLatAccel) / max(cfg.tvFullLatAccel - cfg.tvMinLatAccel, 0.1), 0, 1)
    if ramp > 0 then
      local perWheel = driveShaft * wheelTorqueRatio() * 0.5
      local inner = innerLeft and tv.frontL or tv.frontR
      wanted = min(cfg.tvTorqueShare * perWheel * ramp, cfg.tvMaxBrakeShare * (inner.brakeTorque or 0))
    end
  end
  local step = cfg.tvRate * dt
  local targetL = innerLeft and wanted or 0
  local targetR = innerLeft and 0 or wanted
  tv.brakeL = tv.brakeL + clampValue(targetL - tv.brakeL, -step * 3, step)
  tv.brakeR = tv.brakeR + clampValue(targetR - tv.brakeR, -step * 3, step)
  if tv.brakeL > 0.5 and not tv.frontL.isBroken then
    tv.frontL.desiredBrakingTorque = min((tv.frontL.desiredBrakingTorque or 0) + tv.brakeL, tv.frontL.brakeTorque or tv.brakeL)
  end
  if tv.brakeR > 0.5 and not tv.frontR.isBroken then
    tv.frontR.desiredBrakingTorque = min((tv.frontR.desiredBrakingTorque or 0) + tv.brakeR, tv.frontR.brakeTorque or tv.brakeR)
  end
  tv.active = max(tv.brakeL, tv.brakeR)
end

-- the game engine answers the time-of-day query of updateAutoLights through this
local function setTimeOfDay(t)
  autoLights.tod = tonumber(t)
end

-- Automatic headlights: the low beam comes on while the system is READY and it is dark by the game's time of day (from
-- autoLightsOnHour to autoLightsOffHour), and goes off again when it gets light or the system is switched off, but only if
-- this controller switched it on. When the driver uses the light switch (the state differs from what was set here), the
-- driver's choice holds until the next READY. BeamNG's time of day is 0..1 with 0 at noon (autoLightsTimeOffset 0.5 turns
-- it into a clock); the clock is published as phevClockHour.
local function updateAutoLights(dt)
  if not cfg.autoLights then
    return
  end
  autoLights.timer = autoLights.timer - dt
  if autoLights.timer <= 0 then
    autoLights.timer = cfg.autoLightsPoll
    local id = obj and obj.getID and obj:getID()
    if id and obj.queueGameEngineLua then
      obj:queueGameEngineLua("local tod = core_environment and core_environment.getTimeOfDay and core_environment.getTimeOfDay() "
        .. "local v = be:getObjectByID(" .. id .. ") "
        .. "if tod and tod.time and v then v:queueLuaCommand('local c = controller.getController(\"phev\") if c and c.setTimeOfDay then c.setTimeOfDay(' .. tostring(tod.time) .. ') end') end")
    end
  end
  local ready = module.systemReady == true
  if ready and not autoLights.lastReady then
    autoLights.override = false
  end
  autoLights.lastReady = ready
  if not autoLights.tod then
    return
  end
  local hour = ((autoLights.tod + cfg.autoLightsTimeOffset) % 1) * 24
  electrics.values.phevClockHour = hour
  local onH, offH = cfg.autoLightsOnHour, cfg.autoLightsOffHour
  local dark
  if onH > offH then
    dark = hour >= onH or hour < offH
  else
    dark = hour >= onH and hour < offH
  end
  local state = electrics.values.lights_state or 0
  if autoLights.set and state ~= autoLights.setState then
    -- the driver switched the lights: their choice holds
    autoLights.override = true
    autoLights.set = false
  end
  if autoLights.override then
    electrics.values.phevAutoLights = 0
    return
  end
  if ready and dark and state == 0 then
    if pcall(function() electrics.setLightsState(1) end) then
      autoLights.set = true
      autoLights.setState = 1
    end
  elseif autoLights.set and (not dark or not ready) then
    pcall(function() electrics.setLightsState(0) end)
    autoLights.set = false
  end
  electrics.values.phevAutoLights = autoLights.set and 1 or 0
end

-- Brake held with throttle at a standstill: returns true while the launch speed is held (see the comment in updateGFX)
local function brakeHoldLaunch(forwardGear, speedKmh, brake, rawPedal)
  local brakeHold = forwardGear and speedKmh < cfg.brakeHoldMaxKmh and brake > cfg.brakeHoldBrake and rawPedal > 0.05
    and module.launchMode == "engine"
  if brakeHold then
    local idleRPM = (module.iceIdleAV or 84) * avToRPM
    local holdRPM = mode == MODE_BOOST and cfg.brakeHoldRPMBoost or cfg.brakeHoldRPM
    idleReq.av = (idleRPM + (holdRPM - idleRPM) * clampValue(rawPedal, 0, 1)) / avToRPM
    idleReq.throttle = cfg.brakeHoldThrottle
  end
  electrics.values.phevBrakeHold = brakeHold and 1 or 0
  return brakeHold
end

local function updateGFX(dt)
  if not module or not ice then
    return
  end

  local pedal = clampValue(electrics.values.throttle or 0, 0, 1)
  -- kickdown: the driver's own pedal past the kickdown point, read before shift logic, traction control
  -- or the speed limiter reduce the throttle. Keyboard throttle always reaches 100 %: kickdown can be
  -- switched off with the "PHEV: kickdown on/off" action.
  -- Forward gears only, and never while arcade shifting swaps the pedals (reversing in arcade mode: the
  -- throttle key brakes, so input.throttle = 1 there means full braking, not kickdown).
  local driverPedal = clampValue((input and input.throttle) or pedal, 0, 1)
  local mainController = controller.mainController
  local pedalsSwapped = mainController ~= nil and mainController.isArcadeSwitched == true
  local forwardGear = (electrics.values.gearIndex or 0) >= 1
  -- keyboard throttle is only ever 0 or 1: there kickdown needs the key held floored for cfg.kbdKickdownHold s
  -- (from logged drives); an analog pedal (controller, wheel) kicks down at once past cfg.kickdownPedal
  local thrState = input and input.state and input.state.throttle
  local keyboard = thrState ~= nil and (thrState.filter == FILTER_KBD or thrState.filter == FILTER_KBD2)
  local floored = driverPedal >= cfg.kickdownPedal
  kbdFloorTimer = (floored and keyboard) and (kbdFloorTimer + dt) or 0
  kickdownActive = kickdownEnabled and forwardGear and not pedalsSwapped and floored
    and (not keyboard or kbdFloorTimer >= cfg.kbdKickdownHold)
  -- the gearbox takes kickdown from here, so strategy and box always agree
  electrics.values.phevKickdown = kickdownActive and 1 or 0
  local brake = clampValue(electrics.values.brake or 0, 0, 1) -- the driver's brake as set by the vehicle controller
  -- (math.abs: updateGFX is at LuaJIT's limit of 60 upvalues, a local alias would push it over)
  local speedKmh = math.abs(electrics.values.wheelspeed or 0) * 3.6

  -- top speed: a gentle proportional limiter over the last few km/h (BeamNG's ECU limiter PID pumps the power
  -- up and down at the limit). It lowers the requested power before the split, so the strategy sees the lower
  -- demand (in Boost the engine's spare capacity then tops up the battery). The engine reads the raw throttle,
  -- so its throttle factor is scaled with the same coefficient below.
  local rawPedal = pedal
  local overTop = math.abs(electrics.values.airspeed or 0) * 3.6 - cfg.topSpeedKmh
  local limitCoef = speedLimitSmoother:get(clampValue(cfg.topSpeedHoldCoef - overTop / cfg.topSpeedBandKmh, 0, 1), dt)
  pedal = pedal * limitCoef
  speedLimitCoef = limitCoef
  electrics.values.phevSpeedLimitCoef = limitCoef
  local shaftAV = module.outputAV1
  local soc = usableSoc()
  updateRegenSlip(dt)
  updateDeliveredEngineTorque(dt)

  -- the car starts in Hybrid (by design: the default start mode); the drive-mode selector follows (syncDriveModes).
  -- After a vehicle reset the chosen mode stays (by design: a reset keeps the chosen mode)
  local ready = module.systemReady
  if ready and not lastReady then
    if keepModeAfterReset then
      keepModeAfterReset = false
    else
      mode = MODE_HYBRID
      electrics.values.phevMode = mode
      chargeFunction = false
    end
    syncDriveModes()
  end
  lastReady = ready

  -- (only with a valid charge reading: right after spawn the battery isn't read yet)
  -- (not under water: Hybrid would start the engine there. The car then stays in Electric with an empty battery: the motor
  -- gives no drive torque, only the regeneration and the friction brakes work, and the car coasts to a stop)
  water.update(dt)
  if mode == MODE_ELECTRIC and module.socValid and soc <= cfg.evMinUsableSoc and not water.block then
    message("PHEV: battery empty, switching to Hybrid")
    setMode(MODE_HYBRID, true)
  end

  -- Electric stays Electric until the driver switches (by design). The real car leaves it on kickdown and above about
  -- 140 km/h (workshop manual); here kickdown gives the motor's full power and the motor tapers to zero towards evMaxSpeedKmh
  -- (evSpeedTaperKmh), so the car only exceeds it downhill. Only an empty battery still moves it to Hybrid (above).

  -- warm-up: an engine that started cold keeps running until it is warm (Hybrid and Boost)
  -- (counts from the moment the engine fires, also while K0 is still synchronising and closing)
  local iceActive = module.k0State == "sync" or module.k0State == "close" or module.k0State == "running"
  -- Only once per drive: after the engine has been warm it never runs (or restarts) just to keep warm; when it
  -- has cooled down later, only the normal reasons start it again.
  if not iceIsCold() then
    warmingUp = false
    warmedUp = true
  elseif iceActive and not warmedUp then
    warmingUp = true
  end

  local motorDemand = 0
  local iceThrottleFactor = 1
  local iceFactorAbsolute = false
  idleReq.av, idleReq.throttle = 0, 0
  frameDt = dt
  warmCoastTouched = false
  -- B position: stronger lift-off regeneration in every mode (forward gears)
  local decelD = bMode and cfg.coastDecelB or cfg.coastDecelD
  local decelBoost = bMode and cfg.coastDecelB or cfg.coastDecelBoost

  -- the strategy's pedal: filtered, with the coast band and lift-off hysteresis (updatePedalState)
  local drivePedal, coastCoef = updatePedalState(dt, pedal)
  updateChargeCoef(dt, drivePedal)
  local drag = engineDragTorque(dt)
  local coupled = module.iceCoupled

  if mode == MODE_ELECTRIC then
    iceWanted = false
    if drivePedal > 0 then
      motorDemand = electricDriveDemand(drivePedal, motorCurveAt) * evSpeedCoef(speedKmh)
    else
      motorDemand = regenDemand(coastCoef, brake, speedKmh, decelD, drag) + creepDemand(drivePedal, brake, speedKmh, shaftAV)
    end
    iceThrottleFactor = 0
  elseif mode == MODE_HYBRID then
    local deficit, kickdown = updateEngineRequest(dt, drivePedal, brake, speedKmh, shaftAV, false)
    if coupled then
      motorDemand, iceThrottleFactor = hybridCoupledSplit(dt, drivePedal, rawPedal, coastCoef, brake, speedKmh, shaftAV, deficit, kickdown, drag)
      iceFactorAbsolute = true
    else
      -- engine off or still starting: drive electrically (up to the Electric limit), the motor covers the start
      engineShare = 0
      fillSmoother:set(0)
      -- (full motor only with the pedal actually pressed in a forward gear: never as a drive torque in reverse)
      if kickdown and drivePedal > 0 and (electrics.values.gearIndex or 0) >= 1 then
        motorDemand = speedLimitCoef
      elseif drivePedal > 0 then
        motorDemand = electricDriveDemand(drivePedal, motorCurveAt, 'hybrid') * evSpeedCoef(speedKmh)
      else
        motorDemand = regenDemand(coastCoef, brake, speedKmh, decelD, drag) + creepDemand(drivePedal, brake, speedKmh, shaftAV)
      end
    end
  elseif mode == MODE_BOOST then
    -- the total output follows the pedal over the full system power; the engine joins eagerly and
    -- drops out on lift-off, braking and at standstill (logs: engine-off moments in all modes)
    local _, kickdown = updateEngineRequest(dt, drivePedal, brake, speedKmh, shaftAV, true)
    if coupled and (drivePedal > 0 or kickdown) then
      motorDemand, iceThrottleFactor = phevCoupledDrive(dt, drivePedal, rawPedal, shaftAV, kickdown)
      iceFactorAbsolute = true
    elseif coupled then
      motorDemand, iceThrottleFactor = coupledCoast(coastCoef, brake, speedKmh, decelBoost, shaftAV, rawPedal, drag)
      iceFactorAbsolute = true
    elseif drivePedal > 0 then
      -- engine off or still starting: the motor delivers the Boost pedal map as far as it can
      motorDemand = electricDriveDemand(drivePedal, systemTorqueAt, 'boost')
      fillSmoother:set(0)
    else
      motorDemand = regenDemand(coastCoef, brake, speedKmh, decelBoost, drag) + creepDemand(drivePedal, brake, speedKmh, shaftAV)
      fillSmoother:set(0)
    end
  end

  if not (mode == MODE_HYBRID and module.iceCoupled) then
    -- charging at a standstill only runs in the Hybrid coupled split (hybridCoupledSplit)
    standstillChargeCoef = 0
  end
  if not module.iceCoupled or not warmCoastTouched then
    -- warm-up coasting only runs in the coupled lift-off branch (coupledCoast); otherwise it is off
    warmCoastCoef = 0
  end
  electrics.values.phevWarmCoast = warmCoastCoef
  electrics.values.phevStandstillCharge = standstillChargeCoef

  -- Brake held with throttle at a standstill in a forward gear (engine running or starting): the DCT keeps its clutch
  -- open (shift logic) and the engine is held at a launch speed, brakeHoldRPM (Boost: brakeHoldRPMBoost) scaled with the
  -- pedal, through its idle controller; the motor gives no drive torque into the open clutch. On brake release the
  -- clutch closes from that speed and kickdown gives the motor's full torque. Before, the motor and the engine at full
  -- throttle revved the free shaft to the limiter (6,860 rpm in logged drives), and the DCT kept the clutch
  -- open as an over-rev until the speed fell below 6,300 rpm: the car didn't move for 2-3 s after the brake was
  -- released. Real DCT: about 2,000-2,500 rpm against the brake, about 3,000-3,500 rpm with launch control (generic:
  -- the real car's DCT figures, no gearbox source).
  if brakeHoldLaunch(forwardGear, speedKmh, brake, rawPedal) then
    motorDemand = 0
    iceThrottleFactor = 0
    iceFactorAbsolute = true
  end
  module.chargeIdleAV = idleReq.av
  module.chargeIdleThrottle = idleReq.throttle

  -- no electric drive torque in P / N, nor while the DCT holds its clutch open for a brake + throttle
  -- launch in electric operation (the motor would only spin up)
  local gearIndex = electrics.values.gearIndex or 0
  if motorDemand > 0 and (gearIndex == 0 or (module.launchMode == "electric" and brake > 0.5 and speedKmh < 2)) then
    motorDemand = 0
  end




  updateRegenLights()

  -- the engine reads the raw throttle: scale its factor like the requested power (Boost sets it absolutely)
  if not iceFactorAbsolute then
    iceThrottleFactor = iceThrottleFactor * limitCoef
  end

  -- Cold engine: its load is limited by coolant temperature (measured on the real car: about 55 % of full torque below 40 C,
  -- rising to full torque at 80 C); the motor fills the rest through the normal fill logic, battery permitting.
  -- Kickdown always gets full engine torque, also with a cold engine (the real car's behaviour).
  local warmCap = kickdownActive and 1 or interpolate(cfg.warmUpTorqueCap, iceCoolant())
  if warmCap < 1 then
    iceThrottleFactor = min(iceThrottleFactor, warmCap / max(rawPedal, 0.01))
  end
  electrics.values.phevWarmUpTorqueCap = warmCap

  blendBrakes(brake)
  updateEmergencyStop(dt, brake)
  updateAutoLights(dt)


  -- The car pulls away electrically: a stopped engine is only started once the shaft turns fast enough for its
  -- idle in the current gear (about 8 km/h in 1st). Kickdown and charging at standstill start it at once.
  local engineStopped = module.k0State == "off"
  local shaftReady = (module.outputAV1 or 0) >= (module.iceIdleAV or 84) * cfg.startShaftIdleRatio
  local startAllowed = not engineStopped or shaftReady or kickdownActive or standstillChargeWanted or chargeFunction and speedKmh < 1
  module.iceRequested = iceWanted and mode ~= MODE_ELECTRIC and startAllowed and not water.block
  module.motorDemand = clampValue(motorDemand, -1, 1)
  module.iceThrottleFactor = iceThrottleFactor
  module.systemCapActive = true

  electrics.values.phevMode = mode
  electrics.values.phevHoldTarget = (chargeFunction and cfg.chargeTarget or holdTarget) * 100
  electrics.values.phevChargeFunction = chargeFunction and 1 or 0
  electrics.values.phevEngineShare = engineShare
  electrics.values.phevWarmingUp = warmingUp and 1 or 0
  electrics.values.phevKickdown = kickdownActive and 1 or 0
  electrics.values.phevKickdownEnabled = kickdownEnabled and 1 or 0
  electrics.values.phevBMode = bMode and 1 or 0
  electrics.values.phevTvNm = tv.active
end

------------------------------------------------------------------------------------------------

local function readConfig(jbeamData)
  jbeamData = jbeamData or {}
  cfg = {
    evMaxSpeedKmh = jbeamData.evMaxSpeedKmh or 140,
    evSpeedTaperKmh = jbeamData.evSpeedTaperKmh or 3,
    evMinUsableSoc = jbeamData.evMinUsableSoc or 0.0,
    -- no engine start while the 1.4's intake is under water (default on; false switches it off)
    hydrolockProtection = jbeamData.hydrolockProtection ~= false,
    waterBlockHold = jbeamData.waterBlockHold or 1.5,
    holdFloor = jbeamData.holdFloor or 0.03,
    chargeTarget = jbeamData.chargeTarget or 0.8,

    -- Hybrid engine start: requested wheel power by speed {km/h, kW}; between the 25th percentile and the
    -- median of the peak wheel power right after real engine starts in the logs of the real car
    -- Hybrid / Boost with the engine off: the coupled pedal map from this speed, blended in over the next km/h
    coupledMapFromKmh = jbeamData.coupledMapFromKmh or 15,
    coupledMapBlendKmh = jbeamData.coupledMapBlendKmh or 15,
    hybridOnPower = jbeamData.hybridOnPowerKW or {{0, 8}, {50, 10}, {70, 12}, {90, 16}, {110, 22}, {130, 28}, {150, 34}},
    hybridLowSocFactor = jbeamData.hybridLowSocFactor or 0.7,
    socMarginForPower = jbeamData.socMarginForPower or 0.05,
    -- low-pass on the engine's net torque that the motor fill works against (s)
    fillEngineTorqueTau = jbeamData.fillEngineTorqueTau or 0.2,
    hybridOffPowerRatio = jbeamData.hybridOffPowerRatio or 0.5,
    hybridEngineSpeedKmh = jbeamData.hybridEngineSpeedKmh or 130,
    -- Boost start / stop power [km/h, kW] (a number is a flat value): electric driving at town speeds (real car: Boost
    -- typically drives electrically at 50 km/h), the engine kept on at nearly any demand above that
    boostOnPowerKW = jbeamData.boostOnPowerKW or 4,
    boostOffPowerKW = jbeamData.boostOffPowerKW or 1.5,
    kickdownPedal = jbeamData.kickdownPedal or 0.95,
    kbdKickdownHold = jbeamData.kbdKickdownHold or 2.0,
    deficitStart = jbeamData.deficitStart or 0.03,
    standstillChargeDeficit = jbeamData.standstillChargeDeficit or 0.05,
    -- logs: the engine stops at about 0 kW wheel power, engine-off spells last a median 7 s
    liftOffDelay = jbeamData.liftOffDelay or 1.0,
    -- engine stop after a lift-off or braking, by mode and road speed {km/h, s} (measured on the real car)
    liftOffDelayHybrid = jbeamData.liftOffDelayHybrid or {{0, jbeamData.liftOffDelay or 1.0}, {60, 1.5}, {100, 3}, {140, 5}, {180, 6}},
    liftOffDelayBoost = jbeamData.liftOffDelayBoost or {{0, jbeamData.liftOffDelay or 1.0}, {60, 2}, {100, 4}, {140, 12}, {180, 14}},
    lowDemandOffTime = jbeamData.lowDemandOffTime or 2,
    minOnTime = jbeamData.minOnTime or 3,
    minOffTime = jbeamData.minOffTime or 1.5,
    -- logs: after a cold start the engine first stops at a median 84 C coolant; 95 % of all stops
    -- happen at 71 C or more, only 3 % below 70 C
    warmUpTemperature = jbeamData.warmUpTemperature or 70,

    -- Hybrid split: the engine alone up to this net torque (Nm at K0; logs: boost from about 160 Nm before
    -- friction), above it the motor fills this share of the rest (logs: about 0.15 % raw charge per km)
    hybridEngineTorque = jbeamData.hybridEngineTorque or 130,
    hybridFillShare = jbeamData.hybridFillShare or 0.5,
    -- cruise assist below that: the motor takes this share of the demand between hybridAssistTorque and
    -- hybridEngineTorque (Nm at K0), faded in by road speed {km/h, 0..1} (real car: 2.4 kWh + 6.7 L/100 km at 130 km/h)
    hybridAssistTorque = jbeamData.hybridAssistTorque or 82,
    hybridAssistShare = jbeamData.hybridAssistShare or 0.27,
    hybridAssistSpeed = jbeamData.hybridAssistSpeedKmh or {{100, 0}, {120, 1}},

    sustainTorque = jbeamData.sustainTorque or 18,
    sustainBand = jbeamData.sustainBand or 0.005,
    chargeTorque = jbeamData.chargeTorque or 60,
    -- Charge mode: the charging torque tapers over the last chargeTaperBand of usable charge below the target, down
    -- to chargeTaperMin of it (it never stops charging below the target)
    chargeTaperBand = jbeamData.chargeTaperBand or 0.05,
    chargeTaperMin = jbeamData.chargeTaperMin or 0.3,
    -- charging at a standstill: generating torque in P / N and in a drive gear (Nm), raised idle in P / N (rpm), most
    -- idle throttle, droop below the idle target (rpm), most lift of the idle target for the load (rpm), fade-in (s),
    -- below this road speed (km/h)
    standstillChargeTorque = jbeamData.standstillChargeTorque or 60,
    standstillChargeTorqueD = jbeamData.standstillChargeTorqueD or 25,
    standstillChargeRPM = jbeamData.standstillChargeRPM or 1800,
    standstillChargeThrottle = jbeamData.standstillChargeThrottle or 0.5,
    standstillChargeDroopRPM = jbeamData.standstillChargeDroopRPM or 250,
    standstillChargeLeadMaxRPM = jbeamData.standstillChargeLeadMaxRPM or 300,
    standstillChargeRampTime = jbeamData.standstillChargeRampTime or 2,
    standstillChargeMaxKmh = jbeamData.standstillChargeMaxKmh or 3,

    creepTorque = jbeamData.creepTorque or 22,
    creepMaxSpeedKmh = jbeamData.creepMaxSpeedKmh or 7,

    -- lift-off regeneration as a deceleration in g: D (Electric and Hybrid) and Boost
    coastDecelD = jbeamData.coastDecelD or 0.03,
    -- Boost keeps at least this usable charge (0.5 = half the usable window)
    boostHoldTarget = jbeamData.boostHoldTarget or 0.5,
    -- a stopped engine starts once the shaft turns at this share of engine idle (pull-away is electric)
    startShaftIdleRatio = jbeamData.startShaftIdleRatio or 1.0,
    -- graded warm-up: below this coolant temperature the engine never stops; above it, it may stop when slowing
    -- down below warmUpReleaseKmh or at standstill (until warmUpTemperature)
    warmUpHardTemperature = jbeamData.warmUpHardTemperature or 50,
    warmUpReleaseKmh = jbeamData.warmUpReleaseKmh or 10,
    -- engine torque limit (share of full load) by coolant temperature during warm-up (measured on the real car)
    warmUpTorqueCap = jbeamData.warmUpTorqueCap or {{20, 0.55}, {40, 0.55}, {50, 0.65}, {60, 0.75}, {65, 0.85}, {75, 0.9}, {80, 1.0}},
    -- Boost: most motor torque used to top the battery up from the engine's spare capacity (Nm)
    boostChargeTorque = jbeamData.boostChargeTorque or 40,
    -- top speed (km/h, true speed), the power share the limiter allows at that speed (a car settles there when its drag takes
    -- about that share of its full power) and the band (km/h) over which the share falls from 1 to 0
    topSpeedKmh = jbeamData.topSpeedKmh or 230,
    topSpeedHoldCoef = jbeamData.topSpeedHoldCoef or 0.75,
    topSpeedBandKmh = jbeamData.topSpeedBandKmh or 2.5,
    -- lift-off deceleration in Boost, including the drag of a coupled engine (measured on the real car: 0.3-0.4 m/s2 above road
    -- load with the engine coupled in overrun)
    coastDecelBoost = jbeamData.coastDecelBoost or 0.04,
    -- pedal: low-pass time constant (s), coast band (lift-off target blended in below coastPedal, drive above it) and
    -- the pedal that ends a lift-off (hysteresis)
    pedalFilterTau = jbeamData.pedalFilterTau or 0.1,
    coastPedal = jbeamData.coastPedal or 0.03,
    coastPedalExit = jbeamData.coastPedalExit or 0.05,
    -- charging from the engine: fade-in time (s) and the smoothed deceleration beyond coasting (m/s2) over which it
    -- fades out
    chargeRampTime = jbeamData.chargeRampTime or 2.0,
    chargeDecelStart = jbeamData.chargeDecelStart or 0.1,
    chargeDecelFull = jbeamData.chargeDecelFull or 0.35,
    -- Boost: motor boost above the engine's full load, faded in with the pedal up to kickdown with this exponent
    boostExponent = jbeamData.boostExponent or 2,
    drivelineEfficiency = jbeamData.drivelineEfficiency or 0.95,
    -- hand-over from regeneration to friction by road speed
    regenFadeStartKmh = jbeamData.regenFadeStartKmh or 10,
    regenFadeEndKmh = jbeamData.regenFadeEndKmh or 5,
    -- front-wheel slip under regeneration (updateRegenSlip): a driven wheel slower than the ground speed by more
    -- than this share (and at least regenSlipMinDiff m/s) cuts regeneration; it returns regenSlipHoldTime s after
    -- the slip ended, ramping up over regenSlipRecoverTime s
    regenSlipRatio = jbeamData.regenSlipRatio or 0.15,
    regenSlipMinDiff = jbeamData.regenSlipMinDiff or 1.0,
    regenSlipMinSpeed = jbeamData.regenSlipMinSpeed or 1.0,
    regenSlipHoldTime = jbeamData.regenSlipHoldTime or 0.5,
    regenSlipRecoverTime = jbeamData.regenSlipRecoverTime or 1.0,
    vehicleMassKg = jbeamData.vehicleMassKg or 1800,
    -- brake lights from regenerative deceleration (m/s2, UN R13-H) and the emergency stop signal (UN R48)
    regenLightOnDecel = jbeamData.regenLightOnDecel or 1.3,
    regenLightOffDecel = jbeamData.regenLightOffDecel or 0.7,
    essMinKmh = jbeamData.essMinKmh or 50,
    essOnDecel = jbeamData.essOnDecel or 6.0,
    essOffDecel = jbeamData.essOffDecel or 2.5,
    essFlashHz = jbeamData.essFlashHz or 4,
    essHazardAtStop = jbeamData.essHazardAtStop ~= false,
    essHazardKmh = jbeamData.essHazardKmh or 10,
    -- B position: lift-off deceleration (g)
    coastDecelB = jbeamData.coastDecelB or 0.14,
    -- brake torque vectoring (updateWheelsIntermediate): on / off, speed range (km/h), lateral acceleration ramp (m/s2), share of the drive
    -- torque per front wheel braked on the inner wheel, most share of its brake torque, rate (Nm/s), rear track (m)
    tvEnabled = jbeamData.tvEnabled ~= false,
    tvMinKmh = jbeamData.tvMinKmh or 15,
    tvMaxKmh = jbeamData.tvMaxKmh or 160,
    tvMinLatAccel = jbeamData.tvMinLatAccel or 3,
    tvFullLatAccel = jbeamData.tvFullLatAccel or 7,
    tvTorqueShare = jbeamData.tvTorqueShare or 0.3,
    tvMaxBrakeShare = jbeamData.tvMaxBrakeShare or 0.12,
    tvRate = jbeamData.tvRate or 1500,
    tvTrack = jbeamData.tvTrack or 1.55,
    -- final drive: the powertrain device that carries the ratio (the reference car's front torsion reactor; "differential_R" on a
    -- longitudinal rear-wheel-drive car) or a fixed ratio, which wins; and the axle the motor regenerates on, see blendBrakes
    finalDriveDevice = jbeamData.finalDriveDevice or "torsionReactorF",
    finalDriveRatio = jbeamData.finalDriveRatio,
    -- "driven": regeneration replaces the friction brake of the driven axle only (the brake balance stays); "all": all four
    -- brakes are lowered by the same share
    regenAxle = jbeamData.regenAxle or "driven",
    -- the share of the estimated regeneration torque that is always counted as delivered (regenWheelTorque)
    regenMeasureFloor = jbeamData.regenMeasureFloor or 0.5,
    -- automatic headlights: on / off, the hours of darkness (on at autoLightsOnHour, off at autoLightsOffHour), the
    -- time-of-day poll (s) and the offset that turns BeamNG's time of day into a clock (0 = noon)
    autoLights = jbeamData.autoLights ~= false,
    autoLightsOnHour = jbeamData.autoLightsOnHour or 18.5,
    autoLightsOffHour = jbeamData.autoLightsOffHour or 6.5,
    autoLightsPoll = jbeamData.autoLightsPoll or 5,
    autoLightsTimeOffset = jbeamData.autoLightsTimeOffset or 0.5,
    -- brake held with throttle at a standstill: launch speed (rpm, Hybrid / Boost, at full pedal), the most idle throttle,
    -- the brake input and road speed below which it applies
    brakeHoldRPM = jbeamData.brakeHoldRPM or 2300,
    brakeHoldRPMBoost = jbeamData.brakeHoldRPMBoost or 2500,
    brakeHoldThrottle = jbeamData.brakeHoldThrottle or 0.6,
    brakeHoldBrake = jbeamData.brakeHoldBrake or 0.3,
    brakeHoldMaxKmh = jbeamData.brakeHoldMaxKmh or 2,
    -- warm-up coasting: engine power into the battery (kW), most engine torque (Nm), fade-in (s), the idle target's lead
    -- over the engine speed (rpm), the usable charge up to which it charges
    warmCoastPowerKW = jbeamData.warmCoastPowerKW or 12,
    warmCoastMaxTorque = jbeamData.warmCoastMaxTorque or 80,
    warmCoastRampTime = jbeamData.warmCoastRampTime or 1.0,
    warmCoastLeadRPM = jbeamData.warmCoastLeadRPM or 300,
    warmCoastMaxSoc = jbeamData.warmCoastMaxSoc or 0.97
  }
end

local function init(jbeamData)
  readConfig(jbeamData)
  -- the hybrid module takes over the "mainEngine" lookup during powertrain validation
  module = powertrain.getDevice("mainEngine")
  if not module or module.type ~= "phevModule" then
    module = powertrain.getDevice("phevModule")
  end
  if not module or module.type ~= "phevModule" then
    log("E", "phev.init", "No phevModule found, controller disabled")
    module = nil
    return
  end
  ice = module.parent
  gearbox = powertrain.getDevice("gearbox")
  local finalDrive = powertrain.getDevice(cfg.finalDriveDevice)
  finalDriveRatio = cfg.finalDriveRatio or (finalDrive and finalDrive.gearRatio) or 3.75
  if not cfg.finalDriveRatio and not finalDrive and cfg.finalDriveDevice ~= "torsionReactorF" then
    log("W", "phev.init", "Final drive device " .. tostring(cfg.finalDriveDevice) .. " not found, using the ratio 3.75")
  end
  brakeDevices = {}
  for _, group in ipairs({wheels and wheels.wheels or {}, wheels and wheels.rotators or {}}) do
    for _, wd in pairs(group) do
      if type(wd) == "table" and type(wd.brakeTorque) == "number" and wd.brakeTorque > 0 then
        table.insert(brakeDevices, wd)
      end
    end
  end
  -- the driven wheels (powertrain marks them isPropulsed while building the tree); front wheels as a fallback
  drivenWheels = {}
  local frontWheels = {}
  for _, wd in pairs(wheels and wheels.wheels or {}) do
    if type(wd) == "table" then
      if wd.isPropulsed then
        table.insert(drivenWheels, wd)
      elseif type(wd.name) == "string" and wd.name:sub(1, 1) == "F" then
        table.insert(frontWheels, wd)
      end
    end
  end
  if #drivenWheels == 0 then
    drivenWheels = frontWheels
  end
  -- brake devices of the regeneration axle (blendBrakes, regenDemand)
  drivenBrakeDevices = {}
  for _, wd in ipairs(brakeDevices) do
    for _, driven in ipairs(drivenWheels) do
      if wd == driven then
        table.insert(drivenBrakeDevices, wd)
        break
      end
    end
  end
  regenBrakeDevices = cfg.regenAxle ~= "all" and drivenBrakeDevices or brakeDevices
  axleBrakeScale = 1
  -- brake torque vectoring: the wheels by axle and side (names FL / FR / RL / RR in the host car's parts)
  tv.frontL, tv.frontR, tv.rearL, tv.rearR = nil, nil, nil, nil
  tv.brakeL, tv.brakeR, tv.active = 0, 0, 0
  for _, wd in pairs(wheels and wheels.wheels or {}) do
    if type(wd) == "table" and type(wd.name) == "string" then
      local n = wd.name
      if n == "FL" then tv.frontL = wd elseif n == "FR" then tv.frontR = wd
      elseif n == "RL" then tv.rearL = wd elseif n == "RR" then tv.rearR = wd end
    end
  end
  regenSlipHold = 0
  regenSlipCoef = 1
  coastWheelTorque = 0
  pedalFiltered = 0
  coasting = true
  chargeCoef = 0
  accelSmoothed = 0
  lastSpeedMs = nil
  k0DragFiltered = 0
  -- display names of the modes and of the charge function (a host car sets modeLabels = [electric, hybrid, boost] and
  -- chargeLabel in its data; the drive-mode part and the key actions keep the internal names)
  local labels = type(jbeamData.modeLabels) == "table" and jbeamData.modeLabels or {}
  modeNames[MODE_ELECTRIC] = labels[1] or "Electric"
  modeNames[MODE_HYBRID] = labels[2] or "Hybrid"
  modeNames[MODE_BOOST] = labels[3] or "Boost"
  chargeName = jbeamData.chargeLabel or "Charge mode"
  -- vehicle mass and driven-wheel radius turn the lift-off deceleration target into wheel torque
  vehicleMass = cfg.vehicleMassKg
  local okMass, mass = pcall(function()
    local sum = 0
    for _, n in pairs(v.data.nodes) do
      sum = sum + obj:getNodeMass(n.cid)
    end
    return sum
  end)
  if okMass and type(mass) == "number" and mass > 500 and mass < 5000 then
    vehicleMass = mass
  end
  wheelRadius = 0.334
  for _, wd in pairs(wheels and wheels.wheels or {}) do
    if type(wd) == "table" and type(wd.radius) == "number" and wd.radius > 0.2 and wd.radius < 0.6 then
      wheelRadius = wd.radius
      break
    end
  end
  fillSmoother = newTemporalSmoothing(4, 6)
  speedLimitCoef = 1
  standstillChargeCoef = 0
  module.chargeIdleAV = 0
  idleReq.av, idleReq.throttle = 0, 0
  warmCoastCoef = 0
  bMode = false
  lights.regenOn, lights.ess, lights.essTimer, lights.hazard, lights.lastAir, lights.decel = false, false, 0, false, nil, 0
  autoLights.timer, autoLights.set, autoLights.override, autoLights.lastReady = 0, false, false, false
  speedLimitSmoother = newTemporalSmoothing(0.6, 0.6, nil, 1)

  water.block, water.timer = false, 0
  deliveredEngineTorque, engineTorqueFiltered, kbdFloorTimer, kickdownActive = 0, nil, 0, false
  mode = MODE_HYBRID
  holdTarget = cfg.holdFloor
  chargeFunction = false
  iceWanted = module.iceRequested or false
  iceOnTimer = 0
  iceOffTimer = 10
  lowDemandTimer = 0
  coastTimer = 0
  standstillTimer = 0
  warmingUp = false
  warmedUp = false
  engineShare = 1
  lastReady = false
  electrics.values.phevMode = mode
end

local function reset(jbeamData)
  sustainCharging = false
  local keptMode = mode
  init(jbeamData)
  -- a reset keeps the driver's mode (with its hold target and engine request); a new start begins in Hybrid
  setMode(keptMode, true)
  keepModeAfterReset = true
end

M.init = init
M.reset = reset
M.updateGFX = updateGFX
M.updateWheelsIntermediate = updateWheelsIntermediate
-- brake torque vectoring on / off at run time (a switch for other controllers and the console)
M.setTorqueVectoring = function(on) if cfg then cfg.tvEnabled = on and true or false end end
M.toggleElectricMode = toggleElectricMode
M.toggleBoostMode = toggleBoostMode
M.toggleChargeMode = toggleChargeMode
M.cycleMode = cycleMode
M.setMode = setMode
M.getMode = getMode
M.setParameters = setParameters
M.toggleKickdown = toggleKickdown
M.toggleBMode = toggleBMode
M.setTimeOfDay = setTimeOfDay

return M
