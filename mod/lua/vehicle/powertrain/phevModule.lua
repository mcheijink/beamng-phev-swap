-- phevModule.lua
-- P2 hybrid module for the Plug-in Hybrid Swap mod.
--
-- One powertrain device that models the hybrid module of the dual-clutch gearbox:
--   * K0, the separating clutch between the 1.4 turbo and the electric motor
--   * the permanent-magnet electric motor (85 kW / 330 Nm)
--   * the gearbox input shaft that carries the motor rotor
--
-- Tree position:  mainEngine (combustionEngine, 1.4 turbo) -> phevModule (this device) -> gearbox (dctGearbox)
--
-- The 1.4 turbo keeps the name "mainEngine" so every original part of the host car (exhaust, radiator, intake,
-- engine block, drive select) configures it as usual. After the powertrain is built this device takes over
-- the "mainEngine" lookup, so BeamNG's vehicle controller, the DCT shift logic and the gauges talk to the
-- hybrid system as "the engine": they see the input-shaft speed, a combined system torque curve, the
-- system's ready state and a minimum speed (idleAV) that depends on whether the engine is coupled.
-- To the combustion engine this device is a clutch-like child: its torqueDiff is the K0 torque.
--
-- The operating strategy (when to run the engine, how much motor torque) lives in the controller
-- phev.lua. This device executes it and owns the physics, the battery accounting and the
-- engine start sequence (motor drags the engine up through slipping K0, K0 opens, the engine
-- revs to the shaft speed, K0 closes).
--
-- Original code written for this add-on. License: MIT, (c) 2026 mcheijink.

local M = {}

M.outputPorts = {[1] = true}
M.deviceCategories = {engine = true, clutchlike = true}
M.requiredExternalInertiaOutputs = {1}

local min = math.min
local max = math.max
local abs = math.abs
local floor = math.floor

local rpmToAV = 0.10471975511965977
local avToRPM = 9.549296585513721

-- engine / K0 states as numbers for the monitor app
local stateCodes = {off = 0, crank = 1, sync = 2, close = 3, running = 4, stop = 5}

local function clampValue(x, lo, hi)
  if x < lo then
    return lo
  elseif x > hi then
    return hi
  end
  return x
end

local function signOf(x)
  if x > 0 then
    return 1
  elseif x < 0 then
    return -1
  end
  return 0
end

local function moveTowards(current, target, step)
  if current < target then
    return min(current + step, target)
  end
  return max(current - step, target)
end

------------------------------------------------------------------------------------------------
-- Motor characteristics
------------------------------------------------------------------------------------------------

-- Maximum drive torque of the motor at a given shaft speed (constant torque, then constant power).
local function motorDriveTorqueAt(device, av)
  local absAV = abs(av)
  if absAV >= device.motorMaxAV then
    return 0
  end
  local torque = device.motorPeakTorque
  if absAV > 1 then
    torque = min(torque, device.motorPeakPower / absAV)
  end
  local taperStartAV = device.motorMaxAV - device.motorTaperAV
  if absAV > taperStartAV then
    torque = torque * (device.motorMaxAV - absAV) / max(device.motorTaperAV, 1e-3)
  end
  return torque
end

-- Motor and inverter losses (W) at a shaft torque and speed, the same in drive and regeneration (motorLossMap).
-- A generic loss model of a permanent-magnet synchronous machine with its IGBT inverter:
--   * inverter: a constant part while torque is controlled (gate drive, DC link, switching at small current), faded
--     in over the first motorLossConstTorque Nm so a motor held at zero torque costs nothing, plus conduction and
--     switching losses in proportion to the phase current (current ~ torque)
--   * copper: phase current squared (~ torque squared)
--   * iron under load: rises with speed^1.5 (hysteresis and eddy currents) and with the flux the load adds
--   * field weakening: the extra d-axis current above motorFieldWeakRPM
-- The no-load spin loss (magnet iron loss and bearings) is a drag on the shaft (motorSpinLoss), paid by whatever turns
-- the shaft, as with the inverter idle. The constants are fitted so that the combined map has the shape of published
-- maps of this class (ORNL benchmarks, e.g. report ORNL/TM-2010/253): about 93-95 % at 50-200 Nm and
-- 2,000-5,000 rpm, about 85-88 % at light load or at full torque around 1,000 rpm, 70-75 % at full torque near standstill.
local function motorLoss(device, torque, av)
  local t = abs(torque)
  if t <= 0 then
    return 0
  end
  local w = abs(av)
  local loss = device.motorLossConst * min(t / device.motorLossConstTorque, 1) + device.motorLossInverter * t
    + device.motorLossCopper * t * t
  local x = w / device.motorBaseAV
  loss = loss + device.motorLossIron * x * math.sqrt(x) * min(t / device.motorPeakTorque, 1)
  if w > device.motorFieldWeakAV then
    local f = min((w - device.motorFieldWeakAV) / max(device.motorMaxAV - device.motorFieldWeakAV, 1), 1)
    loss = loss + device.motorLossFieldWeak * f * f
  end
  return loss
end

-- Electrical power at the inverter's DC side (W, + = from the battery) for a motor torque at a shaft speed.
local function motorElectricPower(device, torque, av)
  local mech = torque * av
  if not device.motorLossMap then
    return mech > 0 and mech / device.motorEfficiency or mech * device.motorEfficiency
  end
  return mech + motorLoss(device, torque, av)
end

-- Battery: open-circuit voltage at the current raw charge (cells x cell table) and the chemical power (W, what leaves
-- the stored energy) for a terminal power: P_chem = OCV x I with I from P_term = OCV x I - R x I^2. Charging (P < 0)
-- stores less than arrives, discharging takes more than is delivered.
local function batteryChemicalPower(device, terminalPower)
  local r = device.batteryResistanceNow or device.batteryResistance
  local u = device.batteryOCVNow
  if r <= 0 or u <= 0 then
    device.batteryCurrent = u > 0 and terminalPower / u or 0
    return terminalPower
  end
  -- beyond the pack's matched-load power the current would be undefined: hold it there (not reached at 90 kW)
  local disc = max(u * u - 4 * r * terminalPower, 0.25 * u * u)
  local current = (u - math.sqrt(disc)) / (2 * r)
  device.batteryCurrent = current
  return u * current
end

-- Maximum regenerative torque at a given shaft speed, limited by the regen power the battery accepts.
local function motorRegenTorqueAt(device, av)
  local absAV = abs(av)
  if absAV < device.regenMinAV then
    return 0
  end
  local torque = motorDriveTorqueAt(device, av)
  -- the regeneration limit, lowered by the battery's live charge limit (charge, temperature: updateBatteryLimits)
  local regenLimit = min(device.regenPowerLimit, device.chargeLimitNow or device.regenPowerLimit)
  local mechanicalRegenPower
  if device.motorLossMap then
    -- the losses at the limit come on top of the power into the battery
    mechanicalRegenPower = regenLimit + motorLoss(device, regenLimit / max(absAV, 1), absAV)
  else
    mechanicalRegenPower = regenLimit / device.motorEfficiency
  end
  torque = min(torque, mechanicalRegenPower / max(absAV, 1))
  -- fade regen out towards low shaft speed so the shaft never gets dragged below idle
  local fadeEndAV = device.regenMinAV * 1.5
  if absAV < fadeEndAV then
    torque = torque * (absAV - device.regenMinAV) / max(fadeEndAV - device.regenMinAV, 1e-3)
  end
  return torque
end

------------------------------------------------------------------------------------------------
-- Battery
------------------------------------------------------------------------------------------------

local function getBattery(device)
  if not device.batteryName then
    return nil
  end
  return energyStorage.getStorage(device.batteryName)
end

-- raw (gross) state of charge 0..1
local function getRawSoc(device)
  local storage = getBattery(device)
  if storage and storage.energyCapacity and storage.energyCapacity > 0 then
    return storage.storedEnergy / storage.energyCapacity
  end
  return 0
end

-- usable state of charge 0..1 inside the battery window (what the car displays)
local function getUsableSoc(device)
  local window = device.socMax - device.socMin
  if window <= 0 then
    return 0
  end
  return clampValue((getRawSoc(device) - device.socMin) / window, 0, 1)
end

local function updateBatteryCoefs(device)
  local usable = getUsableSoc(device)
  device.usableSoc = usable
  -- the charge is known once the battery storage exists (not in the first frames after spawn)
  device.socValid = getBattery(device) ~= nil
  device.rawSoc = getRawSoc(device)
  -- open-circuit voltage of the pack at this charge (for the internal-resistance loss in the physics step)
  local ocvTable = device.batteryCellOCV
  local raw = clampValue(device.rawSoc, 0, 1)
  local cellV = ocvTable[#ocvTable][2]
  for i = 2, #ocvTable do
    if raw <= ocvTable[i][1] then
      local a, b = ocvTable[i - 1], ocvTable[i]
      cellV = a[2] + (b[2] - a[2]) * (raw - a[1]) / max(b[1] - a[1], 1e-6)
      break
    end
  end
  device.batteryOCVNow = cellV * device.batteryCells
  local storage = getBattery(device)
  local hasBattery = storage ~= nil and storage.energyCapacity > 0
  -- taper drive power over the last 1 % of the window, taper regen over the top 1 %
  device.batteryDriveCoef = hasBattery and clampValue(usable / 0.01, 0, 1) or 0
  device.batteryRegenCoef = hasBattery and clampValue((1 - usable) / 0.01, 0, 1) or 0
end

-- Battery power limits (BMS), by charge and pack temperature, and the pack temperature itself.
--   * policy maps (share of the rated discharge / charge power): by usable charge (batteryDischargeSocMap: a taper at the
--     bottom of the window; batteryChargeSocMap: a CV-like taper towards full) and by pack temperature
--     (batteryDischargeTempMap, batteryChargeTempMap: cold and hot derating)
--   * cell voltage limits: the terminal voltage may not fall below batteryCellVmin x cells while discharging nor rise
--     above batteryCellVmax x cells while charging, with the resistance at the pack temperature (it rises in the cold:
--     exp(batteryResistanceTempCoef x (batteryResistanceRefC - T)), at least batteryResistanceMinCoef)
--   * pack temperature: one thermal mass (batteryHeatCapacity), heated by the internal-resistance loss, cooled to
--     ambient through the housing (batteryPassiveUA), the low-temperature coolant circuit from batteryCoolStartC
--     (batteryCoolantUA) and the refrigerant chiller (batteryChillerPower, ramped in from batteryChillerStartC to
--     batteryChillerFullC) while the system is on. Workshop manual: liquid-cooled pack with a heat exchanger in the A/C circuit,
--     operating range -28 to 60 C. The figures are assumed (no data from the maker).
local function interpolatePoints(points, x)
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

local function updateBatteryLimits(device, dt)
  local ambient = powertrain.currentEnvTemperatureCelsius or 20
  if not device.batteryTemp then
    device.batteryTemp = ambient + device.batteryStartOffset
  end
  if dt and dt > 0 then
    local t = device.batteryTemp
    local heat = device.batteryLossEnergy or 0
    device.batteryLossEnergy = 0
    local cooling = device.batteryPassiveUA * (t - ambient)
    if device.systemReady then
      if t > device.batteryCoolStart and t > ambient then
        cooling = cooling + device.batteryCoolantUA * (t - ambient) * clampValue((t - device.batteryCoolStart) / 3, 0, 1)
      end
      cooling = cooling + device.batteryChillerPower * clampValue((t - device.batteryChillerStart) / max(device.batteryChillerFull - device.batteryChillerStart, 0.1), 0, 1)
    end
    device.batteryTemp = t + (heat - cooling * dt) / device.batteryHeatCapacity
    device.batteryCoolingPower = cooling
  end
  local t = device.batteryTemp
  local r = device.batteryResistance * max(math.exp(device.batteryResistanceTempCoef * (device.batteryResistanceRefC - t)), device.batteryResistanceMinCoef)
  device.batteryResistanceNow = r
  local u = device.batteryOCVNow
  local usable = device.usableSoc or 0
  local discharge = device.batteryMaxDischargePower * interpolatePoints(device.batteryDischargeSocMap, usable) * interpolatePoints(device.batteryDischargeTempMap, t)
  local charge = device.batteryMaxChargePower * interpolatePoints(device.batteryChargeSocMap, usable) * interpolatePoints(device.batteryChargeTempMap, t)
  if r > 0 and u > 0 then
    local vmin = device.batteryCellVmin * device.batteryCells
    local vmax = device.batteryCellVmax * device.batteryCells
    discharge = min(discharge, max((u - vmin) / r * vmin, 0))
    charge = min(charge, max((vmax - u) / r * vmax, 0))
  end
  device.dischargeLimitNow = max(discharge, 0)
  device.chargeLimitNow = max(charge, 0)
end

local function updateEnergyUsage(device)
  local storage = getBattery(device)
  if storage and device.spentEnergy ~= 0 then
    local newEnergy = clampValue(storage.storedEnergy - device.spentEnergy, 0, storage.energyCapacity)
    if storage.setStoredEnergy then
      storage:setStoredEnergy(newEnergy)
    else
      storage.storedEnergy = newEnergy
    end
  end
  device.spentEnergy = 0
end

------------------------------------------------------------------------------------------------
-- Physics (runs at the physics rate, 2000 Hz)
------------------------------------------------------------------------------------------------

local function updateVelocity(device, dt)
  -- like engines, this device integrates its own (shaft) speed in updateTorque
end

local function updateTorque(device, dt)
  local ice = device.parent
  local iceAV = ice.outputAV1
  local shaftAV = device.outputAV1

  -- K0 separating clutch: angle spring + damper, torque limited by the commanded K0 capacity
  local k0Capacity = device.k0LockTorque * device.k0Ratio * device.k0CapacityCoef
  local avDiff = iceAV - shaftAV
  local k0Torque = 0
  if k0Capacity > 0.5 then
    local maxAngle = k0Capacity / device.k0Spring
    local angle = clampValue(device.k0Angle + avDiff * dt, -maxAngle, maxAngle)
    device.k0Angle = angle
    k0Torque = clampValue(angle * device.k0Spring + avDiff * device.k0Damp, -k0Capacity, k0Capacity)
  else
    device.k0Angle = 0
  end
  -- seen by the combustion engine as its clutch load
  device.torqueDiff = k0Torque

  -- motor torque
  local motorTorque = 0
  if device.motorActive then
    local demand = device.motorDemand
    if demand > 0 then
      motorTorque = demand * motorDriveTorqueAt(device, shaftAV) * device.batteryDriveCoef
      if device.systemCapActive then
        -- system output limit: 160 kW / 400 Nm at the gearbox input, engine share first
        local cap = min(device.systemTorqueLimit, device.gearTorqueCap or math.huge)
        if shaftAV > 1 then
          cap = min(cap, device.systemPowerLimit / shaftAV)
        end
        motorTorque = min(motorTorque, max(cap - max(k0Torque, 0), 0))
      end
    elseif demand < 0 then
      motorTorque = demand * motorRegenTorqueAt(device, shaftAV) * device.batteryRegenCoef
    end

    -- Traction / yaw control (drivingDynamics sets throttleFactor on "mainEngine") asks for less drive
    -- torque. The motor is the fast actuator (about 10 ms against about 250 ms for the engine), so it takes
    -- the whole cut first: it drops its own drive torque and, with the engine coupled, can even generate to
    -- cancel engine torque. What the motor cannot absorb is left for the engine throttle (see updateGFX).
    device.tcShortfall = 0
    if device.throttleFactor < 1 then
      local driveRequest = max(motorTorque, 0) + max(k0Torque, 0)
      local cut = (1 - device.throttleFactor) * driveRequest
      local lowest = -motorRegenTorqueAt(device, shaftAV) * device.batteryRegenCoef
      local wanted = motorTorque - cut
      if wanted < lowest then
        device.tcShortfall = lowest - wanted
        wanted = lowest
      end
      motorTorque = wanted
    end

    -- during cranking the motor also supplies the torque that drags the engine up. The crank phase lasts until
    -- K0 is open again (see updateIceStateMachine), so the K0 torque that remains while K0 opens after the
    -- engine fired, after a failed start or after an aborted start doesn't reach the wheels either
    if device.k0CrankPhase then
      if k0Torque < 0 then
        motorTorque = motorTorque - k0Torque * device.batteryDriveCoef
      else
        motorTorque = motorTorque - k0Torque * device.batteryRegenCoef
      end
    end

    -- idle hold: the motor keeps the input shaft at idle speed while the engine is not carrying it
    if device.idleHoldActive then
      local idleError = device.idleAV - shaftAV
      if idleError > 0 then
        motorTorque = motorTorque + min(idleError * device.idleHoldP, device.idleHoldMaxTorque) * device.batteryDriveCoef
      end
    end

    -- rev limits (motor maximum speed, ESC temporary limiter) and ignition cuts from the shift logic
    if shaftAV > device.motorMaxAV or shaftAV > device.tempRevLimiterAV or device.ignitionCutTime > 0 then
      motorTorque = min(motorTorque, 0)
    end

    -- synchronised DCT shift (set by phevDct.lua): the motor brings the input shaft to the new gear's
    -- speed while the slipping DCT clutch carries the drive torque. Shaft balance (the shaft alone):
    --   shaft inertia * dAV/dt = motor + K0 - clutch,  with dAV/dt = (targetAV - shaftAV) / tau
    -- The K0 torque measured here already carries the engine's torque and its inertia torque, so only the shaft's
    -- own inertia belongs in the speed term. With shaft + engine inertia in it as well (before), the speed changed
    -- about three times faster than tau, and in a coupled upshift the motor absorbed the engine's torque plus
    -- that inertia torque: -30 to -85 kW for 0.1-0.3 s, the engine reading up to about 120 kW at K0.
    -- Coupled upshift, inertia phase: the engine's torque is reduced instead, as a DCT does, so that the engine,
    -- not the motor, gives up what the slowing shaft and engine need (the motor keeps the torque it had when the
    -- phase began): combustion torque = clutch + (shaft + engine inertia) * dAV/dt - motor hold + engine friction.
    -- It is released by a ramp (syncTorqueRampRate) once the clutch locks.
    if device.syncTargetRatio then
      local child = device.clutchChildren[1]
      local targetAV = (child.outputAV1 or 0) * device.syncTargetRatio
      local rate = (targetAV - shaftAV) / max(device.syncTau, 0.002)
      local coupled = device.iceCoupled and device.k0Ratio >= 0.99
      if not device.syncWasActive then
        device.syncMotorHold = motorTorque
        device.syncDown = rate > 0
        device.syncStartAV = max(abs(shaftAV), 1)
      end
      if rate < 0 and coupled then
        local engineInertia = ice.inertia or 0
        local friction = ice.frictionTorque or 0
        -- the engine can come down only to zero combustion torque: the speed changes no faster than that allows
        -- (at least at a quarter of the nominal rate), so the motor doesn't have to absorb the rest
        local torqueFloor = device.lastClutchLoad - device.syncMotorHold + friction
        if torqueFloor > 0 then
          rate = max(rate, min(-torqueFloor / (device.inertia + engineInertia), (targetAV - shaftAV) / (4 * max(device.syncTau, 0.002))))
        end
        local wanted = device.lastClutchLoad + (device.inertia + engineInertia) * rate - device.syncMotorHold + friction
        device.syncIceLimit = max(wanted, 0)
        device.syncIceLimitSet = true
        ice.maxTorqueLimit = device.syncIceLimit
      end
      -- feed forward the clutch torque measured in the previous physics step (the slipping clutch's actual
      -- torque, including its damping), so the speed error is closed without an offset
      motorTorque = device.lastClutchLoad - k0Torque + device.inertia * rate
      -- Coupled downshift: the engine revs itself (and the shaft) up with a throttle blip through its idle controller
      -- (idleAVOverwrite = the new gear's speed, throttle capped at what the speed change needs), and the motor keeps
      -- the torque it had when the shift began (drive, or regeneration while coasting) plus at most revMatchPower
      -- for the speed match. Before, the motor alone accelerated the coupled engine (its inertia and its drag in
      -- fuel cut) within syncTau: +200-330 Nm right after -120 to -320 Nm of regeneration, the motor swinging between
      -- about -48 and +75 kW within 0.2 s (coasting and braking downshifts 5-4 to 2-1 with the engine coupled).
      local blipAV = nil
      if device.syncDown and coupled then
        local extra = device.revMatchPower / max(abs(shaftAV), 50)
        -- the motor keeps its power from the start of the shift (its torque falls as the shaft speeds up, towards what
        -- the same wheel torque needs in the lower gear), so regeneration stays within the battery's limit
        local hold = device.syncMotorHold * device.syncStartAV / max(abs(shaftAV), 1)
        motorTorque = clampValue(motorTorque, hold - extra, hold + extra)
        if targetAV > shaftAV then
          local engineInertia = ice.inertia or 0
          local iceAV = max(ice.outputAV1 or shaftAV, 1)
          local meanRate = (targetAV - shaftAV) / max(3 * device.syncTau, 0.006)
          local need = device.lastClutchLoad + (device.inertia + engineInertia) * meanRate - hold - extra + (ice.frictionTorque or 0)
          local fullGross = (ice.torqueCurve and ice.torqueCurve[floor(iceAV * avToRPM)] or 0) * (ice.intakeAirDensityCoef or 1) * (ice.forcedInductionCoef or 1)
          -- the idle controller is proportional only: aim a little past the new gear's speed, so it doesn't settle
          -- short of it (the clutch locks within 25 rpm / 1.5 %)
          blipAV = targetAV + max(0.05 * targetAV, 8)
          device.syncBlipThrottle = clampValue(need / max(fullGross, 1), 0.05, 1)
        end
      end
      if blipAV then
        ice.idleAVOverwrite = blipAV
        ice.maxIdleThrottleOverwrite = device.syncBlipThrottle
      elseif device.syncBlipAV then
        ice.idleAVOverwrite = 0
        ice.maxIdleThrottleOverwrite = 0
      end
      device.syncBlipAV = blipAV
      if motorTorque > 0 then
        motorTorque = min(motorTorque, motorDriveTorqueAt(device, shaftAV)) * device.batteryDriveCoef
      else
        -- the generator side may use the full motor curve for these short pulses (the energy is tiny)
        motorTorque = max(motorTorque, -motorDriveTorqueAt(device, shaftAV))
      end
    end

    -- after a downshift the motor returns from its synchronising torque to the strategy's demand over a ramp
    -- (syncReleaseRate) instead of in one step: the step (often 200-300 Nm, from the speed match to regeneration)
    -- set off a driveline shuffle that the coupled engine's K0 spring carried for 0.2-0.3 s
    if not device.syncTargetRatio then
      if device.syncWasActive and device.syncDown then
        device.syncReleaseTorque = device.motorTorque or motorTorque
        if device.syncBlipAV then
          device.syncBlipAV = nil
          ice.idleAVOverwrite = 0
          ice.maxIdleThrottleOverwrite = 0
        end
      end
      if device.syncReleaseTorque then
        local step = device.syncReleaseRate * dt
        local released = moveTowards(device.syncReleaseTorque, motorTorque, step)
        if abs(released - motorTorque) < 1e-6 then
          device.syncReleaseTorque = nil
        else
          device.syncReleaseTorque = released
        end
        motorTorque = released
      end
    end

    -- battery power limits
    -- (electrical power at the inverter from the loss map; the scaling is exact for the constant efficiency and
    -- converges within a few physics steps with the map)
    local mechanicalPower = motorTorque * shaftAV
    if mechanicalPower > 0 then
      local electricalPower = motorElectricPower(device, motorTorque, shaftAV)
      local dischargeLimit = device.dischargeLimitNow or device.batteryMaxDischargePower
      if electricalPower > dischargeLimit then
        motorTorque = motorTorque * dischargeLimit / electricalPower
      end
    elseif mechanicalPower < 0 then
      local electricalPower = -motorElectricPower(device, motorTorque, shaftAV)
      -- shift synchronisation pulses last a few hundredths of a second: allowed up to the discharge limit
      local chargeLimit = device.syncTargetRatio and (device.dischargeLimitNow or device.batteryMaxDischargePower)
        or (device.chargeLimitNow or device.batteryMaxChargePower)
      if electricalPower > chargeLimit then
        motorTorque = motorTorque * chargeLimit / electricalPower
      end
    end

    motorTorque = clampValue(motorTorque, -device.motorPeakTorque, device.motorPeakTorque)
  end

  -- the engine torque reduction of a coupled upshift (sync block above): ramped out once the phase is over
  if device.syncIceLimit and not device.syncIceLimitSet then
    device.syncIceLimit = device.syncIceLimit + device.syncTorqueRampRate * dt
    if device.syncIceLimit > device.syncTorqueReleaseLimit then
      device.syncIceLimit = nil
      ice.maxTorqueLimit = math.huge
    else
      ice.maxTorqueLimit = device.syncIceLimit
    end
  end
  device.syncIceLimitSet = false
  device.syncWasActive = device.syncTargetRatio ~= nil

  -- energy bookkeeping (J), positive = taken from the battery: motor and inverter (loss map), the auxiliaries while
  -- the system is on (12 V converter, climate fans, pumps), and the pack's internal resistance
  local electricPower = motorElectricPower(device, motorTorque, shaftAV)
  local terminalPower = electricPower + (device.systemReady and device.auxPower or 0)
  local chemicalPower = batteryChemicalPower(device, terminalPower)
  device.spentEnergy = device.spentEnergy + chemicalPower * dt
  -- the internal-resistance loss heats the pack (updateBatteryLimits)
  device.batteryLossEnergy = (device.batteryLossEnergy or 0) + abs(chemicalPower - terminalPower) * dt
  device.instantElectricPower = electricPower
  device.instantTerminalPower = terminalPower
  device.instantChemicalPower = chemicalPower

  -- load from the gearbox clutches
  local load = 0
  for i = 1, device.activeOutputPortCount do
    local port = device.activeOutputPorts[i]
    load = load + device.clutchChildren[port].torqueDiff
  end

  -- shaft drag, limited for stability like the stock engines do: K0 and bearings (friction), the motor's no-load spin
  -- loss (magnet iron loss) and the wet dual-clutch gearbox's drag (open-clutch shear, oil churning, pump), see shaftDragConst
  local absShaftAV = abs(shaftAV)
  local frictionTorque = device.shaftDragConst + device.shaftDragCoef * absShaftAV
  frictionTorque = min(frictionTorque, absShaftAV * device.inertia * 2000) * signOf(shaftAV)

  local newAV = shaftAV + dt * (k0Torque + motorTorque - load - frictionTorque) * device.invInertia
  for i = 1, device.activeOutputPortCount do
    local port = device.activeOutputPorts[i]
    device[device.outputTorqueNames[port]] = load
    device[device.outputAVNames[port]] = newAV
  end
  device.outputAV1 = newAV

  device.motorTorque = motorTorque
  device.k0Torque = k0Torque
  device.instantMotorPower = motorTorque * shaftAV
  device.shaftInertiaEffective = device.inertia + device.k0Ratio * (ice.inertia or 0)
  device.lastClutchLoad = load
  -- low-passed (about 50 ms) for the shift logic: the torque the box carries into a shift, without a driveline
  -- shuffle's swing at the instant the shift starts
  device.clutchLoadFiltered = device.clutchLoadFiltered + (load - device.clutchLoadFiltered) * min(1, dt / 0.05)
end

------------------------------------------------------------------------------------------------
-- Engine start / stop sequence (runs at the graphics rate)
------------------------------------------------------------------------------------------------

local function setIceSyncTarget(device, targetAV)
  local ice = device.parent
  ice.idleAVOverwrite = targetAV
  ice.maxIdleThrottleOverwrite = targetAV > 0 and device.syncMaxThrottle or 0
end

local function iceCanRun(device)
  local ice = device.parent
  return not ice.isBroken and not ice.isDisabled and ice.hasFuel ~= false
end

local function updateIceStateMachine(device, dt)
  local ice = device.parent
  local state = device.k0State
  local wanted = device.iceRequested and device.systemReady and iceCanRun(device) and device.startRetryTimer <= 0
  device.stateTimer = device.stateTimer + dt
  device.startRetryTimer = max(device.startRetryTimer - dt, 0)

  if state == "off" then
    device.k0Target = 0
    setIceSyncTarget(device, 0)
    if wanted then
      -- no 12 V starter: the engine is dragged up by the motor through K0
      ice.starterThrottleKillCoef = 1
      ice:setIgnition(1)
      device.k0State = "crank"
      device.k0CrankPhase = true
      device.stateTimer = 0
    end
  elseif state == "crank" then
    device.k0Target = device.k0CrankRatio
    setIceSyncTarget(device, 0)
    if not wanted then
      device.k0State = "stop"
      device.stateTimer = 0
    elseif ice.outputAV1 >= device.iceFireAV then
      device.k0State = "sync"
      device.stateTimer = 0
      device.syncTimer = 0
      device.syncTrimAV = 0
    elseif device.stateTimer > device.crankTimeout then
      -- failed start: give up for a while
      ice:setIgnition(0)
      device.k0State = "off"
      device.stateTimer = 0
      device.startRetryTimer = device.startRetryDelay
      device.startFailures = device.startFailures + 1
    end
  elseif state == "sync" then
    -- K0 open, the engine revs itself up to the shaft speed with its own idle controller. That controller is
    -- proportional only (throttle = P x speed error), so the engine settles short of its target by the error that
    -- gives the throttle it needs at that speed: 50-75 rpm at 2,200-2,700 rpm (in simulation: median 62 rpm, against
    -- the 40 rpm sync tolerance). Every Hybrid start at motorway speed then ran into the 2.5 s timeout while the motor
    -- drove the car alone (57 starts, 139 s, 1.3 kWh of the 3.4 kWh the simulated drive took from the battery in Hybrid).
    -- The target therefore leads the shaft by syncLeadRPM and is trimmed by the remaining speed error (integral,
    -- syncTrimGain per second, once the engine is within syncTrimRangeRPM of the shaft).
    device.k0Target = 0
    local shaftTarget = max(device.outputAV1, ice.idleAV)
    local gap = shaftTarget - ice.outputAV1
    if abs(gap) < device.syncTrimRangeAV then
      device.syncTrimAV = clampValue(device.syncTrimAV + gap * device.syncTrimGain * dt, -device.syncTrimMaxAV, device.syncTrimMaxAV)
    end
    setIceSyncTarget(device, shaftTarget + device.syncLeadAV + device.syncTrimAV)
    if not wanted then
      device.k0State = "stop"
      device.stateTimer = 0
    elseif abs(ice.outputAV1 - device.outputAV1) < device.syncToleranceAV then
      device.syncTimer = device.syncTimer + dt
      if device.syncTimer > device.syncHoldTime then
        device.k0State = "close"
        device.stateTimer = 0
      end
    else
      device.syncTimer = 0
      if device.stateTimer > device.syncTimeout then
        -- close anyway, K0 slips the remaining difference
        device.k0State = "close"
        device.stateTimer = 0
      elseif ice.outputAV1 < ice.idleAV * 0.4 and device.stateTimer > 0.5 then
        -- engine died during synchronisation
        ice:setIgnition(0)
        device.k0State = "off"
        device.stateTimer = 0
        device.startRetryTimer = device.startRetryDelay
        device.startFailures = device.startFailures + 1
      end
    end
  elseif state == "close" then
    device.k0Target = 1
    setIceSyncTarget(device, max(device.outputAV1, ice.idleAV) + device.syncLeadAV + device.syncTrimAV)
    if not wanted then
      device.k0State = "stop"
      device.stateTimer = 0
    elseif device.k0Ratio >= 0.999 then
      setIceSyncTarget(device, 0)
      device.k0State = "running"
      device.stateTimer = 0
      device.startFailures = 0
    end
  elseif state == "running" then
    device.k0Target = 1
    if device.syncBlipAV then
      -- a coupled downshift's throttle blip (updateTorque) runs through the engine's idle controller
      local ice2 = device.parent
      ice2.idleAVOverwrite = device.syncBlipAV
      ice2.maxIdleThrottleOverwrite = device.syncBlipThrottle or 0
    elseif (device.chargeIdleAV or 0) > 0 then
      -- charging at a standstill (strategy: Charge mode, Hybrid charge sustaining): the engine's idle controller
      -- holds a raised idle against the motor's generating torque
      local ice2 = device.parent
      ice2.idleAVOverwrite = device.chargeIdleAV
      ice2.maxIdleThrottleOverwrite = device.chargeIdleThrottle or 0.5
    else
      setIceSyncTarget(device, 0)
    end
    if not wanted or ice.isStalled then
      device.k0State = "stop"
      device.stateTimer = 0
    end
  elseif state == "stop" then
    device.k0Target = 0
    setIceSyncTarget(device, 0)
    if device.k0Ratio <= 0.001 then
      ice:setIgnition(0)
      device.k0State = "off"
      device.stateTimer = 0
    end
  end

  -- K0 actuation: closing and opening ramps
  local rate = device.k0Target > device.k0Ratio and device.k0CloseRate or device.k0OpenRate
  if device.k0State == "crank" then
    rate = device.k0CrankRate
  end
  device.k0Ratio = moveTowards(device.k0Ratio, device.k0Target, rate * dt)
  -- The crank phase (K0 capacity limited to the crank torque, the motor supplies the K0 torque) lasts until K0
  -- is open again. Ending it with the "crank" state would let the capacity jump from the crank torque to the
  -- full K0 torque at the still partly closed ratio (about 80 to 180 Nm) while K0 opens, with nothing to
  -- compensate it: a tug at every engine start and failed start. K0 closing for real ("close") always ends it.
  if device.k0CrankPhase and device.k0State ~= "crank" and (device.k0Ratio <= 0.001 or device.k0State == "close" or device.k0State == "running") then
    device.k0CrankPhase = false
  end
  device.k0CapacityCoef = device.k0CrankPhase and (device.crankTorque / device.k0LockTorque) / max(device.k0CrankRatio, 0.01) or 1

  device.iceCoupled = device.k0State == "running"
end

------------------------------------------------------------------------------------------------
-- Graphics-rate update
------------------------------------------------------------------------------------------------

-- Layout for BeamNG's Powertrain UI app, once the vehicle is built:
--  * the app keys devices by name, so the 1.4 turbo (still named "mainEngine" from its part, like this module after
--    the lookup takeover) is renamed; its name is only needed while the vehicle is built (exhaust links)
--  * this module shows as the electric motor, placed between the engine and the DCT; the DCT, shafts and
--    differential get positions along the real transverse layout instead of floating freely
local function setupVisualLayout(device)
  local ice = device.parent
  if ice and ice.name == device.name then
    ice.name = "iceEngine"
  end
  device.visualType = "electricMotor"

  local function lerp(a, b, t)
    return {x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t, z = a.z + (b.z - a.z) * t}
  end
  local e = ice and ice.visualPosition
  local fl, fr, rl
  for _, d in pairs(powertrain.getDevices()) do
    if d.visualType == "wheel" and d.visualPosition and type(d.name) == "string" then
      if d.name:find("FL") then
        fl = d.visualPosition
      elseif d.name:find("FR") then
        fr = d.visualPosition
      elseif d.name:find("RL") then
        rl = d.visualPosition
      end
    end
  end
  if e and fl and fr then
    local axle = lerp(fl, fr, 0.5)
    -- schematic, not to scale: engine, motor and DCT in a row ahead of the front axle (the app squeezes
    -- the real positions into the front of a whole-car view), the differential slightly behind the axle
    local forward = (rl and rl.y > axle.y) and -1 or 1
    e = {x = e.x, y = axle.y + forward * 0.7, z = e.z}
    ice.visualPosition = e
    -- the DCT sits on the side of the engine facing the farther front wheel (transverse engine)
    local side = (abs(fl.x - e.x) > abs(fr.x - e.x)) and fl or fr
    local gbPos = {x = e.x + (side.x - e.x) * 0.6, y = e.y, z = e.z}
    device.visualPosition = lerp(e, gbPos, 0.5)
    local gearbox = device.clutchChildren[1]
    if gearbox and gearbox.outputPorts then
      gearbox.visualPosition = gbPos
      local chain = {}
      local d = gearbox
      while d and d.children and #d.children == 1 and d.type ~= "differential" do
        d = d.children[1]
        chain[#chain + 1] = d
      end
      local diffTarget = {x = axle.x + (gbPos.x - axle.x) * 0.3, y = axle.y - forward * 0.25, z = axle.z}
      for i, cd in ipairs(chain) do
        if cd.visualType ~= "wheel" then
          cd.visualPosition = lerp(gbPos, diffTarget, i / #chain)
        end
      end
      local diff = chain[#chain]
      if diff and diff.children and diff.visualPosition then
        for _, axleShaft in ipairs(diff.children) do
          local spindle = axleShaft.children and axleShaft.children[1]
          if spindle and spindle.visualPosition then
            axleShaft.visualPosition = lerp(diff.visualPosition, spindle.visualPosition, 0.5)
          end
        end
      end
    end
  end
  if powertrain.sendDeviceTree then
    powertrain.sendDeviceTree()
  end
end

-- linear interpolation in a {{x, y}, ...} table, clamped at the ends (the guarded helper of the battery limits)
local interpolate = interpolatePoints

-- Engine temperatures. BeamNG's combustionEngineThermals heats the coolant together with a heavy engine block
-- (168 kg of aluminium, tightly coupled to the coolant) and puts all of the engine's friction (25 Nm, about
-- 7 kW at 2,600 rpm) straight into 4.35 kg of oil. Compared with the logs of the real car (engine
-- temperatures) the coolant then warms up too slowly at first, the oil warms up 2-3 times too fast and runs
-- 5-10 C too hot, and the engine-off cool-down doesn't depend on speed or weather. Corrections on top of
-- BeamNG's model, fitted with an offline simulation of BeamNG's equations driven by the logged engine speed,
-- torque and road speed of many logged trips:
--   * warm-up: while the engine runs below the thermostat (1 C below), extra coolant heat of gain x gross
--     engine power (before friction, capped). The gain falls as the coolant warms (warmUpHeatGain table):
--     a cold engine first heats the little coolant around the head, whose sensor reading rises fastest then.
--   * engine off with the system on: coolant heat loss in proportion to its excess over ambient, more at speed
--     (radiator airflow) and in cold weather (cabin heater). With the ignition off only BeamNG's own losses.
--   * oil: the thermal mass of oil and crankcase (oilMassFactor x BeamNG's oil), part of BeamNG's friction heat moved
--     to the coolant (oilFrictionShare stays in the oil), a share of the engine's net power that heats the oil once
--     the coolant is warm (pistons, turbo; oilLoadGateTemp), an oil-coolant exchanger that opens with the coolant
--     temperature (oilCoolerGateTemp), and airflow cooling of the sump. In the logs the oil hardly moves for the
--     first 150 s and then follows the coolant at up to 0.25 K/s. The correction rescales the oil temperature step
--     BeamNG made since the last update; a jump (vehicle reset, temperatures set from outside) restarts it.
local function updateEngineThermals(device, dt)
  local ice = device.parent
  local th = ice and ice.thermals
  if not th or not th.coolantTemperature or not th.oilTemperature or dt <= 0 then
    return
  end
  local ambient = powertrain.currentEnvTemperatureCelsius or 20
  local coolant = th.coolantTemperature
  local running = device.k0State == "sync" or device.k0State == "close" or device.k0State == "running"
  -- net power (through K0, or the engine's own while it isn't coupled) and gross power (plus BeamNG's
  -- static and speed-squared friction)
  local netPower = 0
  local grossPower = 0
  if running then
    netPower = (device.k0Torque or 0) * device.outputAV1
    if device.k0State ~= "running" then
      netPower = (ice.combustionTorque or 0) * (ice.outputAV1 or 0)
    end
    local iceAV = abs(ice.outputAV1 or 0)
    local friction = (ice.friction or 0) * (ice.wearFrictionCoef or 1) * (ice.damageFrictionCoef or 1)
    local dynamicFriction = (ice.dynamicFriction or 0) * (ice.wearDynamicFrictionCoef or 1) * (ice.damageDynamicFrictionCoef or 1)
    grossPower = max(netPower + (friction + dynamicFriction * iceAV) * iceAV, 0)
  end

  -- coolant: warm-up heat, engine-off loss
  local coolantPower = 0
  if running then
    if coolant < device.warmUpThermostat - 1 then
      coolantPower = min(grossPower, device.warmUpHeatMaxPower) * interpolate(device.warmUpHeatGain, coolant)
    end
  elseif (electrics.values.ignitionLevel or 2) >= 2 and coolant > ambient then
    local speedKmh = abs(electrics.values.wheelspeed or 0) * 3.6
    local lossCoef = device.coolDownLossCoef * (1 + device.coolDownLossSpeedCoef * min(speedKmh, 130) / 100)
      + device.coolDownHeaterLoss * clampValue(device.coolDownHeaterAmbient - ambient, 0, 25)
    coolantPower = -(coolant - ambient) * lossCoef
  end

  -- oil
  local oil = th.oilTemperature
  local lastOil = device.thermalOilLast
  if lastOil and device.oilHeatCapacity and abs(oil - lastOil) < 3 then
    -- BeamNG puts the engine's static friction straight into the oil; oilFrictionShare of it stays there, the rest
    -- goes to the coolant (block)
    local frictionHeat = running and (ice.friction or 0) * (ice.wearFrictionCoef or 1) * (ice.damageFrictionCoef or 1) * abs(ice.outputAV1 or 0) or 0
    local frictionToCoolant = (1 - device.oilFrictionShare) * frictionHeat
    -- load heat (pistons, turbo bearings): switched in by the coolant temperature, like oil-jet piston cooling
    local loadGate = clampValue((coolant - device.oilLoadGateTemp) / 10, 0, 1)
    local loadHeat = device.oilLoadShare * loadGate * max(netPower, 0)
    -- oil <-> coolant heat exchanger, opened by the coolant temperature (coolant held in the block while it warms up)
    local coolerGate = clampValue((coolant - device.oilCoolerGateTemp) / 10, 0, 1)
    local coolerHeat = device.oilCoolerCoef * coolerGate * (coolant - lastOil)
    local airSpeed = abs(obj:getFrontAirflowSpeed()) * 0.8
    local sumpToAir = device.oilAirCoef * airSpeed / (15 + airSpeed) * (lastOil - ambient)
    local oilEnergy = (oil - lastOil) * device.oilHeatCapacity + (loadHeat + coolerHeat - sumpToAir - frictionToCoolant) * dt
    oil = max(lastOil + oilEnergy / (device.oilHeatCapacity * device.oilMassFactor), ambient)
    th.oilTemperature = oil
    coolantPower = coolantPower - loadHeat - coolerHeat + frictionToCoolant
  end
  device.thermalOilLast = oil

  th.additionalCoolantEnergyPerStep = (th.additionalCoolantEnergyPerStep or 0) + coolantPower * dt
end

-- Fuel use. BeamNG burns gross engine work (before friction) / burnEfficiency(load) / 41.5 MJ/kg, where load is the
-- engine's torque share of its live full load and the efficiency table of the 1.4 turbo part falls from 0.49 at 70 %
-- load to 0.30 at full load. In simulation that gave 240 g/kWh at 80-160 Nm, 290-340 g/kWh at full load and
-- 450-560 g/kWh below 40 Nm. The logs of the real car give a Willans line: fuel =
-- fuelWillansSlope x indicated power (net torque + fuelIndicatedFriction) + fuelWillansOffset, with the logged
-- fuel rate's 2.3x scale error taken out at fuelScaleF; plus a knock and enrichment penalty near full load that the
-- logs can't show (assumed). The correction scales BeamNG's own fuel per frame through invBurnEfficiencyCoef, which
-- the engine resets every frame and only multiplies into the fuel (its thermals keep the table, so the heat to
-- coolant and oil is unchanged). Idle and overrun (net torque up to 0) keep BeamNG's own fuel; the correction
-- blends in up to fuelBlendTorque Nm.
local function updateFuelCalibration(device)
  local ice = device.parent
  if not device.fuelCalibration or not ice or not ice.invBurnEfficiencyTable or not device.iceCoupled then
    electrics.values.phevFuelCoef = 1
    return
  end
  local av = abs(ice.outputAV1 or 0)
  local gross = ice.lastOutputTorque or 0
  local net = ice.combustionTorque or 0
  if av < 10 or gross <= 1 or net <= 0 then
    electrics.values.phevFuelCoef = 1
    return
  end
  local inv = ice.invBurnEfficiencyTable[floor(clampValue(ice.instantEngineLoad or 0, 0, 1) * 100)] or 1
  local bngRate = gross * av * inv / device.fuelEnergyDensity -- kg/s
  if bngRate <= 0 then
    electrics.values.phevFuelCoef = 1
    return
  end
  local rpm = av * avToRPM
  local full = 0
  local data = ice.torqueData
  if data and data.curves and data.curves[data.finalCurveName] then
    full = data.curves[data.finalCurveName].torque[floor(rpm)] or 0
  end
  local x = full > 1 and net / full or 0
  local penalty = 1 + device.fuelKnockPenalty * clampValue((x - 0.7) / 0.3, 0, 1)
    + device.fuelEnrichPenalty * clampValue((x - 0.8) / 0.2, 0, 1) * clampValue((rpm - 4500) / 1500, 0, 1)
  local indicatedKW = (net + device.fuelIndicatedFriction) * av * 0.001
  local targetRate = (device.fuelWillansSlope * indicatedKW / 3600 + device.fuelWillansOffset) / device.fuelScaleF * penalty * 0.001 -- kg/s
  local w = clampValue(net / device.fuelBlendTorque, 0, 1)
  local coef = 1 + w * (targetRate / bngRate - 1)
  ice.invBurnEfficiencyCoef = (ice.invBurnEfficiencyCoef or 1) * coef
  electrics.values.phevFuelCoef = coef
end

local function updateGFX(device, dt)
  updateEngineThermals(device, dt)
  updateFuelCalibration(device)
  if not device.visualLayoutDone then
    device.visualLayoutDone = true
    setupVisualLayout(device)
  end
  local ice = device.parent

  updateEnergyUsage(device)
  updateBatteryCoefs(device)
  updateBatteryLimits(device, dt)

  if device.ignitionCoef <= 0 then
    device.systemReady = false
  end
  device.starterEngagedCoef = 0
  device.motorActive = device.ignitionCoef > 0 and device.systemReady and not device.isBroken

  updateIceStateMachine(device, dt)

  -- Launch torque in 1st gear (firstGearTorqueLimit, Nm at the gearbox input; the motor is cut back so that engine + motor
  -- stay within it), in Hybrid and Boost (electrics phevMode 1 / 2) or whenever the engine is coupled; Electric pull-away is
  -- unchanged. With the full 400 Nm in 1st the sim ran 0-100 km/h in 6.1-6.5 s against the maker's 7.4 s, while 50-100 km/h
  -- (3.9 s) already matches the real car's best logged 4.0 s: the difference is all in the launch. The first version applied
  -- the limit only with the engine coupled, so a kickdown start ran its first 1.2 s on the motor's full 330 Nm (0-100 still
  -- 6.5-6.6 s). Provisional, calibrated on the 0-100 time
  local gearIndex = electrics.values.gearIndex or 0
  local hybridMode = (electrics.values.phevMode or 0) ~= 0
  device.gearTorqueCap = (gearIndex == 1 and (device.iceCoupled or hybridMode)) and device.firstGearTorqueLimit or math.huge

  -- Minimum speed and launch mode reported to the DCT shift logic:
  --   electric: engine decoupled, the motor pulls from 0 rpm, the clutch may close at standstill
  --   engine:   engine coupled or being started, the shaft must stay at engine idle, the clutch slips
  local engineActive = device.k0State == "crank" or device.k0State == "sync" or device.k0State == "close" or device.k0State == "running"
  device.launchMode = engineActive and "engine" or "electric"
  device.idleAV = engineActive and device.iceIdleAV or 0
  device.idleRPM = device.idleAV * avToRPM

  -- the motor holds the shaft at engine idle while the engine is being started and not yet locked
  device.idleHoldActive = device.motorActive and engineActive and device.k0State ~= "running" and device.k0State ~= "close"

  -- the combustion engine only gets pedal throttle while it is coupled.
  -- Traction / yaw control: the motor takes the torque cut first (updateTorque). The engine throttle follows
  -- only when the motor can't absorb the whole cut, or when the intervention lasts longer than the engine's
  -- response time, so the motor doesn't keep generating against the engine.
  if device.throttleFactor < 1 then
    device.tcActiveTime = device.tcActiveTime + dt
  else
    device.tcActiveTime = 0
  end
  local engineCut = (device.tcShortfall or 0) > 1 or device.tcActiveTime > device.tcEngineFollowTime
  device.iceTcFactor = engineCut and device.throttleFactor or min(device.iceTcFactor + dt * 4, 1)
  electrics.values[device.iceThrottleFactorName] = device.iceCoupled and device.iceThrottleFactor * device.iceTcFactor or 0
  electrics.values.phevTcFactor = device.throttleFactor
  -- engine-like throttle values that BeamNG's traction control (slip only counts while driving) and
  -- the electric-motor gauges read from "mainEngine"
  local driveDemand = max(device.motorDemand or 0, 0)
  device.throttle = device.iceCoupled and max(device.parent.throttle or 0, driveDemand) or driveDemand
  device.regenThrottle = max(-(device.motorDemand or 0), 0)

  device.ignitionCutTime = max(device.ignitionCutTime - dt, 0)

  -- values other systems read from "the engine"
  device.outputRPM = device.outputAV1 * avToRPM
  -- the hybrid system never stalls: it is either ready (motor and engine may both stand still) or off
  device.isStalled = false
  device.revLimiterActive = (device.iceCoupled and ice.revLimiterActive) or device.outputAV1 > device.motorMaxAV
  device.thermals = ice.thermals
  local maxTorqueNow = device.systemTorqueLimit
  if device.outputAV1 > 1 then
    maxTorqueNow = min(maxTorqueNow, device.systemPowerLimit / device.outputAV1)
  end
  local shaftTorque = (device.k0Torque or 0) + (device.motorTorque or 0)
  device.instantEngineLoad = clampValue(shaftTorque / max(maxTorqueNow, 1), 0, 1)
  device.engineLoad = device.instantEngineLoad
  device.combustionTorque = shaftTorque

  -- telemetry for the controller, UI and debugging
  local motorPowerKW = (device.instantMotorPower or 0) * 0.001
  electrics.values.phevMotorTorque = device.motorTorque or 0
  electrics.values.phevMotorPowerKW = motorPowerKW
  electrics.values.phevK0Ratio = device.k0Ratio
  electrics.values.phevK0Torque = device.k0Torque or 0
  electrics.values.phevIceRPM = ice.outputAV1 * avToRPM
  electrics.values.phevIceRunning = device.iceCoupled and 1 or 0
  electrics.values.phevIceState = device.k0State
  electrics.values.phevSocUsable = device.usableSoc * 100
  electrics.values.phevSocRaw = device.rawSoc * 100
  electrics.values.phevReady = device.systemReady and 1 or 0

  -- values for the hybrid monitor UI app
  local shaftAV = device.outputAV1
  -- battery power at the terminals (inverter + auxiliaries) and what leaves the cells (+ the internal-resistance loss)
  local batteryPower = device.instantTerminalPower or 0
  local storage = getBattery(device)
  local grossKWh = storage and storage.energyCapacity and storage.energyCapacity / 3600000 or 0
  local usableCapKWh = grossKWh * (device.socMax - device.socMin)
  electrics.values.phevIceStateCode = stateCodes[device.k0State] or 0
  electrics.values.phevShaftRPM = shaftAV * avToRPM
  electrics.values.phevLaunchElectric = device.launchMode == "electric" and 1 or 0
  electrics.values.phevEnginePowerKW = (device.k0Torque or 0) * shaftAV * 0.001
  -- the engine's own net power (combustion minus friction, without the inertia torque that passes K0 while the shaft
  -- changes speed in a shift), and engine net + motor: these stay within 115 / 160 kW, while the K0 power reads up to
  -- about 230 kW for 0.1 s in a full-load upshift
  local iceNetKW = device.iceCoupled and (ice.combustionTorque or 0) * (ice.outputAV1 or 0) * 0.001 or 0
  electrics.values.phevEngineNetKW = iceNetKW
  electrics.values.phevSystemNetKW = iceNetKW + motorPowerKW
  electrics.values.phevSystemPowerKW = ((device.k0Torque or 0) + (device.motorTorque or 0)) * shaftAV * 0.001
  -- power leaving the DCT towards the wheels (after the gearbox losses)
  local gb = device.clutchChildren[1]
  electrics.values.phevWheelPowerKW = gb and (gb.outputTorque1 or 0) * (gb.outputAV1 or 0) * 0.001 or 0
  electrics.values.phevBatteryKW = batteryPower * 0.001
  electrics.values.phevBatteryChemKW = (device.instantChemicalPower or 0) * 0.001
  electrics.values.phevBatteryLossKW = ((device.instantChemicalPower or 0) - batteryPower) * 0.001
  electrics.values.phevMotorLossKW = ((device.instantElectricPower or 0) - (device.instantMotorPower or 0)) * 0.001
  electrics.values.phevBatteryCurrent = device.batteryCurrent or 0
  electrics.values.phevBatteryVoltage = (device.batteryOCVNow or 0) - (device.batteryCurrent or 0) * device.batteryResistance
  electrics.values.phevBatteryGrossKWh = grossKWh
  electrics.values.phevBatteryTempC = device.batteryTemp or 0
  electrics.values.phevBattDischargeLimitKW = (device.dischargeLimitNow or 0) * 0.001
  electrics.values.phevBattChargeLimitKW = (device.chargeLimitNow or 0) * 0.001
  electrics.values.phevBattCoolingKW = (device.batteryCoolingPower or 0) * 0.001
  electrics.values.phevBatteryUsableKWh = device.usableSoc * usableCapKWh
  electrics.values.phevBatteryUsableCapKWh = usableCapKWh
  electrics.values.phevSocWindowMin = device.socMin * 100
  electrics.values.phevSocWindowMax = device.socMax * 100
  electrics.values.phevCoolantC = ice.thermals and (ice.thermals.coolantTemperature or ice.thermals.oilTemperature) or 0
  electrics.values.phevIceThrottle = electrics.values[device.iceThrottleFactorName] or 0

  -- drives the Cherrier's vanilla power gauge (vivaceGauges.lua), same scaling
  local moving = (electrics.values.wheelspeed or 0) > 0.1
  local powerNorm = clampValue(motorPowerKW / 85, -1, 1)
  local meter = (powerNorm > 0 and powerNorm * 1.55 or powerNorm * 0.8) * (moving and 1 or 0)
  device.powerDisplay = device.powerDisplaySmoother:get(meter * 3000) + 2400
  electrics.values.powerDisplay = device.powerDisplay
end

------------------------------------------------------------------------------------------------
-- Sounds: an electric motor whine, the combustion engine plays its own sounds
------------------------------------------------------------------------------------------------

-- AVAS (acoustic vehicle alerting system): a synthetic drive sound while the car moves electrically at
-- low speed, so pedestrians hear it. Loudest at walking pace, pitch rising with speed, silent from
-- avasMaxSpeedKmh and whenever the engine is coupled.
local function updateAvasSound(device, dt)
  if not device.avasSoundID then
    return
  end
  local speedKmh = abs(electrics.values.wheelspeed or 0) * 3.6
  local gearEngaged = (electrics.values.gearIndex or 0) ~= 0
  local electric = device.motorActive and not device.iceCoupled and device.k0State ~= "close"
  local target = 0
  if electric and gearEngaged and speedKmh < device.avasMaxSpeedKmh then
    target = clampValue((device.avasMaxSpeedKmh - speedKmh) / (device.avasMaxSpeedKmh - device.avasFadeStartKmh), 0, 1)
    if speedKmh < 0.5 and (electrics.values.brake or 0) > 0.05 then
      target = target * device.avasStandstillVolume
    end
  end
  -- the sound follows the drive load: accelerating sounds fuller, slightly louder and higher than
  -- rolling along; regeneration counts as light load
  local driveLoad = clampValue((device.motorTorque or 0) / device.motorPeakTorque, 0, 1)
  local load = device.avasLoadSmoother:get(driveLoad, dt)
  local volume = device.avasVolumeSmoother:get(target, dt) * (1 + device.avasLoadVolumeGain * load)
  local rpm = device.avasRPMSmoother:get(device.avasBaseRPM + speedKmh * device.avasRPMPerKmh + load * device.avasRPMPerLoad, dt)
  local soundLoad = device.avasLoadMin + (device.avasLoadMax - device.avasLoadMin) * load
  local fundamentalFreq = sounds.hzToFMODHz(rpm * device.avasFundamentalRPMCoef)
  obj:setEngineSound(device.avasSoundID, rpm, soundLoad, fundamentalFreq, volume)
  electrics.values.phevAvasVolume = volume
  electrics.values.phevAvasLoad = load
end

local function updateSounds(device, dt)
  if device.engineSoundID then
    local rpm = device.soundRPMSmoother:get(abs(device.outputAV1 * avToRPM), dt)
    local load = clampValue(device.soundLoadSmoother:get(abs(device.motorTorque or 0) / device.motorPeakTorque, dt), device.soundMinLoadMix, device.soundMaxLoadMix)
    local fundamentalFreq = sounds.hzToFMODHz(rpm * device.fundamentalFrequencyRPMCoef)
    local volume = device.motorActive and device.engineVolumeCoef or 0
    obj:setEngineSound(device.engineSoundID, rpm, load, fundamentalFreq, volume)
  end
  updateAvasSound(device, dt)
end

local function initEngineSound(device, soundID, samplePath, engineNodeIDs, offLoadGain, onLoadGain, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].blendFile = samplePath
  obj:queueGameEngineLua(string.format("core_sounds.initEngineSound(%d,%d,%q,%s,%f,%f)", objectId, soundID, samplePath, serialize(engineNodeIDs), offLoadGain, onLoadGain))
end

local function setEngineSoundParameterList(device, soundID, params, reference)
  device.soundConfiguration[reference] = device.soundConfiguration[reference] or {}
  device.soundConfiguration[reference].params = tableMergeRecursive(device.soundConfiguration[reference].params or {}, params)
  device.soundConfiguration[reference].soundID = soundID
  obj:queueGameEngineLua(string.format("core_sounds.setEngineSoundParameterList(%d,%d,%s)", objectId, soundID, serialize(params)))
end

local function soundParams(soundConfig)
  return {
    main_gain = soundConfig.mainGain or 0,
    eq_a_freq = sounds.hzToFMODHz(soundConfig.lowCutFreq or 20),
    eq_b_freq = sounds.hzToFMODHz(soundConfig.highCutFreq or 10000),
    eq_c_freq = sounds.hzToFMODHz(soundConfig.eqLowFreq or 500),
    eq_c_gain = soundConfig.eqLowGain or 0,
    eq_c_reso = soundConfig.eqLowWidth or 0,
    eq_d_freq = sounds.hzToFMODHz(soundConfig.eqHighFreq or 2000),
    eq_d_gain = soundConfig.eqHighGain or 0,
    eq_d_reso = soundConfig.eqHighWidth or 0,
    eq_e_gain = soundConfig.eqFundamentalGain or 0,
    eq_e_reso = soundConfig.eqFundamentalWidth or 1,
    onLoadGain = soundConfig.onLoadGain or 1,
    offLoadGain = soundConfig.offLoadGain or 1,
    muffled = soundConfig.muffled or 0.5
  }
end

local function initAvasSound(device, jbeamData)
  local avasConfig = jbeamData.avasSoundConfig and v.data[jbeamData.avasSoundConfig]
  if not avasConfig or not avasConfig.sampleName then
    return
  end
  device.soundConfiguration = device.soundConfiguration or {}
  device.avasSoundID = powertrain.getEngineSoundID()
  device.avasMaxSpeedKmh = avasConfig.maxSpeedKmh or 30
  device.avasFadeStartKmh = avasConfig.fadeStartKmh or 20
  device.avasStandstillVolume = avasConfig.standstillVolume or 0.5
  device.avasBaseRPM = avasConfig.baseRPM or 1200
  device.avasRPMPerKmh = avasConfig.rpmPerKmh or 120
  device.avasLoadMin = avasConfig.loadMin or 0.15
  device.avasLoadMax = avasConfig.loadMax or 1
  device.avasLoadVolumeGain = avasConfig.loadVolumeGain or 0.35
  device.avasRPMPerLoad = avasConfig.rpmPerLoad or 400
  device.avasFundamentalRPMCoef = (avasConfig.fundamentalFrequencyCylinderCount or 6) / 120
  device.avasVolumeSmoother = newTemporalSmoothing(2, 1.5)
  device.avasLoadSmoother = newTemporalSmoothing(4, 3)
  device.avasRPMSmoother = newTemporalSmoothingNonLinear(avasConfig.rpmSmootherInRate or 15, avasConfig.rpmSmootherOutRate or 25)
  local sampleFolder = avasConfig.sampleFolder or "art/sound/blends/"
  local samplePath = sampleFolder .. avasConfig.sampleName .. ".sfxBlend2D.json"
  device:initEngineSound(device.avasSoundID, samplePath, {device.engineNodeID}, avasConfig.offLoadGain or 1, avasConfig.onLoadGain or 1, "avas")
  device:setEngineSoundParameterList(device.avasSoundID, soundParams(avasConfig), "avas")
  device.updateSounds = updateSounds
end

local function initSounds(device, jbeamData)
  -- this device answers to "mainEngine" after validation, so powertrain may pass the engine's jbeam
  -- data here: always use this device's own data
  jbeamData = device.jbeamData
  -- After the lookup takeover this device sits twice in powertrain's device table ("phevModule" and
  -- "mainEngine"), so powertrain.initSounds calls this twice. Allocate the motor and AVAS sounds only once,
  -- otherwise the first sound IDs are orphaned (created in the sound engine, but never updated again).
  if device.soundsInitialized then
    return
  end
  device.soundsInitialized = true
  if sounds.usesOldCustomSounds then
    return
  end
  initAvasSound(device, jbeamData)
  if not jbeamData.soundConfig then
    return
  end
  local soundConfig = v.data[jbeamData.soundConfig]
  if not soundConfig or not soundConfig.sampleName then
    log("W", "phevModule.initSounds", "No motor sound config found: " .. tostring(jbeamData.soundConfig))
    return
  end
  device.soundConfiguration = device.soundConfiguration or {}
  device.engineSoundID = powertrain.getEngineSoundID()
  device.soundRPMSmoother = newTemporalSmoothingNonLinear(soundConfig.rpmSmootherInRate or 15, soundConfig.rpmSmootherOutRate or 25)
  device.soundLoadSmoother = newTemporalSmoothingNonLinear(soundConfig.loadSmootherInRate or 20, soundConfig.loadSmootherOutRate or 20)
  device.soundMaxLoadMix = soundConfig.maxLoadMix or 1
  device.soundMinLoadMix = soundConfig.minLoadMix or 0
  device.fundamentalFrequencyRPMCoef = (soundConfig.fundamentalFrequencyCylinderCount or 6) / 120
  device.engineVolumeCoef = 1
  local onLoadGain = soundConfig.onLoadGain or 1
  local offLoadGain = soundConfig.offLoadGain or 1
  local sampleFolder = soundConfig.sampleFolder or "art/sound/blends/"
  local samplePath = sampleFolder .. soundConfig.sampleName .. ".sfxBlend2D.json"
  device:initEngineSound(device.engineSoundID, samplePath, {device.engineNodeID}, offLoadGain, onLoadGain, "motor")
  device:setEngineSoundParameterList(device.engineSoundID, {
    main_gain = soundConfig.mainGain or 0,
    eq_a_freq = sounds.hzToFMODHz(soundConfig.lowCutFreq or 20),
    eq_b_freq = sounds.hzToFMODHz(soundConfig.highCutFreq or 10000),
    eq_c_freq = sounds.hzToFMODHz(soundConfig.eqLowFreq or 500),
    eq_c_gain = soundConfig.eqLowGain or 0,
    eq_c_reso = soundConfig.eqLowWidth or 0,
    eq_d_freq = sounds.hzToFMODHz(soundConfig.eqHighFreq or 2000),
    eq_d_gain = soundConfig.eqHighGain or 0,
    eq_d_reso = soundConfig.eqHighWidth or 0,
    eq_e_gain = soundConfig.eqFundamentalGain or 0,
    eq_e_reso = soundConfig.eqFundamentalWidth or 1,
    onLoadGain = onLoadGain,
    offLoadGain = offLoadGain,
    muffled = 0.5
  }, "motor")
  device.updateSounds = updateSounds
end

local function resetSounds(device, jbeamData)
  if device.soundRPMSmoother then
    device.soundRPMSmoother:reset()
    device.soundLoadSmoother:reset()
  end
  if device.avasVolumeSmoother then
    device.avasVolumeSmoother:reset()
    device.avasRPMSmoother:reset()
    device.avasLoadSmoother:reset()
  end
end

local function getSoundConfiguration(device)
  return device.soundConfiguration
end

------------------------------------------------------------------------------------------------
-- Interface that BeamNG's vehicle controller, DCT shift logic and ESC expect from "the engine"
------------------------------------------------------------------------------------------------

local function setIgnition(device, value)
  device.ignitionCoef = value > 0 and 1 or 0
  if device.ignitionCoef == 0 then
    device.systemReady = false
    device.iceRequested = false
    device.k0State = "off"
    device.k0CrankPhase = false
    device.k0Ratio = 0
    device.k0Target = 0
    device.parent:setIgnition(0)
    setIceSyncTarget(device, 0)
  end
end

-- the "start" button: no cranking, the hybrid system is simply ready ("READY" lamp)
local function activateStarter(device)
  if device.isBroken then
    return
  end
  device.ignitionCoef = 1
  device.systemReady = true
  device.starterEngagedCoef = 0
end

local function deactivateStarter(device)
  -- nothing to release: readiness is immediate
end

local function cutIgnition(device, time)
  device.ignitionCutTime = time
  if device.iceCoupled and device.parent.cutIgnition then
    device.parent:cutIgnition(time)
  end
end

local function setTempRevLimiter(device, revLimiterAV, maxOvershootAV)
  device.tempRevLimiterAV = revLimiterAV
  device.isTempRevLimiterActive = true
  if device.parent.setTempRevLimiter then
    device.parent:setTempRevLimiter(revLimiterAV, maxOvershootAV)
  end
end

local function resetTempRevLimiter(device)
  device.tempRevLimiterAV = math.huge
  device.isTempRevLimiterActive = false
  if device.parent.resetTempRevLimiter then
    device.parent:resetTempRevLimiter()
  end
end

local function buildTorqueData(device)
  local ice = device.parent
  local iceCurve = ice and ice.torqueCurve or {}
  local maxRPM = floor(device.maxRPM)
  local systemTorque, systemPower, motorTorque, motorPower, iceTorque, icePower = {}, {}, {}, {}, {}, {}
  local maxTorque, maxPower, maxTorqueRPM, maxPowerRPM = 0, 0, 0, 0
  local torqueToPowerKW = rpmToAV * 0.001 * 1.35962 -- Nm * rpm -> PS (UI shows power in the user's unit)
  for rpm = 0, maxRPM do
    local av = rpm * rpmToAV
    local tMotor = motorDriveTorqueAt(device, av)
    local tIce = (iceCurve[rpm] or 0)
    local tSystem = tMotor + tIce
    tSystem = min(tSystem, device.systemTorqueLimit)
    if av > 1 then
      tSystem = min(tSystem, device.systemPowerLimit / av)
    end
    systemTorque[rpm + 1] = tSystem
    systemPower[rpm + 1] = tSystem * rpm * torqueToPowerKW
    motorTorque[rpm + 1] = tMotor
    motorPower[rpm + 1] = tMotor * rpm * torqueToPowerKW
    iceTorque[rpm + 1] = tIce
    icePower[rpm + 1] = tIce * rpm * torqueToPowerKW
    if tSystem > maxTorque then
      maxTorque = tSystem
      maxTorqueRPM = rpm
    end
    if systemPower[rpm + 1] > maxPower then
      maxPower = systemPower[rpm + 1]
      maxPowerRPM = rpm
    end
  end
  local curves = {
    {torque = systemTorque, power = systemPower, name = "System (engine + motor)", priority = 100, width = 2},
    {torque = iceTorque, power = icePower, name = "1.4 turbo", priority = 50, width = 2, dash = {10, 4}},
    {torque = motorTorque, power = motorPower, name = "Electric motor", priority = 40, width = 2, dash = {8, 3, 4, 3}}
  }
  -- the vehicle controller indexes curves by number for shift point calculation
  local indexed = {}
  for i, c in ipairs(curves) do
    indexed[i] = c
    -- stock code reads curve tables from index 0 (rpm) upwards: provide a 0-based copy
    local t0 = {}
    for rpm = 0, maxRPM do
      t0[rpm] = c.torque[rpm + 1]
    end
    c.torque = t0
    local p0 = {}
    for rpm = 0, maxRPM do
      p0[rpm] = c.power[rpm + 1]
    end
    c.power = p0
  end
  return {maxRPM = maxRPM, curves = indexed, maxTorque = maxTorque, maxPower = maxPower, maxTorqueRPM = maxTorqueRPM, maxPowerRPM = maxPowerRPM, finalCurveName = 1, deviceName = device.name, vehicleID = obj:getId()}
end

local function getTorqueData(device)
  if not device.torqueData then
    device.torqueData = buildTorqueData(device)
  end
  return device.torqueData
end

local function sendTorqueData(device, data)
  guihooks.trigger("TorqueCurveChanged", data or device:getTorqueData())
end

------------------------------------------------------------------------------------------------
-- Lifecycle
------------------------------------------------------------------------------------------------

local function registerStorage(device, storageName)
  local storage = energyStorage.getStorage(storageName)
  if storage and storage.type == "electricBattery" then
    device.batteryName = storageName
  end
end

local function calculateInertia(device)
  local outputInertia = 0
  local cumulativeGearRatio = 1
  local maxCumulativeGearRatio = 1
  if device.children and #device.children > 0 then
    local child = device.children[1]
    outputInertia = child.cumulativeInertia or 0
    cumulativeGearRatio = child.cumulativeGearRatio or 1
    maxCumulativeGearRatio = child.maxCumulativeGearRatio or 1
  end
  device.cumulativeInertia = outputInertia + device.inertia
  device.cumulativeGearRatio = cumulativeGearRatio
  device.maxCumulativeGearRatio = maxCumulativeGearRatio
  -- The system curve doesn't depend on the gearbox: build it only when it is missing (after a reset). The DCT
  -- calls calculateTreeInertia on every gear pre-selection, and rebuilding the curves each time (about 0.5 MB
  -- of tables) caused garbage-collector hitches around shifts.
  if not rawget(device, "torqueData") then
    device.torqueData = buildTorqueData(device)
  end
end

-- Break-beam events registered for "mainEngine" (the 1.4 turbo) arrive here after the lookup takeover: forward
-- them. Defined above validate, which attaches it: a local defined further down isn't visible there, and
-- validate attached nil, so every broken beam raised a Lua error in powertrain.beamBroke.
local function beamBroke(device, id)
  local ice = device.parent
  if ice and ice.beamBroke then
    ice:beamBroke(id)
  end
end

-- the jbeam data powertrain builds for a device: its powertrain entry merged with its own section (powertrain.init)
local function findDeviceJbeamData(name)
  for _, entry in pairs(v.data.powertrain or {}) do
    if type(entry) == "table" and entry.name == name then
      local data = deepcopy(entry)
      tableMergeRecursive(data, v.data[name] or {})
      return data
    end
  end
  return nil
end

local function validate(device)
  if not device.parent or not device.parent.deviceCategories.engine or not device.parent.torqueCurve then
    log("E", "phevModule.validate", "The hybrid module must be the child of a combustionEngine device")
    return false
  end

  device.clutchChildren = {}
  device.activeOutputPorts = {}
  for _, child in ipairs(device.children or {}) do
    if child.deviceCategories.clutchlike then
      device.clutchChildren[child.inputIndex] = child
    else
      log("E", "phevModule.validate", "Found a non clutchlike child: " .. tostring(child.name))
      return false
    end
  end
  for i = 1, device.numberOfOutputPorts do
    if device.clutchChildren[i] or i == 1 then
      table.insert(device.activeOutputPorts, i)
      device.outputTorqueNames[i] = "outputTorque" .. tostring(i)
      device.outputAVNames[i] = "outputAV" .. tostring(i)
      device[device.outputTorqueNames[i]] = device[device.outputTorqueNames[i]] or 0
      device[device.outputAVNames[i]] = device[device.outputAVNames[i]] or 0
    end
  end
  device.clutchChildren[1] = device.clutchChildren[1] or {torqueDiff = 0}
  device.activeOutputPortCount = #device.activeOutputPorts

  local ice = device.parent
  device.thermals = ice.thermals
  device.iceIdleAV = ice.idleAV or device.iceIdleAV
  -- report engine idle until the first update: the DCT shift logic derives its launch constants from it
  device.idleAV = device.iceIdleAV
  device.idleRPM = device.idleAV * avToRPM

  -- take over the "mainEngine" lookup: the vehicle controller and the DCT shift logic look up the engine
  -- by that name and must talk to the hybrid system. The 1.4 turbo stays reachable as "iceEngine" (and as
  -- this device's parent); its own name stays "mainEngine" so its jbeam data and resets keep working.
  local lookup = powertrain.getDevices()
  -- thermostat temperature (radiator part) and BeamNG's oil heat capacity (oil mass x 1800 J/kgK), for the
  -- thermal corrections (updateEngineThermals)
  local engineData = v.data and v.data[ice.name] or {}
  device.warmUpThermostat = engineData.thermostatTemperature or 90
  device.oilHeatCapacity = (engineData.oilVolume or 5) * 0.87 * 1800
  device.thermalOilLast = nil
  lookup.mainEngine = device
  lookup.iceEngine = ice

  -- the real car is "ready" with the engine stopped: the 1.4 turbo spawns and resets with ignition on but not
  -- running (at most ignition level 2), the strategy starts it through K0 when it is needed.
  -- Ordering assumption: the engine reads this field in its reset, which has already run when validate is called, so
  -- the output speeds are zeroed here for the first spawn; the module's own resetState reads electrics ignitionLevel.
  ice.spawnVehicleIgnitionLevel = math.min(ice.spawnVehicleIgnitionLevel or 0, 2)
  for i = 1, ice.activeOutputPortCount or 1 do
    local port = ice.activeOutputPorts and ice.activeOutputPorts[i] or 1
    if ice.outputAVNames and ice.outputAVNames[port] then
      ice[ice.outputAVNames[port]] = 0
    end
  end
  ice.outputAV1 = 0
  ice.lastOutputAV1 = 0
  ice.outputRPM = 0
  -- The host car's original parts address the engine by name: traction and stability control key their
  -- wheel-group settings by the device name, drive modes adjust exhaust and turbo of "mainEngine".
  -- Answer to that name as well, and pass engine-only calls through to the 1.4 turbo.
  device.name = "mainEngine"
  device.uiName = device.uiName or "Hybrid system"
  -- the system curve: built now with the final name (the DCT may have built it lazily before this device
  -- validated), afterwards only after a reset (see calculateInertia)
  device.torqueData = buildTorqueData(device)
  -- break events registered for "mainEngine" (the 1.4 turbo) now arrive here: forward them. Attached only
  -- now, after powertrain registered its break events, so the engine doesn't receive them twice.
  device.beamBroke = beamBroke
  -- powertrain's breakDevice() iterates the trigger beams of the device it resolves by name
  device.breakTriggerBeam = ice.breakTriggerBeam or {}
  device.turbocharger = ice.turbocharger
  device.supercharger = ice.supercharger
  device.nitrousOxideInjection = ice.nitrousOxideInjection

  -- powertrain resets every device with the jbeam data it stored under the device's current name. After the
  -- takeover and the rename to "iceEngine" (Powertrain app, see setupVisualLayout) that lookup gives the 1.4 turbo
  -- nothing, and its reset and resetSounds index it (a Lua error on every vehicle reset): pass its own data.
  local iceJbeamData = findDeviceJbeamData(ice.name)
  if iceJbeamData then
    local iceReset, iceResetSounds, iceInitSounds = ice.reset, ice.resetSounds, ice.initSounds
    if iceReset then
      ice.reset = function(d, jbeamData)
        return iceReset(d, jbeamData or iceJbeamData)
      end
    end
    if iceResetSounds then
      ice.resetSounds = function(d, jbeamData)
        return iceResetSounds(d, jbeamData or iceJbeamData)
      end
    end
    if iceInitSounds then
      ice.initSounds = function(d, jbeamData)
        return iceInitSounds(d, jbeamData or iceJbeamData)
      end
    end
  else
    log("W", "phevModule.validate", "No jbeam data found for the engine " .. tostring(ice.name))
  end

  table.insert(powertrain.engineData, {maxRPM = device.maxRPM, torqueReactionNodes = device.torqueReactionNodes})
  return true
end

-- Engine break triggers are dispatched by the name "mainEngine", which now resolves to this device:
-- the break belongs to the 1.4 turbo. Seize the engine, open K0 and keep driving electrically.
local function onBreak(device)
  local ice = device.parent
  if ice and not ice.isBroken and ice.onBreak then
    ice:onBreak()
  end
  -- open K0 at once: a seized engine must not brake the drivetrain
  device.k0Target = 0
  device.k0Ratio = 0
  device.k0Angle = 0
  if device.k0State ~= "off" then
    device.k0State = "stop"
  end
end

-- Calls that BeamNG dispatches by the name "mainEngine" but that belong to the 1.4 turbo:
-- part condition (career wear), exhaust sound offsets (drive modes). Break-beam events: beamBroke, above validate.
local function setPartCondition(device, subSystem, odometer, integrity, visual)
  local ice = device.parent
  if ice and ice.setPartCondition then
    return ice:setPartCondition(subSystem, odometer, integrity, visual)
  end
end

local function getPartCondition(device, subSystem)
  local ice = device.parent
  if ice and ice.getPartCondition then
    return ice:getPartCondition(subSystem)
  end
end

local function setExhaustGainMufflingOffset(device, mufflingOffset, gainOffset)
  local ice = device.parent
  if ice and ice.setExhaustGainMufflingOffset then
    ice:setExhaustGainMufflingOffset(mufflingOffset, gainOffset)
  end
end

local function setMode(device, mode)
  device.mode = mode
end

local function resetState(device, jbeamData)
  device.outputAV1 = 0
  device.outputTorque1 = 0
  device.inputAV = 0
  device.torqueDiff = 0
  device.isBroken = false
  device.isDisabled = false
  device.starterDisabled = false
  device.mode = "connected"

  device.k0State = "off"
  device.k0CrankPhase = false
  device.k0Ratio = 0
  device.k0Target = 0
  device.k0Angle = 0
  device.k0CapacityCoef = 1
  -- clear the engine's idle-controller targets of an engine start in progress (the engine's own reset clears
  -- idleAVOverwrite but not maxIdleThrottleOverwrite). The parent is only linked once the tree is built.
  if device.parent and device.parent.deviceCategories then
    setIceSyncTarget(device, 0)
  end
  device.k0Torque = 0
  device.stateTimer = 0
  device.syncTimer = 0
  device.syncTrimAV = 0
  device.startRetryTimer = 0
  device.startFailures = 0
  device.iceCoupled = false
  device.iceRequested = false
  device.iceThrottleFactor = 1

  device.motorTorque = 0
  device.motorDemand = 0
  device.throttleFactor = 1
  device.systemCapActive = true
  device.syncTargetRatio = nil
  device.syncTau = 0.02
  device.lastClutchLoad = 0
  device.syncWasActive = false
  device.syncMotorHold = 0
  device.syncIceLimitSet = false
  if device.syncIceLimit and device.parent then
    device.parent.maxTorqueLimit = math.huge
  end
  device.syncIceLimit = nil
  device.syncDown = false
  device.syncBlipAV = nil
  device.syncBlipThrottle = 0
  device.syncReleaseTorque = nil
  device.clutchLoadFiltered = 0
  device.throttle = 0
  device.regenThrottle = 0
  device.tcShortfall = 0
  device.tcActiveTime = 0
  device.iceTcFactor = 1
  device.instantMotorPower = 0
  device.instantElectricPower = 0
  device.instantTerminalPower = 0
  device.instantChemicalPower = 0
  device.batteryCurrent = 0
  device.spentEnergy = 0
  device.ignitionCutTime = 0
  device.tempRevLimiterAV = math.huge
  device.isTempRevLimiterActive = false
  device.revLimiterActive = false
  device.idleHoldActive = false
  device.powerDisplay = 2400
  device.powerDisplaySmoother:reset()

  -- spawn state follows the vehicle's spawn ignition level, like the stock engines
  local spawnLevel = electrics.values.ignitionLevel or 0
  device.ignitionCoef = spawnLevel > 1 and 1 or 0
  device.systemReady = spawnLevel > 2
  device.starterEngagedCoef = 0
  device.motorActive = device.systemReady
  device.launchMode = "electric"
  device.idleAV = device.iceIdleAV
  device.idleRPM = device.idleAV * avToRPM
  -- ready with both motors stopped (the 1.4 turbo spawns stopped, see validate)
  device.outputAV1 = 0
  device.isStalled = false

  device.usableSoc = 0
  device.socValid = false
  device.rawSoc = 0
  device.batteryDriveCoef = 1
  device.batteryRegenCoef = 1
  device.chargeIdleAV = 0
  device.batteryLossEnergy = 0
  -- the pack starts at ambient + batteryStartOffset (set in the first update, when the ambient is known)
  device.batteryTemp = nil
  device.dischargeLimitNow = device.batteryMaxDischargePower
  device.chargeLimitNow = device.batteryMaxChargePower
  device.batteryResistanceNow = device.batteryResistance
end

local function reset(device, jbeamData)
  resetState(device, jbeamData)
  rawset(device, "torqueData", nil)
end

local function new(jbeamData)
  local device = {
    deviceCategories = shallowcopy(M.deviceCategories),
    requiredExternalInertiaOutputs = shallowcopy(M.requiredExternalInertiaOutputs),
    outputPorts = shallowcopy(M.outputPorts),
    name = jbeamData.name,
    type = jbeamData.type,
    inputName = jbeamData.inputName,
    inputIndex = jbeamData.inputIndex,
    gearRatio = 1,
    friction = 0,
    -- the shift logic may use the motor to synchronise DCT shifts (see phevDct.lua)
    phevSyncShift = true,
    -- traction control: the engine throttle follows the motor's torque cut after this long (s)
    tcEngineFollowTime = jbeamData.tcEngineFollowTime or 0.3,
    tcShortfall = 0,
    tcActiveTime = 0,
    iceTcFactor = 1,
    throttle = 0,
    regenThrottle = 0,
    syncTau = 0.02,
    lastClutchLoad = 0,
    -- coupled upshifts: the engine torque reduction of the inertia phase and its release ramp (Nm/s, Nm)
    syncWasActive = false,
    syncMotorHold = 0,
    syncIceLimit = nil,
    syncIceLimitSet = false,
    syncTorqueRampRate = jbeamData.syncTorqueRampRate or 1500,
    syncTorqueReleaseLimit = jbeamData.syncTorqueReleaseLimit or 320,
    -- coupled downshifts: most extra motor power for the speed match (W; the engine blips for the rest) and the
    -- rate at which the motor returns to the strategy's demand after a downshift (Nm/s)
    revMatchPower = (jbeamData.revMatchPowerKW or 12) * 1000,
    syncReleaseRate = jbeamData.syncReleaseRate or 1500,
    syncDown = false,
    syncStartAV = 1,
    clutchLoadFiltered = 0,
    shaftInertiaEffective = jbeamData.inertia or 0.08,
    cumulativeGearRatio = 1,
    maxCumulativeGearRatio = 1,
    isPhysicallyDisconnected = true,
    isPropulsed = true,
    electricsName = jbeamData.electricsName,
    visualShaftAVName = jbeamData.visualShaftAVName,
    availableModes = {"connected"},
    mode = "connected",
    numberOfOutputPorts = 1,
    outputTorqueNames = {},
    outputAVNames = {},
    clutchChildren = {},
    activeOutputPorts = {},
    activeOutputPortCount = 0,

    -- shaft: K0 and bearing friction (Nm, Nm per rad/s), the motor's no-load spin loss and the dual-clutch gearbox's drag
    -- (open wet clutch, churning, pump) at the input shaft, all as constant + coefficient x speed
    inertia = jbeamData.inertia or 0.08,
    shaftFriction = jbeamData.friction or 0.2,
    shaftDynamicFriction = jbeamData.dynamicFriction or 0.0004,
    motorSpinLoss = jbeamData.motorSpinLoss or 0.48,
    motorSpinLossCoef = jbeamData.motorSpinLossCoef or 0.0018,
    dctDragTorque = jbeamData.dctDragTorque or 0.4,
    dctDragCoef = jbeamData.dctDragCoef or 0.003,

    -- "engine" facing values
    idleRPM = jbeamData.idleRPM or 800,
    maxRPM = jbeamData.maxRPM or 6000,
    starterMaxAV = (jbeamData.readyRPM or 500) * rpmToAV,
    hasRevLimiter = true,
    idleAVStartOffset = 0,
    idleAVOverwrite = 0,

    -- motor (workshop manual: 85 kW, 330 Nm, 7000 rpm)
    motorPeakTorque = jbeamData.motorPeakTorque or 330,
    motorPeakPower = (jbeamData.motorPeakPowerKW or 85) * 1000,
    motorMaxAV = (jbeamData.motorMaxRPM or 7000) * rpmToAV,
    motorTaperAV = (jbeamData.motorTaperRPM or 300) * rpmToAV,
    -- nominal efficiency: used for estimates (shift logic) and, with motorLossMap = false, as a constant both ways
    motorEfficiency = jbeamData.motorEfficiency or 0.9,
    -- motor + inverter loss map (see motorLoss): W, W per Nm, W per Nm^2, W at base speed and full torque, W at
    -- full field weakening; the corner speed of the torque curve and the speed where field weakening starts
    motorLossMap = jbeamData.motorLossMap ~= false,
    motorLossConst = jbeamData.motorLossConstW or 300,
    motorLossConstTorque = jbeamData.motorLossConstTorque or 5,
    motorLossInverter = jbeamData.motorLossInverter or 6.5,
    motorLossCopper = jbeamData.motorLossCopper or 0.040,
    motorLossIron = jbeamData.motorLossIronW or 900,
    motorLossFieldWeak = jbeamData.motorLossFieldWeakW or 1500,
    motorBaseAV = (jbeamData.motorBaseRPM or 2460) * rpmToAV,
    motorFieldWeakAV = (jbeamData.motorFieldWeakRPM or 4500) * rpmToAV,
    -- auxiliaries on the HV battery while the system is on (12 V converter etc., logs of the real car about 0.7 kW)
    auxPower = (jbeamData.auxPowerKW or 0.7) * 1000,
    regenPowerLimit = (jbeamData.regenPowerLimitKW or 40) * 1000,
    regenMinAV = (jbeamData.regenMinRPM or 900) * rpmToAV,

    -- system limits (160 kW / 400 Nm)
    systemPowerLimit = (jbeamData.systemPowerKW or 160) * 1000,
    systemTorqueLimit = jbeamData.systemTorque or 400,
    -- input torque limit in 1st gear with the engine coupled (Nm), see updateGFX
    firstGearTorqueLimit = jbeamData.firstGearTorqueLimit or 400,

    -- battery
    batteryName = nil,
    batteryMaxDischargePower = (jbeamData.batteryMaxDischargeKW or 90) * 1000,
    batteryMaxChargePower = (jbeamData.batteryMaxChargeKW or 40) * 1000,
    socMin = jbeamData.batteryUsableMin or 0.225,
    socMax = jbeamData.batteryUsableMax or 0.90,
    -- internal resistance of the pack (ohm), cells in series and the cell's open-circuit voltage by raw charge
    batteryResistance = jbeamData.batteryResistance or 0.13,
    batteryCells = jbeamData.batteryCells or 96,
    batteryCellOCV = type(jbeamData.batteryCellOCV) == "table" and jbeamData.batteryCellOCV
      or {{0, 3.40}, {0.1, 3.55}, {0.2, 3.61}, {0.3, 3.64}, {0.4, 3.67}, {0.5, 3.71}, {0.6, 3.76}, {0.7, 3.84}, {0.8, 3.92}, {0.9, 4.02}, {1.0, 4.12}},
    batteryOCVNow = 352,
    -- BMS power limits and pack temperature (updateBatteryLimits): maps {usable charge or C, share of the rated power},
    -- cell voltage limits, resistance rise in the cold, thermal mass and cooling (all assumed)
    batteryDischargeSocMap = type(jbeamData.batteryDischargeSocMap) == "table" and jbeamData.batteryDischargeSocMap or {{0, 0.5}, {0.03, 0.75}, {0.1, 1.0}},
    batteryChargeSocMap = type(jbeamData.batteryChargeSocMap) == "table" and jbeamData.batteryChargeSocMap or {{0, 1.0}, {0.85, 1.0}, {0.95, 0.6}, {1.0, 0.25}},
    batteryDischargeTempMap = type(jbeamData.batteryDischargeTempMap) == "table" and jbeamData.batteryDischargeTempMap or {{-20, 0.35}, {-10, 0.55}, {0, 0.8}, {10, 1.0}, {45, 1.0}, {55, 0.6}, {60, 0.2}},
    batteryChargeTempMap = type(jbeamData.batteryChargeTempMap) == "table" and jbeamData.batteryChargeTempMap or {{-10, 0.1}, {0, 0.25}, {10, 0.6}, {20, 1.0}, {40, 1.0}, {50, 0.6}, {55, 0.3}, {60, 0}},
    batteryCellVmin = jbeamData.batteryCellVmin or 3.0,
    batteryCellVmax = jbeamData.batteryCellVmax or 4.2,
    batteryResistanceRefC = jbeamData.batteryResistanceRefC or 25,
    batteryResistanceTempCoef = jbeamData.batteryResistanceTempCoef or 0.035,
    batteryResistanceMinCoef = jbeamData.batteryResistanceMinCoef or 0.7,
    batteryHeatCapacity = (jbeamData.batteryHeatCapacityKJK or 130) * 1000,
    batteryPassiveUA = jbeamData.batteryPassiveUA or 15,
    batteryCoolantUA = jbeamData.batteryCoolantUA or 60,
    batteryCoolStart = jbeamData.batteryCoolStartC or 25,
    batteryChillerPower = (jbeamData.batteryChillerKW or 3) * 1000,
    batteryChillerStart = jbeamData.batteryChillerStartC or 32,
    batteryChillerFull = jbeamData.batteryChillerFullC or 38,
    batteryStartOffset = jbeamData.batteryStartOffsetC or 5,

    -- K0 clutch
    k0LockTorque = jbeamData.k0LockTorque or 600,
    k0Spring = jbeamData.k0Spring or 4000,
    k0Damp = jbeamData.k0Damp or 25,
    k0CloseRate = 1 / (jbeamData.k0CloseTime or 0.25),
    k0OpenRate = 1 / (jbeamData.k0OpenTime or 0.15),
    k0CrankRatio = jbeamData.k0CrankRatio or 0.3,
    k0CrankRate = 1 / (jbeamData.k0CrankRampTime or 0.08),
    crankTorque = jbeamData.crankTorque or 80,
    crankTimeout = jbeamData.crankTimeout or 2,
    iceFireAV = (jbeamData.iceFireRPM or 600) * rpmToAV,
    syncToleranceAV = (jbeamData.syncToleranceRPM or 40) * rpmToAV,
    syncHoldTime = jbeamData.syncHoldTime or 0.08,
    -- after this long K0 closes anyway and slips the remaining difference (before the sync lead: 2.5 s)
    syncTimeout = jbeamData.syncTimeout or 1.0,
    -- the engine's speed target in the sync phase leads the shaft (the idle controller is proportional only) and is
    -- trimmed by the remaining error: lead, integral gain (1/s), the range in which the trim works and its limit
    syncLeadAV = (jbeamData.syncLeadRPM or 50) * rpmToAV,
    syncTrimGain = jbeamData.syncTrimGain or 5,
    syncTrimRangeAV = (jbeamData.syncTrimRangeRPM or 300) * rpmToAV,
    syncTrimMaxAV = (jbeamData.syncTrimMaxRPM or 250) * rpmToAV,
    -- the engine may use full throttle to catch up with the shaft (engine starts while the car accelerates)
    syncMaxThrottle = jbeamData.syncMaxThrottle or 1.0,
    -- engine temperatures, see updateEngineThermals (values fitted to the real car's logs)
    -- warm-up: extra coolant heat per watt of gross engine power while the thermostat is closed, by coolant
    -- temperature, and the engine power it is capped at
    warmUpHeatGain = type(jbeamData.warmUpHeatGain) == "table" and jbeamData.warmUpHeatGain or {{10, 9.5}, {35, 6.3}, {55, 3.5}, {75, 1.1}, {88, 1.6}},
    warmUpHeatMaxPower = (jbeamData.warmUpHeatMaxPowerKW or 40) * 1000,
    -- engine off, system on: extra coolant heat loss (W per K above ambient), its relative increase per
    -- 100 km/h, and the cabin heater's extra loss (W/K per K of ambient below coolDownHeaterAmbient)
    coolDownLossCoef = jbeamData.coolDownLossCoef or 72,
    coolDownLossSpeedCoef = jbeamData.coolDownLossSpeedCoef or 0.7,
    coolDownHeaterLoss = jbeamData.coolDownHeaterLoss or 3.9,
    coolDownHeaterAmbient = jbeamData.coolDownHeaterAmbient or 15,
    -- oil: share of the net engine power that heats the oil, sump airflow cooling (W/K at full airflow),
    -- thermal mass factor on BeamNG's oil
    oilLoadShare = jbeamData.oilLoadShare or 0.36,
    oilAirCoef = jbeamData.oilAirCoef or 210,
    oilMassFactor = jbeamData.oilMassFactor or 13.0,
    -- share of BeamNG's friction heat that stays in the oil (the rest heats the coolant), the coolant temperature from
    -- which the load heat reaches the oil (C, over 10 K), the oil-coolant exchanger (W/K) and its opening temperature
    oilFrictionShare = jbeamData.oilFrictionShare or 0.8,
    oilLoadGateTemp = jbeamData.oilLoadGateTemp or 80,
    oilCoolerCoef = jbeamData.oilCoolerCoef or 200,
    oilCoolerGateTemp = jbeamData.oilCoolerGateTemp or 84,
    startRetryDelay = jbeamData.startRetryDelay or 5,
    -- fuel (updateFuelCalibration): Willans line from the logs of the real car on the logged (2.3x high) fuel-rate scale,
    -- g/kWh of indicated power and g/s, the scale factor, the logged friction (Nm), the near-full-load penalties
    fuelCalibration = jbeamData.fuelCalibration ~= false,
    fuelWillansSlope = jbeamData.fuelWillansSlope or 537.5,
    fuelWillansOffset = jbeamData.fuelWillansOffset or 0.479,
    fuelScaleF = jbeamData.fuelScaleF or 2.57,
    fuelIndicatedFriction = jbeamData.fuelIndicatedFriction or 14,
    fuelKnockPenalty = jbeamData.fuelKnockPenalty or 0.08,
    fuelEnrichPenalty = jbeamData.fuelEnrichPenalty or 0.10,
    fuelBlendTorque = jbeamData.fuelBlendTorque or 15,
    fuelEnergyDensity = (jbeamData.fuelEnergyDensityMJ or 41.5) * 1e6,

    -- idle hold by the motor
    idleHoldP = jbeamData.idleHoldP or 4,
    idleHoldMaxTorque = jbeamData.idleHoldMaxTorque or 60,

    iceThrottleFactorName = jbeamData.iceThrottleFactorName or "phevIceThrottleFactor",
    powerDisplaySmoother = newExponentialSmoothing(30),

    reset = reset,
    validate = validate,
    calculateInertia = calculateInertia,
    updateGFX = updateGFX,
    registerStorage = registerStorage,
    initSounds = initSounds,
    resetSounds = resetSounds,
    initEngineSound = initEngineSound,
    setEngineSoundParameterList = setEngineSoundParameterList,
    getSoundConfiguration = getSoundConfiguration,
    setIgnition = setIgnition,
    activateStarter = activateStarter,
    deactivateStarter = deactivateStarter,
    cutIgnition = cutIgnition,
    setTempRevLimiter = setTempRevLimiter,
    resetTempRevLimiter = resetTempRevLimiter,
    getTorqueData = getTorqueData,
    sendTorqueData = sendTorqueData,
    onBreak = onBreak,
    setPartCondition = setPartCondition,
    getPartCondition = getPartCondition,
    setExhaustGainMufflingOffset = setExhaustGainMufflingOffset,
    setMode = setMode,
    getUsableSoc = getUsableSoc,
    getRawSoc = getRawSoc,
    motorDriveTorqueAt = motorDriveTorqueAt,
    motorRegenTorqueAt = motorRegenTorqueAt,
    motorElectricPower = motorElectricPower,
    batteryChemicalPower = batteryChemicalPower,
    -- for offline tests
    updateBatteryLimits = updateBatteryLimits
  }

  device.jbeamData = jbeamData
  device.throttleFactor = 1
  device.iceIdleAV = device.idleRPM * rpmToAV
  device.idleAV = device.iceIdleAV
  device.maxAV = device.maxRPM * rpmToAV
  device.invInertia = 1 / device.inertia
  device.shaftDragConst = device.shaftFriction + device.motorSpinLoss + device.dctDragTorque
  device.shaftDragCoef = device.shaftDynamicFriction + device.motorSpinLossCoef + device.dctDragCoef
  device.energyStorage = jbeamData.energyStorage

  -- torqueData is read as a field by the DCT during validation, before this device validates: build it lazily
  setmetatable(device, {__index = function(t, key)
    if key == "torqueData" and t.parent then
      local data = buildTorqueData(t)
      rawset(t, "torqueData", data)
      return data
    end
    return nil
  end})

  -- node for the motor sound and the visual position
  local reactionNodes = jbeamData.torqueReactionNodes_nodes
  if type(reactionNodes) == "table" then
    for _, nodeId in pairs(reactionNodes) do
      if type(nodeId) ~= "number" then
        reactionNodes = nil
        break
      end
    end
  else
    reactionNodes = nil
  end
  device.torqueReactionNodes = reactionNodes or {-1, -1, -1}
  local refNode = v.data.refNodes and v.data.refNodes[0] and v.data.refNodes[0].ref or 0
  device.engineNodeID = device.torqueReactionNodes[1]
  if not device.engineNodeID or device.engineNodeID < 0 then
    device.engineNodeID = refNode
  end

  device.velocityUpdate = updateVelocity
  device.torqueUpdate = updateTorque

  resetState(device, jbeamData)
  return device
end

M.new = new

return M
