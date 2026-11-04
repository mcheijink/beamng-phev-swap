# Plug-in Hybrid Swap: where the core values come from

The shared core parts (`phev_system`, `phev_dct6`, `phev_battery_*` in `mod/vehicles/common/phev/phev_core.jbeam`) and the engine, turbo and pack data of the vehicle adapters are original values for this mod. They are not taken from another mod. Each value has one of these sources:

- **Measurements** of a real plug-in hybrid estate car: drive logs and own calibration runs.
- **Published data**: maker's figures, workshop manual, press data.
- **Stock value** of the game's own parts, named. The stock parts are the game's; the value is reused because there is no better source.
- **Fit**: derived from the sources above with a stated method.

## Own values

| Item | Value | Source |
|---|---|---|
| Base torque table of the 1.4 engine (500 .. 6,500 rpm, Nm) | 81.4 / 124.5 / 143.2 / 145.6 / 159.4 / 160.1 / 160.9 / 161.6 / 162.4 / 156.8 / 152.4 / 147.2 / 136.6 / 127.8 / 120.6 | Fit: solved for the published full-load curve (250 Nm at 1,500-3,500 rpm, 115 kW at 5,000 rpm) with the turbo model below |
| Turbo efficiency column (1,250 .. 6,000 rpm) | 1.0 from 1,250 rpm, ramp below | Measurements: boosted from 1,250 rpm, 185-199 kPa at full load at every rpm |
| Turbo exhaust column (0 .. 1,250 rpm) | rpm / 1,500 | Exhaust flow in proportion to rpm |
| Turbo pressure curve (turbine rpm: psi) | 25,000: 0.47, 50,000: 1.34, 75,000: 2.46, 100,000: 3.79, 125,000: 5.3, 150,000: 6.97, 175,000: 8.78, 200,000: 10.73, 225,000: 12.81, 250,000: 15 | Fit: power law 15 psi x (speed / 250,000)^1.5 to the measured boost map |
| Turbo maxExhaustPower | 4,375 | Fit, together with the pressure curve |
| Turbo inertia | 0.9 | Measurements: 63 % of the boost rise after a median 1.5 s (model: 1.53 s) |
| Turbo pressureRatePSI | 15 | Measurements: boost decay, 63 % of 12 psi in 0.5 s |
| Turbo frictionCoef | 15.5 | Stock value of the game's 2.0 turbo parts |
| Engine idle / maximum / limiter rpm | 800 / 6,500 / 6,000 | Stock values of the game's engines (idle 800 in 36 stock parts, limiter 6,000 in 8); 6,500 is the top of the published curve |
| Engine friction / dynamic friction | 25 / 0.025 | Stock values (stock V8 parts); the oil-heat fit is made with them |
| Engine burn-efficiency table, starter throttle kill time, damage thresholds (Sunburst) | the game's own tables and thresholds | Stock values: the same tables and thresholds are in the game's engine parts (2.0 diesel, V8s) |
| Gearbox parkLockTorque | 2,000 | Stock value of the host cars' dual-clutch boxes; holds a 2 t car on a 1:2 slope |
| Gearbox friction (the only drag the dual-clutch device applies) | 0 | The box's drag is in the hybrid module (own calibration) |
| Gearbox keys not set (dynamicFriction, torqueLossCoef, gearChangeTime, shiftEfficiency, oneWayViscousCoef, engineBrakeTorque) | not set | Checked in the game's `dctGearbox.lua`: the device does not read them |
| Shift filters: shiftDownRPMOffsetCoef, transmissionGearChangeDelay, aggressionHoldOffThrottleDelay, aggressionSmoothingUp / Down, gearboxDecisionSmoothingUp / Down | 1.15 / 0.25 / 2.5 / 2 / 0.15 / 2 / 2 | Stock dual-clutch values of the host cars' 7-speed DCT part: calm, slow shift decisions |
| Shift keys of the automatic gearbox logic (min / max / sportGearChangeTime, throttleCoefWhileShifting) and highShiftDownRPM | not set | Not read by the dual-clutch shift logic; the full-load points are calculated by the game (`calculateOptimalLoadShiftPoints`) |
| Gear ratios (R, 1st .. 6th) | -2.203, 3.5, 2.1334, 1.4248, 1.02, 0.7847, 0.6462 | Published ratio listing of the real gearbox, fitted to the drive-log measurements |
| Battery pack mass | 173 kg | Published pack 135 kg + 38 kg charger, cables and other parts |

## Values that equal common stock values

- Launch clutch rpm 1,200 / 1,500: stock 2.0 diesel engine part.
- Starter throttle kill time 0.65: 10 stock parts.
- Power electronics mass of 53 kg: an estimate fitted to the measured mass of the real car; no published figure was found.

## Behaviour that follows from these values

- Part-load torque at high rpm: the turbo gain does not fall off above 3,000 rpm, so the engine without boost makes about 160 Nm gross up to 3,500 rpm and 128 Nm at 6,000 rpm.
- Boost build-up follows the pressure curve and inertia above.
- Shift decisions are calm (the stock filters).
- The gearbox adds no drag of its own (the module carries it).
