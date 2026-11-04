# Plug-in Hybrid Swap: adapter interface for vehicle mods

How a vehicle mod (a car that is not one of the game's own) uses the drivetrain of the Plug-in Hybrid Swap mod (`phev_swap_mcheijink.zip`). The worked example is the ETK 800 port inside this repository: `mod/vehicles/common/phev/phev_etk.jbeam` (parts `phev_etk_ecu`, `phev_etk_transmission_dct`, `phev_etk_drivemodes`, `phev_etk_battery_*`) and `mod/vehicles/etk800/`. Every part and key name below can be looked up there. In the text, "a car mod" is any mod that adds a car and wants to offer a configuration with this drivetrain.

## 1. What the mod gives you

| Piece | Where | What it is |
|---|---|---|
| Strategy controller `phev` | `lua/vehicle/controller/phev.lua` | Drive modes Electric / Hybrid / Boost, Charge mode, engine start and stop, torque split, regeneration, brake blending, brake lights from regeneration, B position, kickdown, optional brake torque vectoring |
| Hybrid module device `phevModule` | `lua/vehicle/powertrain/phevModule.lua` | One powertrain device: K0 clutch, 85 kW motor, input shaft, battery accounting, engine start sequence, thermal model |
| Shift logic `phevDct` | `lua/vehicle/controller/shiftLogic-phevDct.lua` | Modified copy of BeamNG's dual-clutch shift logic (bCDDL, header kept) |
| Core parts | `vehicles/common/phev/phev_core.jbeam` | `phev_system` (module + strategy data), `phev_dct6` (gearbox ratios, shift map), `phev_battery_10`, `_13`, `_18` and `phev_battery_9_9` (battery cells; each has its own slot type `phev_cells_<kWh>`) |
| Monitor app | `ui/modules/apps/PhevMonitor` | "Plug-in Hybrid Monitor", reads the `phev*` electrics, shows only on a car that publishes `phevMode` |

`vehicles/common/` is loaded for every car, so a car mod does not have to ship the core parts. It only has to offer the slots that take them.

The Lua is in the game's shared folders (`lua/vehicle/...`), not in a car folder. From the game code (`lua/vehicle/main.lua`, `powertrain.lua`, `controller.lua`, `vehicleController.lua`): `package.path` starts with `lua/vehicle/?.lua`, the powertrain device types are found by listing `lua/vehicle/powertrain` plus the car's `lua/powertrain`, a controller row `phev` is loaded with `require("controller/phev")`, and `shiftLogicName phevDct` falls back to `controller/shiftLogic-phevDct`. Nothing in the Lua restricts these to the game's own files. If a game version does not serve a mod's `lua/vehicle` files to `require`, put the three Lua files into each car's own `lua/` folder instead.

## 2. What the car mod adds (the adapter)

The adapter is a few small parts in the car mod.

### 2.1 Parts and slots

| Adapter part | Slot type it fills | What it holds |
|---|---|---|
| Engine data | the car's engine or ECU slot | `mainEngine` with the 1.4 turbo performance (torque table solved for the car's own intake and exhaust), `electricsThrottleFactorName "phevIceThrottleFactor"`, `turbocharger` data. A car mod that ships its own engine part puts this in the engine part; the vanilla ports use the ECU slot of the game's engine |
| Transmission | the car's transmission slot | the gearbox row, mass nodes of motor and electronics, the gearbox node, the adapter keys of 2.3, and the **slots below** |
| Battery pack | a slot the transmission part (or the tank part) offers | nodes and beams that carry the pack's mass, the petrol tank if the car's tank part is replaced; one slot `phev_battery` |
| Final drive | the car's final-drive slot | the ratio, on `differential_F`, `differential_R` or `torsionReactorF` |
| Drive modes | the car's drive-mode slot | a `driveModes` section whose three modes call the `phev` controller (2.5) |

Slots the transmission part must offer (use the part's own slot format, `slots` or `slots2`; mark them `"coreSlot": true` as the example does):

| Slot name | Default part | Description |
|---|---|---|
| `phev_system` | `phev_system` | Hybrid module and strategy |
| `phev_gearbox` | `phev_dct6` | Gearbox settings |

and the battery part offers

| Slot name | Default part |
|---|---|
| `phev_battery` | `phev_battery_13` (or `phev_battery_9_9`) |

Write this slot row in the `slots2` format with `allowTypes` of the one cell type that fits the pack (`["phev_cells_13"]`, or `_10`, `_18`, `_9_9`): the slot name stays `phev_battery`, and a pack part cannot be combined with cells of another size. A host that offers several pack sizes ships one pack part per size, each with its own mass nodes (the vanilla ports do, see [PHEV-TUNING.md](PHEV-TUNING.md)).

Example (ETK port): `phev_etk_transmission_dct` offers `phev_system`, `phev_gearbox` and the car's own battery slot; `phev_etk_battery_10` / `_13` / `_18` offer the cells.

### 2.2 The powertrain chain

The core part `phev_system` adds `phevModule` (type `phevModule`, input `mainEngine`, index 1) and the controller row `phev`. The adapter's transmission part adds

```
["dctGearbox", "gearbox", "phevModule", 1]
```

(a `deformGroups` option is fine). The engine keeps the name `mainEngine`: the exhaust, radiator, intake, drive-select parts and the game's vehicle controller then keep working, because the module takes over the `mainEngine` lookup after the powertrain is built. Everything after the gearbox is the car's own chain (transfer case, driveshaft, differential, halfshafts).

Names the Lua looks up: devices `mainEngine`, `phevModule`, `gearbox`; the final-drive device named by `finalDriveDevice` (default `torsionReactorF`); wheel names `FL FR RL RR`; the controller `driveModes`; energy storages `mainBattery` (from the core cells) and `mainTank`.

### 2.3 The adapter keys

The core parts must not be overridden key by key: BeamNG merges a child part over its parent, so a key set by the core would win over the same key in the adapter. The core therefore holds only keys that are identical on every car, and these keys are **never** in the core. Set them in the sections `phevModule` and `phev` of the transmission part (`soundConfigPhevMotor` is its own section):

| Section | Key | Meaning | ETK port | Default if missing |
|---|---|---|---|---|
| `phevModule` | `torqueReactionNodes:` | engine nodes the motor's reaction torque acts on | `e3r e4r e2l` | none: the reaction is lost |
| `phevModule` | `warmUpHeatGain`, `coolDownLossCoef`, `coolDownHeaterLoss` | coolant heat model, scaled with the engine block's mass (block 136 kg: 58.3 / 3.16) | see the part | the module's own values |
| `soundConfigPhevMotor` | `sampleName`, `mainGain` | motor sound sample (`ElectricMotor_02` is vanilla) | `ElectricMotor_02`, -8 | no motor sound |
| `phev` | `finalDriveDevice` | device that holds the final-drive ratio | `differential_R` | `torsionReactorF` |
| `phev` | `regenAxle` | `all` (brake blending on all wheels) or `driven` (driven axle only, rear-wheel drive) | `driven` | `all` |
| `phev` | `vehicleMassKg` | fallback mass (the real mass is read at init) | 1980 | 1800 |
| `phev` | `tvEnabled`, `tvTrack` | brake torque vectoring on the inner front wheel; track width | false, 1.55 | true, 1.55 |
| `phev` | `autoLights` | the controller switches the low beam by game time of day | false | true (a car mod with its own lamp logic sets false) |
| `phev` | `modeLabels`, `chargeLabel` | display names `[electric, hybrid, boost]` and of Charge mode in the controller's messages | not set | Electric, Hybrid, Boost, Charge mode |
| `gearbox` | `gearboxNode:` | gearbox node(s) | `tra1` | none |
| `gearbox` | `gearWhine*` | gear whine events | not set | the game's |
| `vehicleController` | `clutchLaunchStartRPM`, `clutchLaunchTargetRPM` | launch clutch speeds (the engine's own idle) | 1200, 1500 | the engine's |

Do not set a key in both the core and the adapter.

### 2.4 Mass

The module adds about 34 kg, the power electronics about 53 kg and the battery pack, charger and cables about 173 kg (13 kWh) to a car. The adapter puts them on nodes of its own car; nothing else about the layout is needed, no new meshes. A mod without free nodes can add mass nodes in the transmission and battery parts and beam them to existing nodes (the vanilla ports do; see `phev_etk.jbeam`). Do not put the electronics on nodes of the engine block group: the engine's thermal mass comes from that group.

### 2.5 Drive-mode part

The drive-mode selector is the car's own part (labels, colours and the other controllers' default settings differ). Each of the three modes has one controller row:

```
["controller", {"controllerName": "phev", "phevMode": "Electric"}]    // or "Hybrid", "Boost"
```

and `driveModes.enabledModes` is `["electric", "hybrid", "boost"]` (the mode keys; the controller keeps the selector in step with its own mode changes, so these three keys must exist). Labels are free text: a car mod may name the modes anything and set `modeLabels` (2.3) to the same names for the controller's messages.

### 2.6 Input actions

Input actions are per vehicle folder in the game: ship `vehicles/<car>/input_actions_<name>.json` and `inputmaps/keyboard_<name>.json` (copy the files in `mod/vehicles/etk800/`). The actions call the controller with `onDown`:

`controller.getControllerSafe('phev').toggleElectricMode() | toggleBoostMode() | toggleChargeMode() | cycleMode() | toggleBMode() | toggleKickdown()`

## 3. Lua names a car mod may call or read

Controller `phev` (`controller.getControllerSafe('phev')`): `toggleElectricMode`, `toggleBoostMode`, `toggleChargeMode`, `cycleMode`, `setMode(0 Electric, 1 Hybrid, 2 Boost)`, `getMode`, `toggleKickdown`, `toggleBMode`, `setTorqueVectoring(on)`, `setTimeOfDay`, `setParameters({phevMode = "Electric" | "Hybrid" | "Boost"})` (called by the drive-mode selector).

Electrics the system publishes (read-only for the car): `phevMode` (0/1/2), `phevReady`, `phevSocUsable`, `phevSocRaw`, `phevBatteryKW`, `phevEnginePowerKW`, `phevMotorPowerKW`, `phevMotorTorque`, `phevIceRunning`, `phevIceStateCode`, `phevChargeFunction`, `phevBMode`, `phevEmergencyStop` (the stop lamps flash), `phevAutoLights`, and about 40 more (see the monitor app). The controller also sets the game's own `brakelights` electric (brake lamps under regeneration, the emergency stop flashing), the hazard flashers after an emergency stop, and the low beam through `electrics.setLightsState` (automatic headlights, `autoLights`). `gearIndex`, `turboBoost`, `fuel` and the others come from the game.

## 4. Worked example: the ETK 800 port

Files in `mod/vehicles/`:

| File / part | Role |
|---|---|
| `common/phev/phev_etk.jbeam` | all adapter parts of the ETK 800 and K-Series ports |
| `phev_etk_ecu` | engine data in the vanilla engine's ECU slot: `mainEngine`, torque table, turbo, `electricsThrottleFactorName`; no module, controller or strategy data |
| `phev_etk_transmission_dct` | DCT: gearbox row on `phevModule`, node mass, slots `phev_system` and `phev_gearbox`, the adapter keys of 2.3 |
| `phev_etk_battery_10`, `_13`, `_18` | the pack mass on mass nodes, petrol tank (`mainTank`, `$fuel`), slot `phev_battery` with the matching cells |
| `phev_etk_drivemodes` | modes Electric / Hybrid / Boost, keys `electric hybrid boost`, controller `phev` |
| `etk800/PHEV_ETK_*.pc`, `info_*.json` | configurations |
| `etk800/input_actions_phev.json`, `inputmaps/keyboard_phev.json` | input actions and default keys |
| `etk800/phev/phev_battery_box.dae` | battery box mesh, see [PHEV-MESHES.md](PHEV-MESHES.md) |

The other ports (`vivace`, `sunburst2`) follow the same pattern with parts in their own vehicle folders and show how a car with a different layout (front-wheel drive, adaptive dampers, a different tank part) changes the keys.

## 5. Checklist for a car mod

1. Require the Plug-in Hybrid Swap mod (mod page text; its zip name does not change between versions).
2. Engine data with `mainEngine` and `electricsThrottleFactorName "phevIceThrottleFactor"` (the module scales the pedal through it).
3. Transmission part: gearbox row on `phevModule`, the two slots, mass nodes, the adapter keys of 2.3.
4. Battery part with the `phev_battery` slot and mass nodes; keep the petrol tank part's `mainTank`.
5. Final drive: ratio on the device that `finalDriveDevice` names.
6. Drive-mode part with the three `phev` mode rows.
7. Input actions per vehicle folder, `onDown`.
8. Do not copy the core parts or the Lua into the car mod (two copies of one controller name make the game refuse the second one).

## 6. Things to verify in your mod

- That `lua/vehicle/...` from the mod zip is found by `require` in your game version (otherwise use the per-car fallback described in section 1).
- That the part tree merges as you expect in the game's loader: slot defaults, `coreSlot` handling and `variables`. Check the log for `phev_*` part errors after spawning.
