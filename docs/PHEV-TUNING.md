# Plug-in Hybrid Swap: tuning per car, battery sizes and petrol tank

How each vanilla car is adjusted for the hybrid hardware: mass, brakes, springs, gearbox geometry, battery sizes and the petrol tank. Everything marked **assumption** is a number chosen, not measured; the checklist at the end says what to look at in the game.

The tables below were generated from the shipped parts, so they match them. The text around them is hand-written.

## 1. Overview

| Item | What the mod does |
|---|---|
| Battery | 10, 13 and 18 kWh parts per host body in the same slot; **13 kWh is the default of all seven configurations** |
| Cell data | core parts `phev_battery_10`, `_13` and `_18`. Each has its own slot type `phev_cells_<kWh>`; the host's pack part allows exactly its own cells, so a pack and its cells cannot be mixed up. The slot name `phev_battery` is the same for all |
| Petrol tank | vanilla tank part kept; `mainTank.fuelCapacity` overridden by the battery part (38 L at 13 kWh) |
| Brakes | `$brakestrength` default raised by the mass ratio |
| Springs | parallel extra spring beams carry the extra load at the vanilla ride height |
| Warm-up scaling | the hybrid module's coolant corrections scale with the engine block mass of each host |

## 2. Method

**Vanilla car.** For each row the "vanilla car" is the vanilla configuration the port started from (`854_190_A`, `844_250_A`, `kc4_250_A`, `vivace_160_DCT`, `ardente_230_DCT`, `sport_EU_DCT`, `sport_EU_wagon_DCT`), merged with an emulation of the game's slot system (`lua/common/jbeam/slotSystem.lua`: child before parent, a later part wins).

**Mass and axle split.** Node masses of the merged tree, front and rear load by the lever rule between the hub nodes `fh1r` and `rh1r` (so a battery at y 0.8-1.0 m puts about 80 % of its mass on the rear axle). The jbeam sum misses wheels, tyres, fuel and the parts a lenient loader does not resolve; those are the same in the vanilla car and ours and are taken as `measured - node sum` from a spawn test in the game, split 50/50. The node-mass difference is exactly the hybrid hardware (34 kg motor + 53 kg electronics + 173 kg pack), so swapping the engine and gearbox parts is mass-neutral.

**Brakes.** Every vanilla brake part sets `brakeTorque` as `$brakestrength * N`, and `$brakestrength` (default 1.0, range 0.6-1.0, 0.4 on the coupe, 0.2 on the Sunburst) is declared in each car's main part. Deceleration per pedal travel is torque over mass, so the factor is the total mass ratio ours / vanilla: **1.16-1.19 at 13 kWh**. The battery part declares `$brakestrength` again with the new default and the range 0.5-1.5. Variables from later parts overwrite earlier ones of the same name in the game's collection (`variables.lua`, `varMerge` and `_sanitizeVars`). One factor for both axles: the bias stays the vanilla one, and the pack puts the extra mass behind the middle, so the rear axle gets more than its share of the extra braking capacity (rear share of load +1.0 to +1.8 points). Fade and ABS thresholds are not touched. The controller reads each wheel's live `brakeTorque` for the blending, so it follows the new values.

**Springs.** The vanilla standard spring parts have fixed rates (ETK rear 33,000 N/m, preload range 0.121 m) and **no** `$spring_*` / `$springheight_*` variables; only the race and rally spring parts do. So the battery part adds a second spring beam in parallel to every vanilla spring beam (the same two nodes, by name) with the extra rate `(s - 1) * k`. Static force is rate times preload range, so the preload range of the extra beams is the extra static load of the axle (our node mass on the axle minus the vanilla car's, lever rule) divided by the summed extra rate of the axle: the extra beams carry exactly the extra load, the car keeps its ride height, and the ride frequency stays because rate and sprung mass grow together. The derived ranges are 10-14 % below the vanilla rows' ranges (ETK rear 0.104 m against 0.121 m), because the vanilla spring's rate times range is a little above the load it carries. `s` is the sprung load ratio per axle, with the unsprung share of an axle's load set to 12 % (**assumption**), motion ratio taken as 1. The beams' `beamDeform` is the force of the extra spring at its preload plus 0.40 m of travel (far beyond the stroke, so it deforms only in a crash) and `beamStrength` is 8 times that; both are own numbers. Nothing of the vanilla spring row is used but the node pair and, as an input to `s`, its rate.

**Dampers: not changed.** Rate and mass grow by `s`, damping does not, so the damping ratio falls by 1/s (about -13 % front, -18 % rear on the ETK at 13 kWh). The adaptive dampers of the Cherrier and the damper geometry of the Sunburst (spring and damper on different node pairs) make a parallel damper less certain; if the car feels floaty at the rear, the same method with the vanilla shock rows (`beamDamp`, `beamDampRebound`, `beamDampFast` scaled by `s - 1`) is the next step.

**Gearbox geometry.** The transmission adapters replace the vanilla transmission part, so they bring the gearbox nodes, their ties to the engine and (Cherrier) a collision hull. Positions are offsets from the engine block's faces (800-Series and Sunburst: node `tra1` 0.60 / 0.68 m behind the rear face; Cherrier: `tra1l`, `tra1r`, `em1l` and two new nodes `phev_vivace_gbc_r/_l` beside the block), within a few cm of where the vanilla part had them. Tie stiffness is the node mass and a natural frequency (140 Hz, 200 Hz, 400 Hz), damping ratio 0.02, deformation limit the carried mass at 90 g. Node masses (gearbox 76 / 74 kg with the motor, case 25 kg on the Cherrier) are unchanged, so the totals of the tables below do not move. Names that stay because vanilla parts require them: `tra1`, `tra1l`, `tra1r`, `em1l`, the node groups `etk_transmission`, `vivace_transmission`, `sunburst2_transaxle`. **To check in the game:** idle and drive-away vibration, gearbox damage group, halfshaft joints (Cherrier) after hard cornering.

**Final drive.** Set per car (3.75 scaled by tyre radius, so the engine speed at a given road speed is the same on every car). Tyres: vanilla.

**Warm-up scaling.** The hybrid module's coolant corrections (`warmUpHeatGain`, `coolDownLossCoef`, `coolDownHeaterLoss`) scale with the engine block mass (sum of the `engine_block` nodes) against the 168 kg block of the reference engine the corrections were fitted on: ETK and K-Series 0.81 (136 kg), Sunburst 0.69, FCV and Ardente 0.52. **Assumption**: the native engine model's own heating is not scaled; refit after a warm-up test.

**Battery sizes.**

- *Mass:* the real 13 kWh pack is 135 kg with its management (press data for the 9.9 kWh pack, assumed the same at 13 kWh). **Assumption:** 70 % of that is cells (13 kWh in 94.5 kg = 138 Wh/kg on cell level, normal for NMC prismatic plug-in cells) and scales with energy; 30 % is housing, cooling plate, busbars and BMS and scales with energy^(2/3). The rest of the pack part (33.5 kg charger, HV cables and other parts; 38 kg on the Cherrier) does not depend on the size. The mass sits on the same nodes, only scaled.
- *Cell data:* the usable window (22.5-90 %), cell voltage curve, BMS maps and speed limits stay those of the 13 kWh pack (same cell generation). Peak discharge power grows slower than the energy because the 85 kW motor is the limit anyway (80 / 90 / 100 kW, **assumption**); charge power scales with the energy (31 / 40 / 55 kW) but the strategy's `regenPowerLimitKW` (40 kW) still caps regeneration; internal resistance scales with 1/energy; heat capacity with the energy; housing and coolant conductance with energy^(2/3).

<!-- sizes:start -->
| Size | Usable window (kWh) | Peak discharge / charge (kW) | Resistance (ohm) | Heat capacity (kJ/K) | UA passive / coolant (W/K) | Pack proper (kg) | Tank from 50 L (L) |
|---|---|---|---|---|---|---|---|
| 10 kWh | 2.2 - 9.0 (6.8 usable) | 80 / 31 | 0.169 | 100 | 12.6 / 50.4 | 106.7 | 39.5 (-21.2 %) |
| 13 kWh | 2.9 - 11.7 (8.8 usable) | 90 / 40 | 0.13 | 130 | 15 / 60 | 135.0 | 38 (-24.2 %) |
| 18 kWh | 4.0 - 16.2 (12.2 usable) | 100 / 55 | 0.094 | 180 | 18.6 / 74.5 | 181.2 | 35.5 (-29.3 %) |
<!-- sizes:end -->

**Petrol tank.** The vanilla tanks are all 50 L. The tank part stays; the battery part carries a `mainTank` section with `fuelCapacity`, which the slot system merges over the vanilla tank part's value (the body's tank slot is merged before the engine / transmission subtree that holds the battery; the emulation confirms it for all seven configurations). The loss for the battery is linear in the pack energy through two published points: a 66 L petrol car against 50 L for the hybrid at 13 kWh (-24.2 %), and 50 L against 40 L at 8.8 kWh (-20 %, a published figure not re-checked). The game clamps the starting fuel to the capacity (`fuelTank.lua`: `min(startingFuelCapacity, capacity)`), so the car starts full at the new size. The vanilla `$fuel` slider (0-50 L) still goes up to 50, a cosmetic leftover: values above the capacity are clamped. The fuel gauge shows the fraction.

## 3. The numbers used (generated)

<!-- tables:start -->
| Config (13 kWh) | Node mass vanilla / ours (kg) | + hardware (kg) | Measured (kg) | Front / rear load vanilla (kg) | Front / rear load ours (kg) | Rear share vanilla -> ours |
|---|---|---|---|---|---|---|
| PHEV_ETK_wagon | 1387 / 1647 | +260 | 1859 | 799 / 800 | 904 / 955 | 50.0 % -> 51.3 % |
| PHEV_ETK_sedan | 1366 / 1626 | +260 | 1838 | 801 / 778 | 906 / 932 | 49.3 % -> 50.7 % |
| PHEV_ETKC_coupe | 1291 / 1551 | +260 | 1782 | 800 / 722 | 902 / 880 | 47.5 % -> 49.4 % |
| PHEV_FCV | 1346 / 1606 | +260 | 1784 | 900 / 624 | 1024 / 760 | 40.9 % -> 42.6 % |
| PHEV_Ardente | 1309 / 1569 | +260 | 1748 | 865 / 623 | 992 / 756 | 41.9 % -> 43.3 % |
| PHEV_Sunburst | 1130 / 1390 | +260 | 1601 | 778 / 563 | 908 / 693 | 42.0 % -> 43.3 % |
| PHEV_Sunburst_wagon | 1147 / 1407 | +260 | 1618 | 773 / 585 | 904 / 714 | 43.0 % -> 44.1 % |

| Battery part | Used by | Pack mass (kg) | Brake `$brakestrength` | Front spring x | Rear spring x | Extra rates (N/m, each wheel) | Tank (L) | Spread between configs (brake / F / R) |
|---|---|---|---|---|---|---|---|---|
| `phev_etk_battery_10` | PHEV_ETK_wagon, PHEV_ETK_sedan | 144.7 | 1.15 | 1.142 | 1.190 | F 3700, R 6300 | 39.5 | 0.002 / 0.000 / 0.005 |
| `phev_etk_battery_13` | PHEV_ETK_wagon, PHEV_ETK_sedan | 173.0 | 1.16 | 1.150 | 1.222 | F 3900, R 7300 | 38 | 0.002 / 0.000 / 0.006 |
| `phev_etk_battery_18` | PHEV_ETK_wagon, PHEV_ETK_sedan | 219.1 | 1.19 | 1.163 | 1.276 | F 4200, R 9100 | 35.5 | 0.003 / 0.000 / 0.008 |
| `phev_etkc_battery_10` | PHEV_ETKC_coupe | 144.7 | 1.15 | 1.139 | 1.211 | F 3600, R 5100 | 39.5 | 0.000 / 0.000 / 0.000 |
| `phev_etkc_battery_13` | PHEV_ETKC_coupe | 173.0 | 1.17 | 1.145 | 1.248 | F 3800, R 5900 | 38 | 0.000 / 0.000 / 0.000 |
| `phev_etkc_battery_18` | PHEV_ETKC_coupe | 219.1 | 1.20 | 1.156 | 1.309 | F 4100, R 7400 | 35.5 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_fcv_10` | PHEV_FCV | 144.7 | 1.15 | 1.149 | 1.206 | F 3000, R 2900 | 39.5 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_fcv` | PHEV_FCV | 173.0 | 1.17 | 1.157 | 1.248 | F 3100, R 3500 | 38 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_fcv_18` | PHEV_FCV | 219.2 | 1.20 | 1.168 | 1.315 | F 3400, R 4400 | 35.5 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_ardente_10` | PHEV_Ardente | 144.7 | 1.16 | 1.159 | 1.202 | F 5100, R 10100 | 39.5 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_ardente` | PHEV_Ardente | 173.0 | 1.17 | 1.167 | 1.242 | F 5300, R 12100 | 38 | 0.000 / 0.000 / 0.000 |
| `phev_vivace_battery_ardente_18` | PHEV_Ardente | 219.2 | 1.21 | 1.180 | 1.308 | F 5800, R 15400 | 35.5 | 0.000 / 0.000 / 0.000 |
| `phev_sunburst_battery_10` | PHEV_Sunburst, PHEV_Sunburst_wagon | 144.6 | 1.17 | 1.178 | 1.218 | F 5300, R 5600 | 39.5 | 0.002 / 0.001 / 0.008 |
| `phev_sunburst_battery_13` | PHEV_Sunburst, PHEV_Sunburst_wagon | 173.0 | 1.19 | 1.191 | 1.256 | F 5700, R 6600 | 38 | 0.002 / 0.001 / 0.009 |
| `phev_sunburst_battery_18` | PHEV_Sunburst, PHEV_Sunburst_wagon | 219.1 | 1.23 | 1.212 | 1.319 | F 6300, R 8200 | 35.5 | 0.003 / 0.001 / 0.012 |

| Config | Sag without the extra springs (cm, front / rear) | Engine block (kg) | Warm-up scale | Final drive | Tank vanilla -> ours (L) |
|---|---|---|---|---|---|
| PHEV_ETK_wagon | 2.0 / 2.3 | 136 | 0.81 | 3.64 | 50 -> 38 |
| PHEV_ETK_sedan | 2.0 / 2.3 | 136 | 0.81 | 3.64 | 50 -> 38 |
| PHEV_ETKC_coupe | 1.9 / 3.2 | 136 | 0.81 | 3.64 | 50 -> 38 |
| PHEV_FCV | 3.0 / 4.8 | 88 | 0.52 | 3.53 | 50 -> 38 |
| PHEV_Ardente | 1.9 / 1.3 | 88 | 0.52 | 3.58 | 50 -> 38 |
| PHEV_Sunburst | 2.1 / 2.5 | 116 | 0.69 | 3.53 | 50 -> 38 |
| PHEV_Sunburst_wagon | 2.1 / 2.5 | 116 | 0.69 | 3.53 | 50 -> 38 |
<!-- tables:end -->

The wagon and sedan (ETK and Sunburst) share one battery part, so their factors are the mean of the two (the last column shows they differ by less than 1 %). 'Sag without the extra springs' is the spring-only estimate `(extra axle load) * g / 2 / k` at motion ratio 1; the tyre adds a little compliance.

## 4. Assumptions and open points

1. Pack mass split (cells 70 % linear, structure energy^(2/3)); fixed 33.5 kg.
2. Unsprung share of an axle's load (12 %) and motion ratio 1 in the spring factors.
3. Peak power, resistance, heat capacity and conductance of the 10 and 18 kWh packs; the usable window is that of the 13 kWh pack.
4. Tank loss for 10 and 18 kWh (interpolated on two published points, one not re-checked).
5. `$brakestrength` redeclared in a later part wins in the game's variable collection (read from `variables.lua`).
6. Brakes: one factor for both axles from the total mass ratio.
7. Warm-up scale by block mass ratio, the same method for all ports.
8. Dampers unchanged (damping ratio falls by 1/s).
9. Extra spring beams: only the node pairs of the vanilla spring rows are used, by name. The preload range is derived from our own axle loads and comes out 10-14 % below the vanilla rows' ranges, so the ride height may be a few mm lower than the vanilla car. Deformation and break limits are own choices (0.40 m of travel, factor 8).

## 5. What to check in the game

1. **Load.** Spawn the seven configurations: no errors in the log for `phev_*` parts; `phev_battery_13` and the host battery part are in `v.data.activeParts`. Open the parts window: the host battery slot offers 10 / 13 / 18 kWh; after changing it the nested "Battery Cells" slot follows to the matching cells without a "wrong slot type" error.
2. **Mass.** Sum of node masses at spawn against the 'Measured' column (13 kWh, 50 L): about 9 kg less with the 38 L tank. 10 kWh: about -28 kg, 18 kWh: about +46 kg.
3. **Brake variable.** `v.data.variables['$brakestrength']`: default 1.16 (ETK) ... 1.19 (Sunburst), max 1.5. If it still reads default 1.0 and max 1.0, the later declaration did not win: the fall-back is to put the factor into the configuration's `vars` (clamped to 1.0, so it would then need replacement brake parts).
4. **Brake feel.** Same pedal travel (0.3 and 0.6) from 100 km/h, vanilla donor configuration against ours: deceleration within about 5 %. Full braking: no early rear lock-up with ABS on.
5. **Ride height.** Flat ground at rest: hub node height (`fh1r`, `rh1r`) against the vanilla donor configuration, within about 5 mm (it would be 20-50 mm lower without the extra springs). Bounce the car by hand or drive over a kerb: it should settle in about two oscillations; if it floats, scale the dampers (section 2).
6. **Tank.** `energyStorage.getStorage('mainTank').capacity` is 38 and the car starts with 38 L (not 50), 39.5 L with the 10 kWh and 35.5 L with the 18 kWh pack.
7. **Battery sizes in use.** 10 kWh: Boost power a little lower (pack limit 80 kW); 18 kWh: longer electric range; `phevBatteryKW` never above the pack's peak limit.
8. **Warm-up.** Cold start of an ETK configuration: coolant warm-up time against the vanilla car and against the FCV.
