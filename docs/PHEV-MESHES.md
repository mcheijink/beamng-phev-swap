# Plug-in Hybrid Swap: meshes (the battery pack box)

The mod adds one mesh per host car: a flat battery housing in the rear seat base. The inverter box and the motor ring are not modelled. The meshes were generated from nothing: no vanilla geometry is copied.

## 1. What was built

| Item | Value |
|---|---|
| Meshes | `phev_battery_box_etk` (ETK 800 wagon and sedan), `phev_battery_box_etkc` (K-Series coupe), `phev_battery_box_fcv`, `phev_battery_box_ardente`, `phev_battery_box_sunburst` (sedan and wagon). One box per battery part; the bodies of one part share it |
| Files | `vehicles/etk800/phev/phev_battery_box.dae`, `vehicles/etkc/phev/...`, `vehicles/vivace/phev/...` (FCV and Ardente), `vehicles/sunburst2/phev/...`. One .dae per vehicle folder, so a car only meets its own materials. Nothing vanilla is overwritten |
| Shape | per box about 136 triangles: a lower tray and a cover (8 mm chamfers, the cover 8 mm inset), three 3 mm ribs on the cover, an HV connector block (7 x 3.5 x 2 cm) on the front face. All inside the box of the table below except the connector |
| Material | vanilla only, no new textures and no new material file: ETK wagon, sedan: `etk800`; coupe: `etkc`; FCV, Ardente: `vivace_mechanical`; Sunburst: `sunburst2_mechanicals`. They are the materials of the host's own fuel tank, subframe and underbody meshes (the dark chassis atlas). All vertices map to a small planar patch round the centre of the bottom face of the host's own tank mesh. **Assumption:** the texel there is plain dark metal; the textures could not be decoded when the meshes were made, so the look is not verified |
| Flexbody | every battery part (10, 13 and 18 kWh, all five host parts) gets `"flexbodies": [["mesh", "[group]:", "nonFlexMaterials"], ["phev_battery_box_<host>", ["<group>"]]]`. The group is that of the part's own mass nodes: `phev_etk_battery` (ETK and coupe), `phev_vivace_battery` (FCV, Ardente) and `phev_sunburst_battery` (Sunburst) |
| Sizes | the 10 and 18 kWh parts use the same box: the volume of the seat base is the limit, not the energy |

## 2. Where it goes, and why not under the floor

Target: a pack under the rear seat or the boot floor, roughly 1.0 x 0.6 x 0.2 m (about 120 L), without lowering the ground clearance and without clipping the floor, tank, exhaust, spare wheel or rear suspension.

**The vanilla underbody has no such volume.** The floor, tank, exhaust and suspension meshes of all seven configurations were checked (side views and vertical ray casts through the .dae triangles). Result at x = 0.3 m beside the tunnel, in front of the tank (the only place under the cabin floor that is not tank):

| Pack part | Lowest vanilla underside line (z, m) | Floor plate in front of the tank (z) | Free height under the floor above that line |
|---|---|---|---|
| ETK wagon, sedan | 0.176 | 0.221 | 0.045 |
| K-Series coupe | 0.158 | 0.215 | 0.057 |
| Cherrier FCV | 0.173 | 0.216 (underbody cover) | 0.043 |
| Cherrier Ardente | 0.164 | 0.179 (underbody cover) | 0.015 |
| Sunburst, wagon | 0.151 | 0.200 | 0.049 |

Behind that plate the fuel tank fills the full height (z 0.16-0.43, one wall to the other, under the rear seat). Further back are the diff and subframe. The rear overhang is crowded as well (wagon: spare-wheel-style well in the load floor, x +-0.25, y 1.68-2.2, bottom z 0.29; muffler, bumper and diffuser; the pocket is only about 0.3-0.5 m wide). A 0.2 m box under the floor would hang 15 cm below the vanilla underside or clip the floor and the tank. The vanilla electric Cherrier solves it by replacing the tank (its battery part sits in the tank slot); this mod keeps the vanilla tank part, so that way is closed without copying vanilla parts.

**What is free:** the base of the rear seat, between the cabin floor (carpet over the tank) and the top of the seat cushion. It is where the mass nodes already are (y 0.6-1.1 m), it is hidden by the seat as the pack is in the real car, and it clips none of the listed meshes. Its size is only 0.05-0.15 m tall; a fit on a 1 cm grid, 12 mm to the floor and under the seat top, gives the box of the table below.

**Consequences:**

- The boxes are **small**: 8-55 L (bounding volume, table in section 3) against about 120 L in the real car (the thin rear seat of the Cherrier hatchbacks holds a plate of 4-5 cm). The mass is unchanged (173 kg on the existing nodes); only the picture is a flat housing. A real pack is flat too, but this is not the 13 kWh volume.
- They are **invisible** with the seat in place. In the game they show with the seat part removed or damaged, in a deep crash, and (to be confirmed) when the part is highlighted in the parts window.
- The mass nodes were not moved ([PHEV-TUNING.md](PHEV-TUNING.md) depends on them); the mesh follows them.
- A visible pack under the car would need a lower underside (worse ground clearance), or the tank replaced by a part of the mod's own. Neither was done.

## 3. Placement and clearances

Box = x, y, z of the housing in vehicle coordinates (x left positive, y rearward positive, z up). Distances are the minimum from the pack's solids (tray, cover, ribs, connector) to the triangles of the vanilla meshes of that configuration (flexbody `pos` applied; static, unloaded). Negative would mean inside. The check fails below 3 mm, or if the box bottom is below the vanilla underside line, or if any part of the footprint is not under the seat's top surface by at least 8 mm. "Spare wheel": none of the seven configurations has one (`sparetire` and `sparewheel` parts exist only on the Sunburst's rally and off-road options).

| Config | Pack mesh | Box x (m) | Box y (m) | Box z (m) | Volume (L) | Floor (mm) | Tank (mm) | Exhaust (mm) | Spare wheel (mm) | Rear suspension (mm) | Lowest vanilla underside z / box bottom z (m) | Under the seat top by (mm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| PHEV_ETK_wagon | `phev_battery_box_etk` | -0.60 .. 0.60 | 0.62 .. 1.00 | 0.428 .. 0.548 | 55 | 23 (intcarpet) | 37 (fueltank) | 156 (exhaust_R) | none in this config | 42 (subframe_R) | 0.176 / 0.428 | 14 |
| PHEV_ETK_sedan | `phev_battery_box_etk` | -0.60 .. 0.60 | 0.62 .. 1.00 | 0.428 .. 0.548 | 55 | 23 (intcarpet_sedan) | 37 (fueltank) | 156 (exhaust_R) | none in this config | 42 (subframe_R) | 0.176 / 0.428 | 14 |
| PHEV_ETKC_coupe | `phev_battery_box_etkc` | -0.50 .. 0.50 | 0.82 .. 1.06 | 0.552 .. 0.730 | 43 | 24 (etkc_body) | 151 (etkc_fueltank) | 266 (etkc_exhaust_R) | none in this config | 79 (etkc_subframe_R) | 0.158 / 0.552 | 28 |
| PHEV_FCV | `phev_battery_box_fcv` | -0.60 .. 0.60 | 0.54 .. 0.82 | 0.444 .. 0.494 | 17 | 10 (carpet) | 45 (fueltank) | 171 (exhaust) | none in this config | 92 (torsionbeam) | 0.173 / 0.444 | 11 |
| PHEV_Ardente | `phev_battery_box_ardente` | -0.50 .. 0.50 | 0.40 .. 0.60 | 0.414 .. 0.452 | 8 | 10 (ardente_body_interior) | 15 (ardente_fueltank) | 141 (ardente_exhaust) | none in this config | 257 (trailingarm_R) | 0.164 / 0.414 | 11 |
| PHEV_Sunburst | `phev_battery_box_sunburst` | -0.50 .. 0.50 | 0.54 .. 0.92 | 0.406 .. 0.474 | 26 | 12 (interior_main) | 33 (fueltank) | 186 (exhaust_pipe) | none in this config | 139 (trailingarm_mount_R) | 0.151 / 0.406 | 15 |
| PHEV_Sunburst_wagon | `phev_battery_box_sunburst` | -0.50 .. 0.50 | 0.54 .. 0.92 | 0.406 .. 0.474 | 26 | 12 (interior_main) | 33 (fueltank) | 186 (exhaust_pipe) | none in this config | 139 (trailingarm_mount_R) | 0.151 / 0.406 | 15 |

Notes: floor = body shell, carpet, underbody covers, heat shield, panels (the nearest mesh is named in brackets); rear suspension = subframe, arms, springs, shocks, sway bar, diff, half shafts, hubs and brakes. The ground clearance is unchanged by construction: the lowest point of the box is 0.23-0.40 m above the vanilla underside line (second last column). The seat gap is the smallest distance between the top of the box and the first surface of the seat (the coupe: the carpet shelf behind the front seats) above it.

## 4. Assumptions and open points

1. The UV patch of the atlas (section 1) is not verified; if the pack looks wrong (paint-coloured, glossy, striped), another UV patch of the same material is the fix.
2. The boxes are placed on the vanilla meshes as they are at rest. Flexbody deformation, seat deformation and suspension travel are not in the check.
3. The seat meshes are open surfaces; "under the seat top" means the first seat surface above the box, found with vertical rays on a 2 cm grid.
4. The coupe has no rear seat: its box sits behind the front seats under the carpet shelf at z 0.78.
5. The ETK and K-Series battery parts live in `vehicles/common/phev/phev_etk.jbeam` and both offer their slot type to both cars; on an ETK 800 the coupe's battery part (and the other way round) would point at a mesh that is not in that car's folder (log warning only).

## 5. What to check in the game

1. **Load.** Spawn the seven configurations: no errors in the log for `phev_battery_box_*` (mesh found, material found). The `.dae` of the car's own folder is read; the log must not complain about missing materials.
2. **Where is it.** Remove the rear seat part (or use the part highlight in the parts window): a dark flat box under the rear seat cushion, between the carpet and the cushion, not poking through either. ETK: 1.2 m wide, 0.36 m long, 0.12 m tall.
3. **Look.** Colour and finish: dark metal, not body paint, no stripes (UV patch, section 4 point 1).
4. **Follows the body.** Drive over a kerb, brake hard, roll the car: the box stays in the seat base and does not wobble or stretch (flexbody on the mass-node group only).
5. **No clipping.** Ground clearance and the underside unchanged (compare with the donor car from below); nothing of the box visible through floor, seat or tank.
6. **Sizes.** 10 and 18 kWh show the same box.
7. **Collision.** None: flexbodies have no collision; the car must drive and crash as before (the mass nodes are unchanged).
