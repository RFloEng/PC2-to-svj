# PC2-to-svj

Converts the cars in **Project CARS 2** and **Project CARS** into SVJ (Standard Vehicle JSON) v0.102 files, with matching **GLB** 3D models.

You point it at your own Project CARS 2 (or Project CARS) installation. It unpacks the game's physics and vehicle archives locally, decodes them, and writes:

- one `.svj.json` per car (engine, gearbox, mass, suspension, tyres, aero, …), validated against the official SVJ v0.102 schema
- one `.glb` per car (LOD-A body, wheels, brakes, interior, with diffuse, normal and specular textures), linked from the SVJ file through SVJ visual bindings

All 212 drivable Project CARS 2 cars convert. Project CARS is supported through a game profile (`--edition pc1`, see below).

> **Not affiliated with or endorsed by the developers or publishers of Project CARS or Project CARS 2.**
> This tool only reads files from a copy of the game you own. It doesn't bypass online protection and doesn't modify the game. The data it extracts belongs to the game's rights holders. Don't redistribute the extracted files or generated models.

## Requirements

- Windows, with a Steam install of Project CARS 2
- Python (developed and tested with 3.13), with `tkinter` for the GUI
- Python packages:
  ```bash
  pip install numpy pygltflib pillow scipy jsonschema
  ```
  `numpy` and `pygltflib` are required for 3D models. `pillow` adds textures, `scipy` is optional, and `jsonschema` enables schema validation.
- [PCarsTools 1.1.4](https://github.com/Nenkai/PCarsTools/releases/tag/1.1.4) by Nenkai, which decrypts the game's archives. Place it at:
  ```
  _pcars2/tools/win-x64/PCarsTools.exe
  ```
- A .NET runtime, version 7 or newer. PCarsTools targets .NET 6; the converter lets it run on a newer runtime automatically.
- The [SVJ specification](https://github.com/RFloEng/SVJ-standard-vehicle-json) (v0.102), unpacked to `svj_spec/SVJ-standard-vehicle-json-main/`. The converter ships the standard's suspension migration in `svj_migrate/`; the specification folder provides the schema for validation.

You don't need to download anything else. PCarsTools expects an Oodle DLL named `oo2core_7_win64.dll`, and the converter copies the game's own `oo2core_4_win64.dll` under that name.

## Usage

### One command

```bash
python convert_all.py
```

Finds the game, unpacks the physics archives (first run only), converts every car's physics, extracts each car's 3D model, collision hulls and hinges, validates the result and writes `batch_summary.csv`. Output goes to one folder per game in the SVJ library, `<library>/Project CARS 2/<car>/`. The library is `$SVJ_LIBRARY`, else the first line of `svj_library.txt` beside the script (ignored by git), else a `svj_library/` folder beside the script. Useful options: `--game "<path>"`, `--car toyota_gt86` (repeatable, `*` patterns allowed), `--no-meshes` (physics only), `--resume` (skip cars that already have a GLB), `--out <folder>`. The SVJ specification is downloaded into `svj_spec/` if missing (it is used for schema validation and the collision overlap check).

### Automobilista 2

Automobilista 2 uses the same file formats as Project CARS 2, so the same pipeline converts it:

```
python convert_all.py --edition ams2        # or --game "D:/Steam/steamapps/common/Automobilista 2"
```

Output goes to `<library>/Automobilista 2/`. What differs from Project CARS 2:

- **The cars are the archives.** Most of the physics of an AMS2 car lives only in its own archive (`Pakfiles/Vehicles/<Car>.bff`), under file names that differ from the archive's (`Porsche_963_GTP.bff` holds `porsche_963.*`). The cockpit, livery and low-detail (`_LD`) archives are skipped; 426 cars are converted.
- Engine curve points store the rpm as a float, and spec sheets (`.mrdf`) are 224 bytes with the same field offsets. The sheet's peak torque is in N.m, not lb-ft: against it the engine curve peaks at 1.00 on naturally aspirated cars. Turbo cars read about 0.7 because the curve is the unboosted one. The engine file also holds a boost map (torque multiplier per rpm at five boost levels, kept in `engine.x_boost_map`); the file does not say which level a car runs, so where one level's peak equals the spec sheet's (within 5 %, 64 of 144 cars with a map) that level is applied and the unboosted curve is kept as `x_unboosted_torque_curve`, flagged. The other 80 keep the unboosted curve.
- **Tyres** are found through the car's descriptor (`.vdfm`), which lists as plain strings the files its car loads (clutch, turbo, failure set, tyres): the string that matches a `.hdtbin` by name is the tyre file. 22 cars name none and use defaults, flagged. Cars whose archive has no descriptor of its own (`formula_retro`, `rally_kart`) get an estimated wheelbase and track.

### Project CARS (the first game)

```bash
python convert_all.py --edition pc1
```

The edition is also inferred when `--game` points at a folder holding `pCARS.exe`. Output goes to `<library>/Project CARS/<car>/`.

What is different from Project CARS 2:

- **Extraction.** Its archives use Xbox compression, so PCarsTools has to be the 32-bit build (`_pcars2/tools/win-x86/PCarsTools.exe`) with `--game-type PC1`, and Windows needs the [Microsoft XNA Framework 4.0 Redistributable](https://www.microsoft.com/en-us/download/details.aspx?id=20914) (the tool calls its native library). Its models are not encrypted.
- **Where the files are.** The chassis, engine, gearbox and suspension files are in `PHYSICSPERSISTENT.bff` (and in each car's own archive, which again holds the files the car loads), the spec sheets (`.mrdf`) in `HRDFPERSISTENT.bff` and the tyres (`.hdtbin`) in `PHYSICSBOOTFLOW.bff`. `convert_all.py` unpacks all three. The spec sheet and tyre files are read with the Project CARS 2 readers: the spec sheet gives mass, drivetrain and rear weight share (the Ford Focus RS comes out FWD, the Audi A1 quattro and Lancer Evo IX AWD), and the tyres give Pacejka D, C and E.
- **Wheel centres, wheelbase, track and tyre size** come from the car's `.vhf` and tyre meshes, because the `.vdfm` descriptors of Project CARS have another layout that is not decoded.
- **Cars without a spec sheet or tyre file** (4 and 13 of the 119) use the chassis file's mass (all mass except fuel, driver included), the drivetrain from the differentials that are on, and the default tyre, all flagged as estimates.
- **Not available:** engine displacement, gearbox type and clutch (no `driveline.rg`), and the engine position, which stays front-engine (`FR`) for RWD cars.
- **Not cars:** `ford_duratec` and `jaguar_30_v6` have engine files but no car archive; they are skipped. 119 cars convert.

Everything read from the chassis, engine, gearbox, suspension and collision files uses the same decoders as Project CARS 2.

### GUI

```bash
python pc2_to_svj_gui.py
```

1. The game folder is detected automatically from common Steam library paths, or you can browse to it.
2. Click **Extract physics** once. This unpacks the physics archives into `_pcars2/out/`.
3. **Convert** a single car, or **Batch** convert all of them.
4. **Meshes** extracts the 3D models for every converted car (about 25 minutes for all 212).

### Command line

```bash
# Physics -> SVJ (expects physics already extracted into _pcars2/out)
python pcars2_to_svj.py --dir _pcars2/out --list
python pcars2_to_svj.py --dir _pcars2/out --car toyota_gt86 --out gt86.svj.json
python pcars2_to_svj.py --dir _pcars2/out --all --outdir "<library>/Project CARS 2"

# 3D models -> GLB, and link each one into its SVJ file
python pcars2_extract_meshes.py --game "<path to Project CARS 2>" --all --cars-dir "<library>/Project CARS 2"
```

To extract the physics archives without the GUI, see [PCARS2_EXTRACTION_NOTES.md](PCARS2_EXTRACTION_NOTES.md).

### Output

```
_pcars2/
  out/            unpacked physics archives
  cars/
    <car>/        <car>.svj.json and <car>.glb, side by side
```

Run the physics conversion again after extracting models and the mesh links are kept.

## What's real and what's estimated

Every value is either decoded from the game files or a flagged estimate. Estimates are marked with `_est` keys, `_note` keys, and the `_known_placeholders` list in `_metadata`.

| Data | Source |
|---|---|
| Torque curve, idle / max RPM | engine file (`.edfbin`) |
| Gear ratios, final drive | gearbox file (`.gdfbin`): the gear settings of the chassis file pick the ratios out of the gearbox file's list, and the final-drive setting times the bevel gives the final drive (every car: 212 of 212 in PC2 and 119 of 119 in PC1; the gear count equals the spec sheet in 210 and 115). The settings index the file's list in its stored order, from its end, or, for race cars that list every ratio they can be fitted with, in descending order of size; cars with more than seven gears have no stored setting for the extra gears and use the file's list as it is. Shift time is the game's upshift delay. Checked on the GT86 against the setup screen |
| Mass, drivetrain (RWD/AWD/FWD), displacement, front/rear weight split | car-select spec sheet (`.mrdf`) |
| Wheel centres, wheelbase, track, tyre width and radius | vehicle descriptor (`.vdfm`) |
| Gearbox type, clutch type and torque capacity, crankshaft inertia, AWD diff layout (205 of 212 cars) | driveline setup (`driveline.rg`), matched by car name |
| Spring and damper rates, default camber, toe and caster, tyre pressure | chassis file (`.cdfbin`): range start + default setting x step, times the file's spring and damper multipliers (wheel rates). 202 of 212 cars; the rest are estimated and flagged `_est`. The slow and fast damper slopes meet at the transition force the file gives. Spring, damper, camber, toe, caster, ride height, steering ratio, brake bias and gear ratios were checked against the game's setup screen for the GT86 and match; the spring and damper multipliers (taken as motion-ratio terms) are not checked |
| Differential type and lock | chassis file (`.cdfbin`): Torsen bias ratios, clutch preload, ramp angles and plate count; the GT86's rear differential (type, 1.5:1 and 4.0:1, 45 and 45 degrees, 2 plates, 40 Nm preload) matches the setup screen. Cars with several types on follow an assumed priority (flagged) |
| Anti-roll bar rate | chassis file (`.cdfbin`), checked on the GT86 (55 and 20 N/mm). `bar_rate` is the roll stiffness that wheel rate gives across the track, arm geometry being unknown |
| Inertia, CG fore/aft, steering ratio and lock, brake torque and bias | chassis file (`.cdfbin`). The inertia axis order is inferred; the CG agrees with the spec-sheet weight split in 148 of 208 cars (within 1 %) |
| CG height | chassis file (`.cdfbin`) `CGHeight` plus the mean ride height, in 209 cars. The file names no reference plane; the underside of the body at ride height fits best: against Assetto Corsa's CG height for 52 cars matched by name and mass it is 2 cm low on average (5 cm scatter), while `CGHeight` alone is 12 cm low |
| Aero model | chassis file (`.cdfbin`): body, wing, diffuser, radiator and brake-duct fields kept by name in `aerodynamics.x_pcars2.aero_model`. The game stores no drag coefficient (forward drag is computed at run time), so `Cd` and `Cl` stay flagged placeholders. Against Assetto Corsa's body drag area for 52 matched cars, `BodyDragBase` correlates at only r = 0.29, so no scale is claimed |
| Turbocharger or supercharger units (106 PC2 and 48 PC1 cars) | turbo file (`.tbfbin`), kept in `engine.x_turbo`: size, turbine optimum rpm, engine rpm, inertia, friction, fuel percentage per unit, in the game's units (field names from the community TBF documentation) |
| Wheel and upright mass and inertia | suspension file (`.sdfbin`), 126 cars; the others keep estimates |
| Suspension hardpoints and links (184 cars on all four corners, 27 on some) | suspension file (`.sdfbin`), placed on the `.vdfm` wheel centres |
| Tyre Pacejka D, C, E | tyre file (`.hdtbin`) |
| **Estimated:** suspension geometry where it couldn't be decoded, Pacejka B, brake disc size, steering rack geometry, Cd and Cl, engine position of RWD cars, differential type | — |

**Which files a car uses.** Each car's own archive (`Pakfiles/Vehicles/<Car>.bff`) holds the chassis, engine, gearbox, suspension, turbo and collision files that car really loads, including ones it shares with another car (the Acura NSX 2017 loads `acura_nsx_gt3.sdfbin`, the Aston Martin DB11 `aston_martin_vantage_gt4.sdfbin`). The converter keeps that folder under `_pcars2/car_physics/<car>/` and uses it first; the common physics archives hold only some of these files, so looking them up by the car's name left 65 cars without a suspension file and 31 without a gearbox file. Tyre files are not in the car archive: they are matched by name, and a car with no tyre file of its own falls back to a true base variant of the same car when one exists (such as `toyota_gt86_rb` using `toyota_gt86`), otherwise to defaults. A different model's data is never borrowed.

**Suspension** is written in the unified model of SVJ spec §25 (v0.100+): bodies, joints and spring/damper elements in `chassis.mass_bodies` and `constraints`, plus `wheel_stations[]`. The converter builds the per-corner geometry first, then runs the spec's own migration tool; everything it generates is marked `x_derived`.

**Hinged parts and collision proxies** come from the plain-text collision XML in each car's vehicle archive (`pcars2_extract_meshes.py`, with `--vhf-only` for a fast pass without meshes). Each convex hull becomes a box proxy (`collision[]`, spec §28); parts with a hinge descriptor (bonnet, boot, bumpers, …) become bodies with the game's mass, a revolute joint and an `<part>_open` pose (§24). The extraction saves `<car>.collision.json` beside the SVJ file and later physics conversions reapply it. Masses, box inertias and the closed-pose angle are flagged estimates; the hinge axis is oriented so a positive angle lifts the part.

**Coordinates** follow SVJ's SAE J670 convention: X forward, Y right, Z down, origin at the front-axle midpoint on the ground. Units are SI, except engine displacement, which the SVJ spec gives in litres.

**3D models** follow SVJ spec 22.4: glTF Y-up, +X right, -Z forward, with the origin at the front-axle midpoint on the ground, so a GLB lines up with the SVJ's hardpoints without any extra transform. The game's mesh space is left-handed, so the conversion mirrors it (and reverses triangle winding). Each part is placed using the game's own scene hierarchy (`.vhf`), so wheels, discs, calipers and the steering wheel sit where they do in the game. Parts are grouped under nodes named by the SVJ naming convention (`SVJ::body::chassis`, `SVJ::wheel::wheel_fl`, `SVJ::brake::disc_fl`, `SVJ::steering::wheel`, …), and the SVJ file binds only the nodes its GLB actually contains.

## Known limitations

- Beam and live axles are written as an axle body that the decoded link records locate (`solid_axle`, 22 PC2 and 13 PC1 cars). The axle body mass is an estimate, and where a corner's link records are too few to locate the axle the game's own layout is kept as is. PC1 chassis files carry no differential records and PC1 has no `driveline.rg`, so PC1 differentials stay estimated. Neither game stores a drag coefficient.
- Project CARS (PC1) data has not been compared with the game's setup screens. Geared top speed at the rev limiter is a median 0.91 (PC2) and 0.82 (PC1) of the spec-sheet top speed, and is no precise check: the sheet's speed also depends on drag and on the gear set the sheet assumes.
- Tyre models have inverted normals in the game's own source data, so tyres may look flat-shaded.
- Textures are capped at 512 px. Specular maps are approximated: gloss becomes roughness, and the specular colour uses the optional `KHR_materials_specular` extension as a reflectance mask.
- Suspension geometry comes from the game's link records. The converter infers `system_type` and link names from the layout: double wishbone, MacPherson, multi-link, trailing arm or semi-trailing arm. Some strut cars are stored by the game as a long virtual upper arm, and those links carry a `_note`. Corners the game data leaves one link short get a flagged estimated toe or upper link so every corner has the 5 constraints it needs; trailing and semi-trailing arms end on `wheel_center`; a MacPherson strut runs from its top mount to an estimated lower point on the upright, and its spring and damper mounts follow it. Torsion beams, unusual layouts and the cars whose suspension file lacks wheel-position records keep a representative double-wishbone estimate, flagged `_est`.

## Project layout

| File | Purpose |
|---|---|
| `convert_all.py` | One-command pipeline for either game: unpack, convert, models, validate |
| `pc2_to_svj_gui.py` | GUI |
| `svj_migrate/` | The SVJ standard's suspension migration, vendored (refresh with `vendor_svj_migrate.py`) |
| `pcars2_to_svj.py` | Physics → SVJ converter |
| `pcars2_extract_meshes.py` | Per-car model extraction pipeline |
| `pcars2_chassis.py` | Chassis file (`.cdfbin`) reader: springs, dampers, inertia, alignment, brakes, steering |
| `pcars2_hashnames.json` | Hash → field name map for those files |
| `pcars2_collision.py` | Collision XML → hinged parts, joints, poses and collision proxies |
| `meb_to_glb.py` | Builds GLB files from `.meb` mesh parts |
| `meb_reader.py` | `.meb` mesh format reader |
| `shcb_reader.py` | Reader for the `ShCB` physics container (`.edfbin`, `.gdfbin`, …) |
| `pcars2_names.py` | Matches car names to the game's differently named files |

## Credits

- [PCarsTools](https://github.com/Nenkai/PCarsTools) by Nenkai, for archive extraction and model decryption. The `.meb` reader follows its binary templates.
- The Project CARS / AMS2 chassis translation by JDougNY, revised for AMS2 by GvsE ([AMS2 modding docs](https://github.com/mikevapour/ams2-modding-docs)). `pcars2_hashnames.json` maps the chassis file's hashed field names from it, and the field encoding in `pcars2_chassis.py` follows it.

## License

Copyright 2026 RFloEng. Licensed under the [Apache License, Version 2.0](LICENSE).
