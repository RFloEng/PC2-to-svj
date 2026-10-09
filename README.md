# PC2-to-svj

Converts the cars in **Project CARS 2** into SVJ (Standard Vehicle JSON) v0.97 files, with matching **GLB** 3D models.

You point it at your own Project CARS 2 installation. It unpacks the game's physics and vehicle archives locally, decodes them, and writes:

- one `.svj.json` per car (engine, gearbox, mass, suspension, tyres, aero, …), validated against the official SVJ schema
- one `.glb` per car (LOD-A body, wheels, brakes, interior, with diffuse textures), linked from the SVJ file

All 212 drivable cars convert.

> **Not affiliated with or endorsed by the developers or publishers of Project CARS 2.**
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
- *Optional:* the SVJ specification (`SVJ-standard-vehicle-json`), unpacked to `svj_spec/SVJ-standard-vehicle-json-main/`, for schema validation. Without it, validation is skipped and reported as such.

You don't need to download anything else. PCarsTools expects an Oodle DLL named `oo2core_7_win64.dll`, and the converter copies the game's own `oo2core_4_win64.dll` under that name.

## Usage

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
python pcars2_to_svj.py --dir _pcars2/out --all --outdir _pcars2/svj_output

# 3D models -> GLB, and link each one into its SVJ file
python pcars2_extract_meshes.py --game "<path to Project CARS 2>" --all --svj-dir _pcars2/svj_output --glb-dir _pcars2/meshes
```

To extract the physics archives without the GUI, see [PCARS2_EXTRACTION_NOTES.md](PCARS2_EXTRACTION_NOTES.md).

### Output

```
_pcars2/
  out/            unpacked physics archives
  svj_output/     <car>.svj.json
  meshes/         <car>.glb
```

Run the physics conversion again after extracting models and the mesh links are kept.

## What's real and what's estimated

Every value is either decoded from the game files or a flagged estimate. Estimates are marked with `_est` keys, `_note` keys, and the `_known_placeholders` list in `_metadata`.

| Data | Source |
|---|---|
| Torque curve, idle / max RPM | engine file (`.edfbin`) |
| Gear ratios, final drive | gearbox file (`.gdfbin`) |
| Mass, drivetrain (RWD/AWD/FWD), displacement, front/rear weight split | car-select spec sheet (`.mrdf`) |
| Wheel centres, wheelbase, track, tyre width and radius | vehicle descriptor (`.vdfm`) |
| Spring rates | suspension file (`.sdfbin`) |
| Tyre Pacejka D, C, E | tyre file (`.hdtbin`) |
| Drag coefficient | chassis file (`.cdfbin`); tentative |
| **Estimated:** suspension geometry other than the wheel centre, damper curves for road cars, Pacejka B, brakes, steering, CG height, engine position of RWD cars, differential type | — |

Cars that don't ship their own physics file (for example, many have no tyre file of their own) fall back to the file of a true base variant of the same car when one exists, such as `toyota_gt86_rb` using `toyota_gt86`. Otherwise they use defaults. A different model's data is never borrowed.

**Coordinates** follow SVJ's SAE J670 convention: X forward, Y right, Z down, origin at the front-axle midpoint on the ground. Units are SI, except engine displacement, which the SVJ spec gives in litres.

**3D models** use glTF's Y-up convention. Each part is placed using the game's own scene hierarchy (`.vhf`), so wheels, discs, calipers and the steering wheel sit where they do in the game.

## Known limitations

- Tyre models have inverted normals in the game's own source data, so tyres may look flat-shaded.
- Only diffuse textures are embedded, capped at 512 px. Normal and specular maps are ignored.
- Suspension link geometry isn't stored by the game, which models kinematics with lookup tables instead. The links in the SVJ files are representative estimates.

## Project layout

| File | Purpose |
|---|---|
| `pc2_to_svj_gui.py` | GUI |
| `pcars2_to_svj.py` | Physics → SVJ converter |
| `pcars2_extract_meshes.py` | Per-car model extraction pipeline |
| `meb_to_glb.py` | Builds GLB files from `.meb` mesh parts |
| `meb_reader.py` | `.meb` mesh format reader |
| `shcb_reader.py` | Reader for the `ShCB` physics container (`.edfbin`, `.gdfbin`, …) |
| `pcars2_names.py` | Matches car names to the game's differently named files |

## Credits

- [PCarsTools](https://github.com/Nenkai/PCarsTools) by Nenkai, for archive extraction and model decryption. The `.meb` reader follows its binary templates.

## License

Copyright 2026 RFloEng. Licensed under the [Apache License, Version 2.0](LICENSE).
