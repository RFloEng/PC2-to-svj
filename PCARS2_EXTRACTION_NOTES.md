# Project CARS 2 → SVJ: Extraction Feasibility (VERDICT: ✅ FEASIBLE)

Investigation date: 2026-06-02. Game install inspected:
`E:\SteamLibrary\steamapps\common\Project CARS 2`

## TL;DR
Unlike rFactor (loose editable text files), pCARS2 (Madness engine) packs **all**
physics into **encrypted + Oodle-compressed `PAK` archives** (`*.bff`). They cannot be
read from a stock install directly. However, extraction is a **solved problem** using
the community tool **PCarsTools** (Nenkai), which decrypts + decompresses + recovers
original file names. I extracted the physics archives end-to-end and confirmed every
SVJ-relevant data category is present.

## The reproducible extraction recipe
1. Tool: **PCarsTools 1.1.4** (C#/.NET 6), prebuilt single-file exe:
   `https://github.com/Nenkai/PCarsTools/releases/download/1.1.4/PCarsTools.exe.zip`
   → unzipped to `_pcars2/tools/win-x64/PCarsTools.exe`
2. **Oodle DLL fix:** PCarsTools P/Invokes `oo2core_7_win64.dll`, but pCARS2 ships
   `oo2core_4_win64.dll`. The v4 codec decodes pCARS2 streams fine, so just copy the
   game's own DLL under the expected name (no untrusted download):
   `cp ".../Project CARS 2/oo2core_4_win64.dll" win-x64/oo2core_7_win64.dll`
3. **.NET:** machine has runtimes 7/8/10 but no .NET 6 and no SDK → run with
   `DOTNET_ROLL_FORWARD=Major` (works). (No source build possible — no SDK.)
4. Extract a physics pak (pCARS2 is the DEFAULT key set; no `--game-type` needed):
   ```
   DOTNET_ROLL_FORWARD=Major ./PCarsTools.exe pak \
     -i ".../Project CARS 2/Pakfiles/PHYSICSPERSISTENT.bff" \
     -g ".../Project CARS 2" -o <outdir>
   ```
   Result: 638/638 files extracted, full original paths recovered. Likewise
   `PHYSICSMENU.bff` → 831/831 files.

## Where the physics lives (per-archive)
- **`Pakfiles/PHYSICSPERSISTENT.bff`** (638 files): shared/common physics
  - `vehicles/physics/vehicles/*.vdfm` (249) — per-car master descriptor (~600 B; references + key scalars)
  - `vehicles/physics/tyres/*.hdtbin` (per-car tire params) + `tires/hrdf/tire.bin` (261 KB shared tire model DB)
  - `vehicles/physics/driveline/driveline.rg` (105 KB, **plain text**)
  - `vehicles/physics/statistics/*.mrdf` (252) — display stats
  - `vehicles/physics/ffb/*.txt` (**plain text**, Lisp S-exprs) — FFB, not core SVJ
- **`Pakfiles/PHYSICSMENU.bff`** (831 files): the detailed per-car setup physics
  - `.edfbin` (212) — **Engine** (torque curve)        → SVJ drivetrain.engine
  - `.cdfbin` (227) — **Chassis** (mass/CG/aero/geom)   → SVJ chassis + aerodynamics
  - `.gdfbin` (180) — **Gearbox** (ratios/final/diff)   → SVJ drivetrain.gearbox/diff
  - `.sdfbin` (145) — **Suspension** (springs/dampers)  → SVJ suspension
  - `.tbfbin` (46), `.bbfbin` (21) — brakes/body/turbo-related
- `Pakfiles/PHYSICSBOOTFLOW.bff` (tiny) — boot flow only.

→ Every SVJ section maps to a pCARS2 source. Coverage is complete.

## File formats & parser difficulty
- **Text** (`.rg`, `.txt`): driveline, FFB. Trivial.
- **`ShCB` binary container** (`.edfbin/.cdfbin/.sdfbin/.gdfbin/.hdtbin`): one regular,
  chunk-based format for the whole family (magic `ShCB`, const `E6 0F DD 53`, chunk tag
  `RI 86 55`). Holds IEEE-754 float records / lookup tables (torque curves, suspension
  rates) that read out directly. **One reader covers all five.**
- **`Q\x02\x01\x0X` binary container** (`.vdfm`, `tire.bin`, `.mrdf`): header + 64-bit
  offset table + typed float/int/string blocks. Second reader.
- **Main RE challenge:** property names inside `ShCB` are stored as **4-byte hashes**
  (e.g. `8B 0A B7 71`), not plaintext. Need a hash→name map (community `.edfbin`↔text
  tools already exist) OR positional/structural decoding (values are readable floats, so
  this works for curves/tables).

## Constraints / honesty
- Requires the user's own legitimate install (offline, static unpack — not a runtime or
  anti-cheat bypass; EasyAntiCheat only matters for online play).
- QuickBMS `nfsshift.bms` is the *old* path; it broke on pCARS2's Sept-2017 retail key
  change. PCarsTools supersedes it. Don't rely on QuickBMS for retail pCARS2.

## Suggested converter architecture (when greenlit)
`pcars2_to_svj.py` mirroring `rf_to_svj.py`'s section-builder design:
1. `ShCBReader` + `MrdfReader` (binary container parsers)
2. Per-section extractors: engine(.edfbin), chassis/aero(.cdfbin), gearbox/diff(.gdfbin),
   suspension(.sdfbin), tires(.hdtbin + tire.bin)
3. Coordinate/unit normalization to SVJ (SI), reusing existing Pacejka/SVJ builders.
4. Optional: integrate extraction (shell out to PCarsTools) so the user points at the
   install and gets SVJ directly; or consume a pre-extracted folder.
