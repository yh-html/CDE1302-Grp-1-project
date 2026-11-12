# Frame CAD

Everything for the 3D-printed frame. Before 12 Nov 2026 this lived in `CAD/` (Fusion) and
`codex/` (code-native CadQuery projects).

**Flown design:** the instructor's stock frame. Its STLs are course material and now live in
`C:\cde1302_course\frame_design\Actual Mod STL of drone frame\` (outside the repo).
None of the designs below were flown.

| Folder | What it is | State |
|---|---|---|
| `fusion/` | Fusion 360 first draft: 3MF exports (`exports/`, `exports/rev1/`), screenshots, `scripts/fusion_helpers.py` | superseded |
| `drone_cad_v1/` | Codex's Astra 01 CadQuery frame (was `codex/drone_cad`) | dropped |
| `drone_cad_v2/` | Claude's copy of Astra with review changes, under-nose camera, guard rev B | dropped |
| `drone_cad_v3/` | Astra v3 rework | untracked, dropped |
| `stock_mounts/` | Camera and flow-sensor holders for the unmodified stock parts | untracked |
| `stock_plus/` | Stock frame plus camera cheeks, dowels, TPU skid shoes, thicker ribs | untracked |
| `PROJECT_SUMMARY.md` | 10 Oct review of the Fusion draft and what was rejected | historical |

The Astra build prompt is archived at `docs/archive/ASTRA_DRONE_FRAME_PROMPT.md`.

## Paths to know after the move

- The CadQuery Python env is still at `C:\cde1302\codex\drone_cad\.venv` (local, not in git).
  It was not moved. `drone_cad_v1/build.ps1` makes its own `.venv` beside itself if missing.
- `stock_mounts/` and `stock_plus/` code loads the stock STLs from
  `C:\cde1302\Actual Mod STL of drone frame`, which is now in `C:\cde1302_course\frame_design\`.
  Point `STL` in `src/parts.py` / `src/stock_parts.py` (and `compare_frame.py`) there before rebuilding.
- The `legacy_exports_untouched` check in `drone_cad_v1/`, `drone_cad_v2/` and `drone_cad_v3/`
  looks for `C:\cde1302\CAD\exports`, which is now `cad/fusion/exports`, so that check will
  report missing files until it is repointed.
