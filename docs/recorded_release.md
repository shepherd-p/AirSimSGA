# Matching the recorded experiments

The root `experiments/` directory is the configurable source-release interface.
`recorded/` preserves the final recording controllers and presentation utilities
separately, including their original machine-path assumptions. Do not execute
those historical scripts unchanged on another computer.

## Native profiles

- `main`: mapping, road navigation and tracking/containment.
- `moving-deck`: reusable contact and motor-interlock component profile.
- `recorded-landing`: final planar surge/sway/yaw vessel implementation and the
  contact/rendering source used with the reference-heading landing workflow.

Profiles are alternative overlays against the same pinned upstream commit;
never apply one profile over another. For the final landing source:

```powershell
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile recorded-landing --check-only
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile recorded-landing --destination D:/research/backend-recorded-landing
```

Follow `build.md` for compilation. `recorded/landing/build_support` also retains
the component build recipe. The UE plugin, native libraries, SDK and scene
must come from matching builds. Local build/runtime receipts must be regenerated;
the author's historical process and binary receipts are not portable credentials.

## Final landing controller and scene

The final controller/recorder is
`recorded/landing/REFERENCE_HEADING_LANDING/tools/record_water_landing.py`, with
`landing_runtime_dynamics.py` beside it. `protocol.json` records the final
configuration: 5 ms simulation steps, 0.5 m/s commanded vessel motion, corrected
bow orientation, moving-deck contact, stopped-motor dwell and observation timing.
The historical final instance used topic/service ports **9189/9190**; the reusable
landing setup in `experiments/` uses **9089/9090**. Match client and simulator ports.

To reproduce this workflow, configure the copied script's `ROOT` and `LAB`, SDK
path, output locations and simulator launch paths for your machine. The original
experiment package is under `recorded/experiments/`; the dedicated landing project
and engine installation are external. `recorded/landing/tools/create_water_assets.py`
and related repair utilities contain the scene/material construction logic.
They must run in a copied, appropriately configured Unreal project; meshes and
materials referenced by package path must be licensed and available locally.
Use the `landing --help` command to inspect recorder options before connecting.

The final water surface/wake presentation is distinct from physical wave forcing.
The recorded vessel model uses planar dynamics. Source and protocol are retained
so users can identify exactly which model and visualization they are extending.

## Mapping, urban tasks and fleets

Use `experiments/run_mapping_plaza.py`,
`experiments/citysample_scene/run_aerial_route_navigation.py`,
`experiments/citysample_scene/run_pursuit.py`, and
`experiments/urban_scene/run_scale.py` through the configurable wrappers where
available. `recorded/experiments/` retains the corresponding final source snapshot.
City Sample must be downloaded separately and its ZoneGraph exported using the
provided native exporter. See `assets.md` and `CITYSAMPLE_RESTORE.md`.

## Data

`data/paper/` contains the task records and analysis supplied with the paper;
`evidence/` includes source hashes and selected summaries. Complete raw camera
and point-cloud streams are not included. Analysis can be rerun from the included
records without launching Unreal; see `data/paper/README.md`.
