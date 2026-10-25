# Reproduction guide

## Inspect numerical records without a simulator

The task trajectories, observations, events, configurations and resource samples
are in `data/paper/`. Recompute the numerical results with:

```powershell
python data/paper/tools/recompute_task_metrics.py
python data/paper/tools/recompute_archive_diagnostics.py
```

These two analysis scripts use Python's standard library. Read `data/paper/README.md`
for the data layout and definitions. Complete raw RGB-D/LiDAR payloads are external;
reconstruction of the full surface requires the raw sensor archive.

## Inspect source and run offline checks

```powershell
python -m airsimsga check-release
python tools/run_offline_tests.py
python tools/materialize_backend.py --upstream D:/research/ProjectAirSim --profile recorded-landing --check-only
```

Install the optional landing/reconstruction dependencies for the full offline
suite. `QA/public_release_checks.json` records checks performed for this release;
other QA records retain their original timestamps and describe earlier checks.

## Run simulations

Follow `build.md`, `assets.md`, `experiments.md` and `recorded_release.md` to build
the backend, obtain scenes, configure paths and launch the corresponding task.
The final landing and main experiment builds have different ports and source
profiles. Use new output directories when collecting data.

`evidence/index.json` retains source/released hashes for selected records;
`data/paper/` provides the numerical dataset paired with the current manuscript.
Historical filesystem paths inside metadata describe provenance and do not need
to exist to execute the bundled numerical-analysis scripts.
