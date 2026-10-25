# AirSimSGA supplementary numerical records and analysis

This supplement contains recorded task data, configurations, reconstruction reports, and offline analysis scripts supporting the manuscript. It does not require a simulator for inspecting the numerical records.

## Contents

- `evidence/mapping/`: archived mapping trajectories, acquisition indices, task settings, reconstruction reports, and audit snapshots.
- `evidence/navigation_*`: road-navigation trajectories, committed plan, configuration, and derived cross-track errors.
- `evidence/tracking_*`: target and interceptor trajectories, observations, events, configuration, and derived errors.
- `evidence/landing_*`: moving-USV landing states, configuration, events, support, rotor and camera-age diagnostics.
- `evidence/fleet/`: six recorded runs with matched camera settings, resource samples, configuration and provenance.
- `tools/recompute_task_metrics.py`: recompute trajectory and estimation metrics from bundled CSV/JSON files using only Python's standard library.
- `tools/recompute_archive_diagnostics.py`: recompute fleet resource and terminal landing diagnostics using only Python's standard library.

Run from any working directory:

```text
python tools/recompute_task_metrics.py
python tools/recompute_archive_diagnostics.py
```

The scripts write reports under `evidence/`. The camera-age metric uses simulation time. Camera message Hz and encoded video FPS have different definitions.

The raw LiDAR and RGB-D payloads and simulator binaries are retained in the full research archive rather than duplicated in this numerical supplement. Mapping quality values can be checked against the included recorded diagnostic report; rebuilding the TSDF surface requires the full sensor archive and the reconstruction environment. Simulation reruns additionally require the corresponding runtime and third-party scene assets.

Some original metadata contains historical filesystem paths. These are provenance fields, not dependencies for the included offline scripts. They do not need to exist on the reader's machine.

The mapping video is an additional execution of the same survey configuration. The mapping figures and numerical results use the archived dataset represented by `evidence/mapping/`; videos 2--5 correspond to the reported navigation, containment, and landing recordings.
