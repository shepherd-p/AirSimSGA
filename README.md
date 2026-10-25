# AirSimSGA

**A Unified Sea-Ground-Air Simulation Platform for Heterogeneous Unmanned Systems**

AirSimSGA supports cooperative robotics experiments with unmanned aerial,
ground and surface vehicles. It provides heterogeneous observation/control
interfaces, surface-vehicle extensions, moving-support contact and actuator
interlocks, and configurable cooperative workflows.

## Demonstrations

| Workflow | Configuration | Reusable entry point |
| --- | --- | --- |
| Cooperative 3-D mapping | CityPark; UAV + UGV | `python -m airsimsga run mapping` |
| Aerial-assisted road navigation | City Sample; UAV + UGV, multiple junctions | `python -m airsimsga run navigation` |
| Target tracking and exit containment | City Sample; 2 UAVs + 2 UGVs + target | `python -m airsimsga run containment` |
| Visual moving-vessel landing | Dedicated water scene; UAV + USV | See [final landing workflow](docs/recorded_release.md) |

Concurrent-fleet configurations and numerical resource-use records are also included.
The example algorithms can be replaced through the sensing and control interfaces.

## Get started

The recorded environment uses Windows, Unreal Engine 5.2 and Python 3.10.
Use separate build and scene directories to preserve existing installations.

```powershell
git clone https://github.com/shepherd-p/AirSimSGA.git
cd AirSimSGA
python -m pip install -e ".[landing,reconstruction]"
python -m airsimsga list
Copy-Item settings/local.example.json settings/local.json
# Set SDK, scene and build paths in settings/local.json.
python -m airsimsga doctor --task navigation
```

`list` and `doctor` do not start the simulator or move vehicles. Running a task
requires a separately built simulator, matching SDK and the corresponding scene.

1. [Build the simulation extensions](docs/build.md).
2. [Obtain and configure scenes/assets](docs/assets.md).
3. [Run the four workflows](docs/experiments.md).
4. [Match the final recorded experiments](docs/recorded_release.md).
5. [Numerical data and analysis](data/paper/README.md).

## Repository layout

- `airsimsga/`: public Python CLI and surface-vehicle interface.
- `experiments/`: portable task controllers, configurations, reconstruction and tests.
- `native/`: pinned native source overlays, component builds and tests.
- `recorded/`: controller, configuration and presentation source from the final local recordings.
- `data/paper/`: task trajectories, numerical records and analysis for the manuscript.
- `evidence/`: selected experiment summaries and historical native test results.
- `docs/`: architecture, build, asset and reproduction instructions.
- `QA/`: recorded source-package checks; see `QA/public_release_checks.json` for this release.

## Dependencies and license

AirSimSGA additions are provided under the MIT license. The simulation backend is
built using the attributed, pinned [Project AirSim](https://github.com/iamaisim/ProjectAirSim)
source and the native extensions in this repository. Original copyright and
license notices are retained in [LICENSE](LICENSE), [licenses](licenses/) and
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

Unreal Engine, City Sample, CityPark, third-party meshes and pretrained models
must be obtained separately under their own licenses. This repository contains
source code and numerical records, not simulator binaries or a complete raw
RGB-D/LiDAR archive. See [asset requirements](docs/assets.md).

[中文说明](README.zh-CN.md) · [Architecture](docs/architecture.md) · [Citation](CITATION.cff)
