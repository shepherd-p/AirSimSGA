> For the final planar-dynamics, reference-heading landing run, use the exact runtime source and entry points in [README_CURRENT.md](../README_CURRENT.md). The generic landing commands below describe the reusable moving-deck interface.

# Four demonstrations

Run from this extracted source repository using editable installation; a Python
adapter wheel alone does not contain the examples/native sources. Prepare the
independent backend and assets, then start the corresponding simulator yourself.

```powershell
python -m pip install -e ".[landing,reconstruction]"
python -m pip install -e D:/research/backend-main/client/python/projectairsim
Copy-Item settings/local.example.json settings/local.json
# Edit SDK, scene and receipt paths in local.json.
python -m airsimsga list
python -m airsimsga doctor --task navigation
```

Settings paths are relative to the JSON file. `list`, `doctor` and `check-release`
do not connect or move robots. `run` loads the scene and controls robots. Main
ports are 8989/8990; landing is 9089/9090. Host/port environment overrides are
`AIRSIMSGA_HOST`, `AIRSIMSGA_PORT_TOPICS`, `AIRSIMSGA_PORT_SERVICES`.

## Mapping: CityPark plaza / main plugin

```powershell
python -m airsimsga run mapping -- --capture-only
python -m airsimsga run mapping -- --reconstruct D:/data/mapping/<run> --no-view
```

Without capture-only, it captures then reconstructs. Capture-only does not need
Open3D. Reconstruction requires Open3D; `reconstruction_python` can select a
separate environment. Raw packets, RGB-D, trajectories and controls are recorded;
`plaza_results` contains cloud/surface/trajectory figures and viewer. Alignment
uses simulator poses and is not standalone SLAM. Surface rendering is a derived
product. Use `--output-dir D:/data` to choose the capture output root.

## Aerial route assistance: City Sample / main plugin

```powershell
python -m airsimsga run navigation -- --max-sim-seconds 700
```

The UAV surveys three intersections before UGV execution of the committed native
ZoneGraph route. The recorded paper example drove 478.47 m. Known-map routing
and UAV observations are distinguished from purely visual road reconstruction.
Collision/discontinuity/time guards retain failed evidence.

## Tracking and exit containment: City Sample / main plugin

```powershell
python -m airsimsga run containment -- --seed 0 --max-sim-seconds 180
python -m airsimsga run containment -- --seed 0 --max-sim-seconds 220 --evidence-views
```

Two UAVs, two interceptor UGVs and one Target use segmentation/depth observations.
Evidence-views changes camera configuration AND target speed: it is a separate
protocol. The current paper and video use the selected containment run indexed in evidence/index.json. The evader follows a scripted route, not a learned
policy. Generated config actor IDs GateLeft/Right denote interceptor UGVs.

## Visual moving-deck landing: dedicated landing plugin

```powershell
python -m airsimsga doctor --task landing
python -m airsimsga run landing -- --speeds 0.5 --repeats 1 --probe
python -m airsimsga run landing -- --speeds 0.5 --repeats 1
```

Probe verifies perception without touchdown descent. Use the correct marker and
geometry plus current build/runtime receipts. Success requires actual continuous
support, relative-motion bounds and physical rotor shutdown over a 3 s dwell;
hovering near the deck is not success. Original consumed camera payloads, pose,
commands, collision/support and rotors are logged. `--output D:/data/new-suite`
chooses a NEW output directory. Static/moving examples are not a statistical
success-rate estimate. Keep failed/partial records.

The `output_root` setting redirects urban outputs; mapping and landing keep their
runner-specific explicit output arguments. See `assets.md` for road export and
scene compatibility, and `build.md` for constructing the two profiles.
