# City Sample air-ground experiments

The completed navigation and right-exit containment runs use City Sample's
`Big_City_LVL`, its exported native ZoneGraph, and the isolated ProjectAirSim
plugins under `F:/EpicGamesEnv/CitySample/Plugins/`. The UE 5.2 and ProjectAirSim
core builds are not rebuilt by these scripts. Start one simulator instance and
run one experiment client at a time. Ambient vehicle and pedestrian spawning is
disabled for the controlled runs.

From PowerShell, with `F:/EpicGamesEnv/ProjectAirSim_AG_Experiments` as the
current directory:

```powershell
& .\citysample_scene\start_citysample_simulator.ps1 -Map Big -Visible -ControlledTraffic
```

Wait for the City Sample scene to load. If a compatible instance is already
running on ports 8989 and 8990, use it instead of starting another. In a second
PowerShell window in the same directory:

```powershell
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\run_navigation.py
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\run_pursuit.py --seed 0
```

The first command completes before starting the second. Each creates a dated
folder in `outputs/citysample_navigation/` or `outputs/citysample_pursuit/`.
Check `summary.json` for `status: completed` and empty `collision_events`.
The same folder contains the camera PNG and original NPZ packets, controls and
trajectories, events, scenario, provenance hashes, and a source snapshot.

Reference results for the manuscript are navigation
`20260930_042724_885397_air` (UGV 144.107 m) and pursuit
`20260930_041328_715126_seed0` (right-exit target 91.315 m). Only the seeded
right-exit City Sample containment run is claimed as completed. Repeating a run
may produce different results and should be assessed from its own summary.

Paper panels are regenerated from those fixed result folders with
`paper_RAL/make_figures.py`, followed by `paper_RAL/make_overview.py` and
`paper_RAL/collect_paper_figures.ps1`. The camera boxes and perspective route
overlays are derived from saved packets and poses; they are not simulator HUD
graphics.

The multi-intersection district navigation experiment is under validation and
must not replace the completed 144 m manuscript run yet. Its current City Sample
scenario uses two independently observed road closures. Run it with:

```powershell
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\prepare_district_navigation.py
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\run_district_navigation.py
```

Each attempt is in `outputs/citysample_district_navigation/`; only a run whose
`summary.json` reports `status: completed`, two decisions, and no collision or
unmodeled impulse can be used as a completed demonstration. The best two-decision
attempt, `20260930_084659_347353_air_ground`, contains both live UAV observations,
two replans, camera and raw sensor packets, and 164.2 m of UGV motion. It ended
`off_route` after an unreported 15.8 m/s physics impulse and is not a successful
navigation result. A same-direction lane change attempt,
`20260930_090101_640526_air_ground`, ended sooner with a 24.2 m/s impulse;
that unvalidated lane change was removed from the scenario. Native City Sample
road/prop collisions currently prevent a verified end-to-end run. No district
navigation figures or metrics from these attempts are used in the manuscript.
`check_depth_projection.py` compares archived depth packets
against evaluation object poses; these poses are never passed to the UGV planner.

For a vehicle-visible record of the completed single-closure bypass, run the
existing navigation client with `--vehicle-evidence`. It archives three
time-stamped external-camera PNG/NPZ pairs in the new run's `vehicle_evidence/`
directory. After confirming `summary.json` reports `completed` and no collisions,
generate the cropped sequence with:

```powershell
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\annotate_vehicle_bypass.py .\outputs\citysample_navigation\<run_directory>
```

The reference vehicle-visible run is `20260930_183535_434159_air`. Its UGV
traveled 143.874 m and passed the closure at a 4.37 m center distance. The
derived annotation manifest hashes the source frames and records the projected
UGV boxes. This does not validate the separate multi-intersection district run.

The separate aerial-route study is intended to make the UAV scout three road
junctions before the UGV drives the resulting route. It keeps the single-closure
bypass above as a separate, completed experiment. With the City Sample simulator
already running, prepare and execute the aerial-route study using:

```powershell
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\prepare_aerial_route_navigation.py
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\run_aerial_route_navigation.py
```

The western route is 483.87 m on City Sample's directed vehicle ZoneGraph. The
UAV first records a high-altitude RGB view and a near-altitude RGB/depth/
segmentation observation at each of its three junctions. Road-surface visibility
is checked against the graph elevation before the shortest-path route is
committed to UGV control. The images validate the surveyed junctions; they do
not independently recover road topology or choose among blocked alternatives.
The UGV then follows the recorded waypoints under simple-drive control.

The reference run is
`outputs/citysample_aerial_route_navigation/20261001_023924_720614_air_ground`.
Its `summary.json` reports `completed`: three UAV observations, three UGV
junction crossings, 478.89 m of measured UGV travel, a 2.59 m maximum route
deviation, a 2.86 m/s maximum sampled displacement speed, and no recorded
non-ground collision. Each aerial depth/segmentation pair has a 0 ms timestamp
gap. Raw camera packets, trajectories/controls, the committed plan, scenario,
source hashes, and an exact source snapshot are in the run folder. Repetitions
must be judged from their own summaries.

The run uses an experiment-only UGV configuration with central contact-isolation
links at `z=-0.45 m` because the original wheel-ground feedback produced pose
jumps in City Sample. It demonstrates route execution with wheel-control
kinematics and recorded poses, not validated wheel-ground contact dynamics.
This generated configuration does not alter the compiled ProjectAirSim build or
the completed single-closure run. Failed contact diagnostics remain under
`outputs/citysample_ugv_pose_diagnostic/` for audit.

To regenerate the two marked figure candidates from that completed run:

```powershell
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\annotate_aerial_route_navigation.py .\outputs\citysample_aerial_route_navigation\20261001_023924_720614_air_ground
& 'C:\Users\Stephen\.conda\envs\projectairsim310\python.exe' .\citysample_scene\audit_aerial_route_navigation.py .\outputs\citysample_aerial_route_navigation\20261001_023924_720614_air_ground
```

They appear in the run's `derived/` directory with
`annotation_manifest.json`; `audit_report.json` records the independent
camera/trajectory/source checks. The camera panels are original RGB frames with
projected route and approximate vehicle-pose boxes; the marks are postprocessing,
not RGB detector output. The older central-district attempt
`20260930_234606_813440_air_ground` remains an unvalidated diagnostic and is
not evidence of completed multi-junction traversal.
