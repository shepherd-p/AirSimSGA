# Independent urban scene for ProjectAirSim studies

This scene is a new UE 5.2 level in CityPark1: `/Game/AGUrban/UrbanBase`.
`/Game/CityPark/Maps/Showcase` remains the project's startup/default map.
The compiled ProjectAirSim plugin and the existing CityPark experiments are unchanged.

## Data and license

- Area: Seattle South Lake Union, OSM API bounding box
  `-122.341,47.621,-122.333,47.627` (west, south, east, north).
- Raw OSM extract: [OpenStreetMap API](https://api.openstreetmap.org/api/0.6/map?bbox=-122.341,47.621,-122.333,47.627),
  copyright OpenStreetMap contributors, ODbL 1.0. Attribute OpenStreetMap in
  any published map or scene figure, and keep the ODbL notice with distributed
  derived map data.
- Converter: [OSM2World 0.4.0](https://osm2world.org/download/), MIT license.
- Mesh optimizer: [meshoptimizer gltfpack v1.3](https://github.com/zeux/meshoptimizer/releases/tag/v1.3), MIT license.
- `source/provenance.json` records the raw and clipped SHA-256 hashes and the
  exact clipping rule. `generated/road_network_report.json` records the model
  hash, road graph and OSM-to-GLB registration accuracy.

The OSM API returns some ways extending beyond the requested rectangle.
`prepare_osm.py` retains nodes inside the box and only complete ways, dropping
incomplete relations. OSM2World reports a recoverable polygon exception for
this extract, so the generated model must be visually checked for missing
surfaces. The converted model is about 660 x 720 m, with 79 retained building
ways and 147 retained motor-road ways. These are source-data counts, not a
claim that every building or road surface is correct in UE.

## Generated files

- `generated/south_lake_union_clipped.glb`: source 3D conversion, 2,101 mesh
  primitives and 41,337 triangles.
- `generated/south_lake_union_runtime.glb`: gltfpack version with 27 mesh
  primitives and the same triangle count. It retains the material textures.
- `generated/gltfpack_report.json`: optimizer's original report.
- `generated/road_network_report.json`: 146 public drivable way segments, 408
  graph nodes; the largest connected component contains 395 nodes and 72
  intersections with degree >= 3. The registered OSM junctions have 0.056 m
  median and 0.713 m 95th-percentile center residual in the GLB coordinate
  system. This does not establish vehicle clearance or collision behavior.

## Rebuild and preview

From `F:\EpicGamesEnv\ProjectAirSim_AG_Experiments`:

```powershell
& 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe' urban_scene/prepare_osm.py
Set-Location urban_scene/tools/osm2world_0_4_0
java --add-exports java.base/java.lang=ALL-UNNAMED --add-exports java.desktop/sun.awt=ALL-UNNAMED --add-exports java.desktop/sun.java2d=ALL-UNNAMED -Xmx4G -jar OSM2World.jar -i ../../source/south_lake_union_clipped.osm -o ../../generated/south_lake_union_clipped.glb --lod 3
Set-Location ../../..
& urban_scene/tools/gltfpack_v1_3/gltfpack.exe -i urban_scene/generated/south_lake_union_clipped.glb -o urban_scene/generated/south_lake_union_runtime.glb -noq -r urban_scene/generated/gltfpack_report.json
& 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe' urban_scene/analyze_road_network.py
```

The new UE level has already been created. To run its independent scene:

```powershell
& urban_scene/start_urban_simulator.ps1 -Offscreen
# Wait until ProjectAirSim ports 8989 and 8990 are ready.
& 'C:/Users/Stephen/.conda/envs/projectairsim310/python.exe' urban_scene/probe_urban_scene.py
```

The probe loads the optimized GLB into the new level using ProjectAirSim's
runtime file-spawn API. It is a scene compatibility check, not an experiment.
The camera frame, when successful, is saved in `generated/urban_probe.png`
with a JSON receipt. An earlier attempt using the unoptimized 2,101-mesh GLB
exhausted UE memory in the runtime importer; use the optimized version.

## Study gates

Before claiming urban navigation or pursuit results, verify the original
camera view, road-to-vehicle scale, drivable collision, sensor visibility,
and stable multiple-robot motion in this map. Navigation then needs a
separate UGV-only baseline and UAV-assisted dynamic closure runs. Pursuit
needs independent seeded target turns, two aerial viewpoints and two ground
interceptors, plus explicit escape/capture and collision criteria. The
existing `MovingDeckLab/configs/scale_matrix` contains candidate fleet-size
configs only; a matched-load multi-run scale study is not yet complete.
