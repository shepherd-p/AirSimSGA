# Scenes and external assets

No UE, City Sample, CityPark, robot/boat meshes, textures or DLLs are included.
Asset identifiers are listed in `asset_requirements.json`. Obtain compatible
assets separately under their terms; configurations do not grant asset rights.
Use copied/separate projects rather than overwriting working scene builds.

## CityPark and landing

Mapping routes target the original plaza. Landing uses a separate project with
water at NED z=8.8 m, tag36h11 ID 1, tag side 0.4 m, hull-frame deck offset
`[-3, 0, -2.77]` m. Replace Unreal references with your boat/marker assets and
recalibrate if geometry changes. The stored calibration has 71 historical
measurements and does not certify replacement scenes.

## Epic City Sample

Obtain City Sample through your Epic installation and preserve its engine/plugin
compatibility. Use `Big_City_LVL` and native vehicle ZoneGraph. Place
`native/citysample/AGZoneGraphExport.cpp` in the copied CitySample module's
`Source/CitySample/Util`. Its includes require ZoneGraph, Json and JsonUtilities
module dependencies. Compile the copied project, start the map and invoke
`AG.ZoneGraph.Export` in the UE console. It writes
`Saved/AGCitySampleZoneGraph.json`. Configure `citysample_project` or
`AIRSIMSGA_ZONEGRAPH`. Disable ambient traffic for the controlled experiments.

Scenario configs retain the original lane IDs. Runners require export SHA256
agreement. If the export differs, run `python -m airsimsga prepare navigation`
and `python -m airsimsga prepare containment`; edit the explicit lane constants
for a changed map, then check directed connectivity/heights/spawn clearance.
This is not automatic arbitrary-map adaptation. Regeneration intentionally
changes local configs, so the original release hash check will report changes.
