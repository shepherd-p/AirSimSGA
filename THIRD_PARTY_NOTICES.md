# Source attribution

AirSimSGA release packaging and experiment additions are provided under MIT.
Native overlays contain modified ProjectAirSim code: original Microsoft and
IAMAI copyright headers remain, and the upstream LICENSE and NOTICE are in
`licenses/`. The upstream base commit is in `native/upstream.lock.json`.
Do not remove original namespace or notices when using the extension profiles.

The full upstream tree and third-party dependency sources are not redistributed
inside this package. Obtain them separately and keep their own notices.
The scientific references to AirSim, AirSimAG, ASVSim and Fossen do not imply
that every capability of those works is implemented here. Classic ASVSim-derived
engine code elsewhere on the author's machine is not copied into this release.

Unreal Engine, Epic City Sample, CityPark and vessel/vehicle/marker meshes are
external assets. Their names and Unreal references in configs are identifiers,
not asset redistribution grants. Obtain assets under their own terms or supply
compatible replacements. Generated route configs contain lane IDs/waypoints
from the experiment's external map; they are task configuration rather than the underlying scene assets.
Retain the scene provider's terms when adapting or distributing derived configurations. No engine, map, commercial mesh, plugin DLL or video is included here.
`recorded/` contains text source/configuration only; downloaded tool and asset payloads are excluded.

Recorded gtest/experiment summaries in `evidence/` document historical runs;
large raw sensor archives remain external. Figure processing software includes
point filtering and surface rendering; those products must not be described as
raw LiDAR, physical sensor measurements or newly measured frames.
