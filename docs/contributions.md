# Implementation map

| Component | Source location |
| --- | --- |
| Public CLI and USV service adapter | `airsimsga/` |
| Cooperative LiDAR/RGB-D capture and reconstruction | `experiments/ag/`, mapping entry scripts |
| Aerial-assisted multi-junction navigation | `experiments/citysample_scene/` |
| Multi-UAV/UGV tracking and containment | `experiments/citysample_scene/`, `experiments/ag/planning.py` |
| Native scene export | `native/citysample/` |
| USV configuration and per-robot thrust services | `native/profiles/main/` |
| Moving support contact, camera synchronization, disarm interlock | `native/profiles/moving-deck/` |
| Final planar-vessel dynamics and landing changes | `native/profiles/recorded-landing/` |
| Final recording and scene-construction scripts | `recorded/` |
| Manuscript numerical records and analysis | `data/paper/` |

The backend is assembled from the pinned upstream source and one native overlay.
Original notices and source provenance are retained. See `native/upstream.lock.json`
and `THIRD_PARTY_NOTICES.md`. Each profile is applied independently to the same base.
