# Configuration notes

The main and landing native profiles are separate builds. The final
`recorded-landing` profile integrates planar surge/sway/yaw vessel motion;
water ripples and foam are presentation effects. The generic main profile uses
a simpler vessel-motion bridge. Propulsion azimuth is not implemented by the
public `SurfaceVehicle` adapter.

Mapping aligns simulated sensor observations with recorded simulator poses.
Navigation uses the native scene road graph and aerial observations; the tracking
example uses segmentation/depth observations; landing uses an AprilTag marker.
Users can replace these example algorithms through the observation/control APIs.

Scene compatibility, camera configuration and controller parameters should be
checked when changing assets or environment geometry. Third-party assets and
simulator binaries are obtained separately. See `recorded_release.md` for the
final recording configuration and `../QA/` for checks performed on this release.
