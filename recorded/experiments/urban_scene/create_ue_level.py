"""Create a separate lit UE level for the runtime-loaded urban mesh."""

import unreal


LEVEL = "/Game/AGUrban/UrbanBase"


def spawn(cls, label, location=(0, 0, 0), rotation=(0, 0, 0)):
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        cls, unreal.Vector(*location), unreal.Rotator(*rotation)
    )
    if actor is None:
        raise RuntimeError("Unable to spawn " + label)
    actor.set_actor_label(label)
    return actor


if unreal.EditorAssetLibrary.does_asset_exist(LEVEL):
    raise RuntimeError("UrbanBase already exists; refusing to overwrite it")

if not unreal.EditorLevelLibrary.new_level(LEVEL):
    raise RuntimeError("Unable to create " + LEVEL)

sun = spawn(unreal.DirectionalLight, "Urban_Sun", rotation=(-40, -35, 0))
sun.get_editor_property("light_component").set_editor_property("intensity", 8.0)
sky = spawn(unreal.SkyLight, "Urban_Skylight")
sky.get_editor_property("light_component").set_editor_property("intensity", 1.0)
spawn(unreal.SkyAtmosphere, "Urban_Atmosphere")
spawn(unreal.ExponentialHeightFog, "Urban_Fog")
spawn(unreal.PlayerStart, "Urban_PlayerStart", location=(0, 0, 500))

if not unreal.EditorLevelLibrary.save_current_level():
    raise RuntimeError("Unable to save " + LEVEL)

print("AG_URBAN_LEVEL_CREATED " + LEVEL)
