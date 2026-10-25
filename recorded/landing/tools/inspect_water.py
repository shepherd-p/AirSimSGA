import unreal,json
from pathlib import Path
root=Path(r"F:/EpicGamesEnv/LandingWaterLab")
assert unreal.EditorLevelLibrary.load_level('/Game/CityPark/Maps/Showcase')
results=[]
for a in unreal.EditorLevelLibrary.get_all_level_actors():
 cls=a.get_class().get_name();label=a.get_actor_label()
 comps=a.get_components_by_class(unreal.StaticMeshComponent)
 interesting=('Reflection' in cls or 'Light' in cls or 'Water' in label or 'Boat' in label or 'Vessel' in label)
 for c in comps:
  mesh=c.get_editor_property('static_mesh')
  if mesh and any(x in mesh.get_path_name().lower() for x in ['waterplane','boat','vessel']):interesting=True
 if not interesting:continue
 d={'name':a.get_name(),'label':label,'class':cls,'location':str(a.get_actor_location()),'rotation':str(a.get_actor_rotation()),'scale':str(a.get_actor_scale3d()),'components':[]}
 for c in comps:
  mesh=c.get_editor_property('static_mesh')
  ci={'name':c.get_name(),'mesh':mesh.get_path_name() if mesh else None,'materials':[]}
  for i in range(c.get_num_materials()):
   m=c.get_material(i)
   if not m:continue
   mi={'path':m.get_path_name()}
   if isinstance(m,unreal.MaterialInstanceConstant):
    try:mi['scalars']={str(n):unreal.MaterialEditingLibrary.get_material_instance_scalar_parameter_value(m,n) for n in unreal.MaterialEditingLibrary.get_scalar_parameter_names(m)}
    except Exception as e:mi['error']=str(e)
   ci['materials'].append(mi)
  d['components'].append(ci)
 results.append(d)
(root/'results/water_scene_inventory.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
# Save a level copy before any rendering isolation edits. Never save the source level.
# Inventory only; map duplication is performed by the dedicated scoped authoring script.
unreal.log('SGA_WATER_INSPECTION_DONE')
