import unreal,json
from pathlib import Path
root=Path(r'F:/EpicGamesEnv/LandingWaterLab');folder='/Game/SGAWaterLab'
mesh=unreal.load_asset(folder+'/lake_grid')
report={'before_bounds':str(mesh.get_bounds())}
# OBJ importer interprets the source as Y-up. Use x,0,z source coordinates.
source=root/'assets/lake_grid.obj';fixed=root/'assets/lake_grid_yup.obj'
lines=[]
for line in source.read_text().splitlines():
 if line.startswith('v '):
  _,x,y,z=line.split();line=f'v {x} 0 {y}'
 lines.append(line)
fixed.write_text('\n'.join(lines)+'\n')
task=unreal.AssetImportTask();task.filename=str(fixed);task.destination_path=folder;task.destination_name='lake_grid';task.automated=True;task.save=True;task.replace_existing=True
options=unreal.FbxImportUI();options.import_mesh=True;options.import_materials=False;options.import_textures=False;options.import_as_skeletal=False
options.static_mesh_import_data.set_editor_property('generate_lightmap_u_vs',False)
task.options=options;unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
mesh=unreal.load_asset(folder+'/lake_grid');report['after_bounds']=str(mesh.get_bounds())
assert mesh.get_bounds().box_extent.z<1,report
assert mesh.get_bounds().box_extent.x>19000 and mesh.get_bounds().box_extent.y>19000,report
(root/'results/grid_axis_repair.json').write_text(json.dumps(report,indent=2))
unreal.log('SGA_GRID_REPAIR_DONE')
