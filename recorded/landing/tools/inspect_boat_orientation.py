import unreal,json
from pathlib import Path
root=Path(r'F:/EpicGamesEnv/LandingWaterLab');mesh=unreal.load_asset('/Game/Models/Vessel/Boat')
report={'asset':mesh.get_path_name(),'bounds':str(mesh.get_bounds())}
task=unreal.AssetExportTask();task.object=mesh;task.filename=str(root/'assets/boat_mesh_orientation.obj');task.automated=True;task.prompt=False;task.replace_identical=True;task.exporter=unreal.StaticMeshExporterOBJ()
report['export_success']=unreal.Exporter.run_asset_export_task(task);report['errors']=list(task.errors)
(root/'results/boat_asset_orientation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
assert report['export_success'],report
unreal.log('SGA_BOAT_ORIENTATION_EXPORTED')
