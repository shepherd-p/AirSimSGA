import unreal,json
from pathlib import Path
root=Path(r'F:/EpicGamesEnv/LandingWaterLab');folder='/Game/SGAWaterLab'
if not unreal.EditorAssetLibrary.does_asset_exist(folder+'/WaterDiagnostic'):
 copied=unreal.EditorAssetLibrary.duplicate_asset('/Game/CityPark/Maps/Showcase',folder+'/WaterDiagnostic')
 assert copied
 assert unreal.EditorAssetLibrary.save_loaded_asset(copied)
assert unreal.EditorLevelLibrary.load_level(folder+'/WaterDiagnostic')
assert unreal.EditorLevelLibrary.get_editor_world().get_path_name().startswith(folder+'/')
lib=unreal.MaterialEditingLibrary;assets=unreal.AssetToolsHelpers.get_asset_tools()
def material(name,foam=False):
 path=folder+'/'+name
 if unreal.EditorAssetLibrary.does_asset_exist(path):return unreal.load_asset(path)
 m=assets.create_asset(name,folder,unreal.Material,unreal.MaterialFactoryNew())
 m.set_editor_property('two_sided',True)
 if foam:
  m.set_editor_property('blend_mode',unreal.BlendMode.BLEND_TRANSLUCENT)
  m.set_editor_property('shading_model',unreal.MaterialShadingModel.MSM_UNLIT)
 else:m.set_editor_property('tangent_space_normal',False)
 return m
def node(m,cls,**attrs):
 x=lib.create_material_expression(m,cls)
 for k,v in attrs.items():x.set_editor_property(k,v)
 return x
def scalar(m,name,value):return node(m,unreal.MaterialExpressionScalarParameter,parameter_name=name,default_value=value)
def custom(m,code,inputs,out):
 x=node(m,unreal.MaterialExpressionCustom,code=code,output_type=out)
 descriptors=[]
 for name in inputs:
  descriptor=unreal.CustomInput();descriptor.set_editor_property('input_name',name);descriptors.append(descriptor)
 x.set_editor_property('inputs',descriptors)
 for name,src in inputs.items():assert lib.connect_material_expressions(src,'',x,name)
 return x
lake=material('M_Lake')
lib.delete_all_material_expressions(lake)
wp=node(lake,unreal.MaterialExpressionWorldPosition);t=scalar(lake,'SimTime',0)
common='float a=WP.x*.035+WP.y*.013-ST*1.9;float b=WP.x*.012-WP.y*.023-ST*1.2;float c=WP.x*.061+WP.y*.043-ST*2.7;'
dis=custom(lake,common+'return float3(0,0,2.0*sin(a)+1.2*sin(b)+.6*sin(c));',{'WP':wp,'ST':t},unreal.CustomMaterialOutputType.CMOT_FLOAT3)
norm=custom(lake,common+'float dx=2*.035*cos(a)+1.2*.012*cos(b)+.6*.061*cos(c);float dy=2*.013*cos(a)-1.2*.023*cos(b)+.6*.043*cos(c);return normalize(float3(-dx,-dy,1));',{'WP':wp,'ST':t},unreal.CustomMaterialOutputType.CMOT_FLOAT3)
color=node(lake,unreal.MaterialExpressionConstant3Vector,constant=unreal.LinearColor(.018,.105,.12,1))
rough=scalar(lake,'Roughness',.14);spec=scalar(lake,'Specular',.45)
assert unreal.SGAWaterAuthoring.connect_wave_offset(lake,dis)
for nd,prop in [(norm,unreal.MaterialProperty.MP_NORMAL),(color,unreal.MaterialProperty.MP_BASE_COLOR),(rough,unreal.MaterialProperty.MP_ROUGHNESS),(spec,unreal.MaterialProperty.MP_SPECULAR)]:assert lib.connect_material_property(nd,'',prop)
lib.recompile_material(lake);assert unreal.EditorAssetLibrary.save_loaded_asset(lake)
foam=material('M_Foam',True);lib.delete_all_material_expressions(foam)
uv=node(foam,unreal.MaterialExpressionTextureCoordinate);alpha=scalar(foam,'Alpha',.4)
mask=custom(foam,'float x=UV.x*2-1,y=UV.y;float foamArm=exp(-pow(abs(x)-(.06+.82*y),2)/.003);float wash=.32*exp(-x*x/.025)*(1-y);float noise=.4+.6*frac(sin(dot(floor(UV*float2(90,180)),float2(12.9898,78.233)))*43758.5453);return saturate((foamArm+wash)*noise*smoothstep(0,.04,y)*(1-smoothstep(.45,1,y))*Alpha);',{'UV':uv,'Alpha':alpha},unreal.CustomMaterialOutputType.CMOT_FLOAT1)
white=node(foam,unreal.MaterialExpressionConstant3Vector,constant=unreal.LinearColor(.58,.64,.62,1))
assert lib.connect_material_property(mask,'',unreal.MaterialProperty.MP_OPACITY)
assert lib.connect_material_property(white,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR)
lib.recompile_material(foam);assert unreal.EditorAssetLibrary.save_loaded_asset(foam)
if not unreal.EditorAssetLibrary.does_asset_exist(folder+'/lake_grid'):
 task=unreal.AssetImportTask();task.filename=str(root/'assets/lake_grid.obj');task.destination_path=folder;task.destination_name='lake_grid';task.automated=True;task.save=True;task.replace_existing=False
 options=unreal.FbxImportUI();options.import_mesh=True;options.import_materials=False;options.import_textures=False;options.import_as_skeletal=False
 options.static_mesh_import_data.set_editor_property('generate_lightmap_u_vs',False)
 task.options=options;assets.import_asset_tasks([task]);assert task.imported_object_paths
# Authoring only in own namespace. This level starts in baseline presentation for isolation captures.
assert unreal.EditorLevelLibrary.save_current_level()
(root/'results/water_asset_receipt.json').write_text(json.dumps({'created':['M_Lake','M_Foam','lake_grid'],'visual_wave_amplitudes_cm':[2,1.2,.6],'wave_clock':'simulation time, not wall time','physical_wave_forces':False},indent=2),encoding='utf-8')
unreal.log('SGA_WATER_ASSETS_DONE')
