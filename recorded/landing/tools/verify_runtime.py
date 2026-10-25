from pathlib import Path
import json,hashlib,psutil,sys
root=Path(__file__).resolve().parents[1]
lab=Path(r'F:/EpicGamesEnv/LandingWaterLab')
launch=json.loads((root/'launch_receipt.json').read_text(encoding='utf-8-sig'))
process=psutil.Process(launch['pid'])
expected=lab/'CityPark2/Plugins/ProjectAirSim/Binaries/Win64/UnrealEditor-ProjectAirSim-Win64-DebugGame.dll'
modules=[Path(m.path).resolve() for m in process.memory_maps(grouped=True) if Path(m.path).name.startswith('UnrealEditor-ProjectAirSim') and m.path.lower().endswith('.dll')]
if modules!=[expected.resolve()]:raise RuntimeError('Independent plugin is not loaded: '+str(modules))
build=json.loads((lab/'build_receipt.json').read_text(encoding='utf-8'))
digest=hashlib.sha256(expected.read_bytes()).hexdigest()
if digest!=build['binaries'][str(expected)]:raise RuntimeError('Plugin hash mismatch')
receipt={'pid':process.pid,'cmdline':process.cmdline(),'loaded_plugin':str(expected),'plugin_sha256':digest,'create_time':process.create_time(),'verified':True}
old=lab/'runtime_receipt.json'
backup=root/'previous_runtime_receipt.json'
if old.exists() and not backup.exists():backup.write_bytes(old.read_bytes())
content=json.dumps(receipt,indent=2)+'\n'
old.write_text(content,encoding='utf-8')
(root/'runtime_receipt.json').write_text(content,encoding='utf-8')
print(json.dumps(receipt,indent=2))
