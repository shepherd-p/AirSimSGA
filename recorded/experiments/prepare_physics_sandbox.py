"""Create a real, independent source/project copy; never links writable originals."""
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

ROOT = Path('F:/EpicGamesEnv')
DEST = ROOT / 'MovingDeckLab'

def digest(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return h.hexdigest()

def main():
    DEST.mkdir(exist_ok=False)
    src = ROOT/'ProjectAirSim'
    record = {'source':str(src),'destination':str(DEST),'files':{}}
    def copy(a,b):
        b.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(a,b)
        record['files'][str(a)]={'copy':str(b),'sha256':digest(a)}
    def tree(a,b,exclude=()):
        for p in a.rglob('*'):
            rel=p.relative_to(a)
            if any(part in exclude for part in rel.parts):continue
            if p.is_file():copy(p,b/rel)
    for name in ['core_sim','physics','simserver','mavlinkcom','rendering','vehicle_apis','tools','thirdparty','client']:
        tree(src/name, DEST/'source'/name, ['_deps','__pycache__','matlab_sfunc','.git'])
    for name in ['CMakeLists.txt','LICENSE','NOTICE.txt']:
        if (src/name).exists():copy(src/name,DEST/'source'/name)
    tree(src/'unreal/Blocks/Plugins/ProjectAirSim/Source',DEST/'source/unreal/Blocks/Plugins/ProjectAirSim/Source')
    record['git_head']=subprocess.check_output(['git','-c','safe.directory=F:/EpicGamesEnv/ProjectAirSim','-C',str(src),'rev-parse','HEAD'],text=True).strip()
    (DEST/'original_worktree.diff').write_bytes(subprocess.check_output(['git','-c','safe.directory=F:/EpicGamesEnv/ProjectAirSim','-C',str(src),'diff','--binary']))
    project=ROOT/'CityPark2'
    for name in ['Source','Config','Content']:
        print('Copy',name,flush=True)
        tree(project/name, DEST/'CityPark2'/name)
    copy(project/'CityPark2.uproject',DEST/'CityPark2/CityPark2.uproject')
    for name in ['ProjectAirSim','Drone','Rover']:
        print('Copy plugin',name,flush=True)
        tree(project/'Plugins'/name,DEST/'CityPark2/Plugins'/name,['Intermediate','Binaries','.git'])
    deps=src/'build/win64/Debug/_deps'
    for name in ['msgpack-src','tinygltf-src','googletest-src']:
        tree(deps/name,DEST/'deps'/name,['.git'])
    (DEST/'copy_manifest.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    print('Copied',len(record['files']),'files into',DEST,flush=True)

if __name__=='__main__':main()
