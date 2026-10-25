"""Offline analysis of measured runs; preserves failed/incomplete run status."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ag.storage import write_json


def rows(path):
    return list(csv.DictReader(path.open(encoding='utf-8'))) if path.exists() else []


def analyze(folder):
    folder = Path(folder)
    manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    summary = {'run_status':manifest['status'],'task':manifest['task'],'source':'measured run files'}
    trajectory = rows(folder/'trajectories.csv')
    fig, axes = plt.subplots(1,2,figsize=(12,5))
    for name in sorted({r['name'] for r in trajectory}):
        rr = [r for r in trajectory if r['name']==name]
        p = np.array([[float(r[k]) for k in ('x','y','z')] for r in rr])
        axes[0].plot(p[:,0],p[:,1],label=name)
        summary[name+'_path_m'] = float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
    if trajectory:
        axes[0].legend()
    if (folder/'planned_path.json').exists():
        path = np.array(json.loads((folder/'planned_path.json').read_text(encoding='utf-8')))
        axes[0].plot(path[:,0],path[:,1],'k--',label='UAV-assisted plan')
        axes[0].legend()
        rr = [r for r in trajectory if r['name']=='Car1']
        actual = np.array([[float(r['x']),float(r['y'])] for r in rr])
        if len(actual):
            summary['goal_distance_m'] = float(np.linalg.norm(actual[-1]-path[-1,:2]))
            summary['planned_length_m'] = float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
    elif (folder/'reference_routes.json').exists() and manifest['task']=='formation':
        routes = json.loads((folder/'reference_routes.json').read_text(encoding='utf-8'))
        summary['cross_track_rmse_m'] = {}
        for name,route in routes.items():
            path = np.array(route)
            axes[0].plot(path[:,0],path[:,1],'--',alpha=.4)
            rr = [r for r in trajectory if r['name']==name]
            actual = np.array([[float(r['x']),float(r['y'])] for r in rr])
            if len(actual) and len(path)>1:
                starts,segments=path[:-1,:2],np.diff(path[:,:2],axis=0)
                delta=actual[:,None,:]-starts[None,:,:]
                t=np.clip(np.sum(delta*segments[None,:,:],axis=2)/np.maximum(np.sum(segments**2,axis=1),1e-9),0,1)
                distance=np.linalg.norm(delta-t[:,:,None]*segments[None,:,:],axis=2).min(axis=1)
                summary['cross_track_rmse_m'][name]=float(np.sqrt(np.mean(distance**2)))
        axes[0].legend(loc='upper center',bbox_to_anchor=(.5,-.14),ncol=4,fontsize=8)
    axes[0].set(xlabel='North (m)',ylabel='East (m)',title='Executed trajectories',aspect='equal')
    tracking = rows(folder/'tracking.csv')
    if tracking:
        tt = np.array([float(r['time']) for r in tracking]); tt -= tt[0]
        for key in ('uav_error_m','ugv_error_m','xy_error_m'):
            values = np.array([float(r[key]) if r[key] else np.nan for r in tracking])
            axes[1].plot(tt,values,label=key)
            valid = values[np.isfinite(values)]
            summary[key] = {'valid':len(valid),'mean':float(valid.mean()) if len(valid) else None,
                            'rmse':float(np.sqrt(np.mean(valid**2))) if len(valid) else None,
                            'max':float(valid.max()) if len(valid) else None}
        axes[1].legend()
        axes[1].set(xlabel='Simulation time (s)',ylabel='XY error (m)',title='Tracking estimates')
        uav = np.array([r['uav_visible']=='True' for r in tracking])
        ugv = np.array([r['ugv_visible']=='True' for r in tracking])
        summary['uav_visibility_fraction']=float(uav.mean())
        summary['ugv_visibility_fraction']=float(ugv.mean())
        summary['ground_only_frames']=int((~uav & ugv).sum())
        summary['under_canopy_uav_image_missing_ground_visible'] = sum(
            r.get('target_origin_under_canopy')=='True' and r.get('uav_reason')=='target_not_in_image' and r['ugv_visible']=='True'
            for r in tracking)
        summary['both_missing_frames']=int((~uav & ~ugv).sum())
        summary['observed_reacquisitions']=int((~uav[:-1]&uav[1:]).sum())
        fig_vis,ax_vis=plt.subplots(figsize=(10,3))
        ax_vis.step(tt,uav.astype(float),where='post',label='UAV visible')
        ax_vis.step(tt,ugv.astype(float)+1.2,where='post',label='UGV visible (+1.2)')
        ax_vis.set(xlabel='Simulation time (s)',ylabel='Visibility',title='Actual segmentation visibility')
        ax_vis.legend()
        fig_vis.tight_layout()
        fig_vis.savefig(folder/'visibility.png',dpi=170)
        plt.close(fig_vis)
    else:
        perf = rows(folder/'performance.csv')
        fps = [(float(r['wall_elapsed_sec']),float(r['fps'])) for r in perf if r['fps']]
        if fps:
            a = np.array(fps)
            axes[1].plot(a[:,0],a[:,1])
            summary['fps_mean'] = float(a[:,1].mean())
        axes[1].set(xlabel='Wall time (s)',ylabel='FPS',title='Measured UE performance' if fps else 'UE performance unavailable')
    fig.suptitle(f"{manifest['task']} — {manifest['status']}")
    fig.tight_layout()
    fig.savefig(folder/'results.png',dpi=170)
    plt.close(fig)
    counts = rows(folder/'sensor_counts.csv')
    summary['sensor_receive_hz'] = {}
    for stream in sorted({r['stream'] for r in counts}):
        rr = [r for r in counts if r['stream']==stream]
        duration = float(rr[-1]['wall_elapsed_sec'])-float(rr[0]['wall_elapsed_sec'])
        summary['sensor_receive_hz'][stream] = (int(rr[-1]['messages_total'])-int(rr[0]['messages_total']))/duration if duration>0 else None
    maps = list(folder.glob('map_*.pcd'))
    if maps:
        fig,ax = plt.subplots(figsize=(8,7))
        for path in maps:
            with path.open() as f:
                for line in f:
                    if line.startswith('DATA'):
                        break
                points = np.loadtxt(f).reshape(-1,3)
            if len(points):
                points=points[::max(1,len(points)//30000)]
                ax.scatter(points[:,0],points[:,1],s=.4,label=path.stem)
        ax.set(xlabel='North (m)',ylabel='East (m)',aspect='equal',title='Measured point cloud map')
        ax.legend()
        fig.savefig(folder/'maps.png',dpi=170)
        plt.close(fig)
    write_json(folder/'analysis.json',summary)
    return summary


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--input-dir',required=True)
    args = p.parse_args()
    print(json.dumps(analyze(args.input_dir),ensure_ascii=False,indent=2))
