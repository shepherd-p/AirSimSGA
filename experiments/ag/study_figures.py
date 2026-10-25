from .fonts import font as portable_font
"""High-resolution figures derived solely from a recorded cooperative study."""
import csv
import json
import math
import shutil
import hashlib
import platform
import gzip
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageDraw,ImageFont
from .math3d import camera_pose
from .storage import write_json

COLORS={'Drone1':'#245bea','Drone2':'#5965ce','Drone3':'#9c43bf','Drone4':'#db5a8d',
        'Car1':'#16a05d','Car2':'#db9209','Car3':'#9b641c','Car4':'#008e9b','Target':'#353535'}


def read_rows(path):
    if not path.exists():return []
    with path.open(encoding='utf-8',newline='') as stream:return list(csv.DictReader(stream))


def statistics(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    return {'samples':len(a),'mean':float(a.mean()),'std':float(a.std()),'variance':float(a.var()),
            'rmse':float(np.sqrt(np.mean(a*a))),'maximum':float(a.max()),'p95':float(np.percentile(a,95))} if len(a) else {'samples':0}


def cross_track(points,path):
    delta=points[:,:2,None]-path[:-1,:2].T[None,:,:]
    delta=np.transpose(delta,(0,2,1));segment=np.diff(path[:,:2],axis=0)
    t=np.clip(np.sum(delta*segment[None,:,:],axis=2)/np.maximum(np.sum(segment**2,axis=1),1e-12),0,1)
    return np.linalg.norm(delta-t[:,:,None]*segment[None,:,:],axis=2).min(axis=1)


class StudyFigures:
    def __init__(self,folder):
        self.folder=Path(folder);self.out=self.folder/'study_results';self.out.mkdir(exist_ok=True)
        self.manifest=json.loads((self.folder/'manifest.json').read_text(encoding='utf-8'))
        self.config=json.loads((self.folder/'config.json').read_text(encoding='utf-8'))
        self.task=self.manifest['task'];self.ground=self.config['ground_z_ned'];self.sources={};self.metrics={'task':self.task,'run_status':self.manifest['status']}
        control_rows=read_rows(self.folder/'trajectories.csv')
        windows={n:(min(float(r['time']) for r in control_rows if r['name']==n),
                    max(float(r['time']) for r in control_rows if r['name']==n)) for n in {r['name'] for r in control_rows}}
        # GetObjectPose (used by the controller) reflects the rendered actor and
        # is queried separately from the clock. Use capture-stamped physics pose
        # topics for publication trajectories and time-aligned evaluation.
        from .math3d import pose_parts
        captured={}
        with gzip.open(self.folder/'pose_collision_topics.jsonl.gz','rt',encoding='utf-8') as stream:
            for line in stream:
                record=json.loads(line)
                if not record['stream'].endswith('/pose'):continue
                n=record['stream'].split('/')[0];m=record['message'];t=int(m['time_stamp'])/1e9
                if n not in windows or not windows[n][0]<=t<=windows[n][1]:continue
                p,r=pose_parts(m)
                captured[(n,int(m['time_stamp']))]={'time':t,'time_ns':int(m['time_stamp']),'name':n,
                    'x':p[0],'y':p[1],'z':p[2],'yaw_rad':math.atan2(r[1,0],r[0,0])}
        rr=sorted(captured.values(),key=lambda r:(r['time'],r['name']))
        if not rr:raise ValueError('No original timestamped physics pose messages')
        with (self.out/'trajectories_topic.csv').open('w',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rr[0]));writer.writeheader();writer.writerows(rr)
        self.metrics['trajectory_source']='study_results/trajectories_topic.csv: original physics pose topics within the per-platform controller recording window'
        self.metrics['controller_pose_timing_note']='Original trajectories.csv uses GetObjectPose followed by a separate simulation-clock query; retained for controller replay, not used as exact sensor-time truth.'
        self.tracks={};self.track_times={};self.yaws={}
        for name in sorted({r['name'] for r in rr}):
            rows=[r for r in rr if r['name']==name]
            self.tracks[name]=np.array([[float(r[k]) for k in 'xyz'] for r in rows])
            self.track_times[name]=np.array([float(r['time']) for r in rows]);self.yaws[name]=np.unwrap([float(r['yaw_rad']) for r in rows])
        self.t0=min(t[0] for t in self.track_times.values())
        self.frames=read_rows(self.folder/'camera_frames.csv')
        for r in self.frames:r['time']=int(r['time_ns'])/1e9;r['boxes']=json.loads(r['visible_robots'])
        plt.rcParams.update({'font.size':12,'figure.facecolor':'white','savefig.facecolor':'white','font.family':'DejaVu Sans'})

    def position(self,name,t):
        return np.column_stack([np.interp(t,self.track_times[name],self.tracks[name][:,i]) for i in range(3)])

    def save(self,fig,name,inputs=None):
        fig.savefig(self.out/f'{name}.png',dpi=300,bbox_inches='tight')
        fig.savefig(self.out/f'{name}.pdf',bbox_inches='tight');plt.close(fig)
        self.sources[name]=inputs or {'data':['study_results/trajectories_topic.csv'],'geometry':'Recorded positions; no invented trajectories'}

    def image(self,row,boxes=False,target_only=False):
        image=Image.open(self.folder/row['image']).convert('RGB')
        if boxes:
            draw=ImageDraw.Draw(image);font=portable_font(max(19,image.width//60), bold=True)
            for name,item in row['boxes'].items():
                if target_only and name!='Target':continue
                if item['pixels']<10:continue
                x0,y0,x1,y1=item['bbox'];color='#39e832' if target_only else COLORS.get(name,'#ffcc00')
                draw.rectangle((x0-4,y0-4,x1+4,y1+4),outline=color,width=3)
                text=name.replace('Drone','UAV ').replace('Car','UGV ')
                y=max(0,y0-35);box=draw.textbbox((x0,y),text,font=font)
                draw.rectangle(box,fill='white');draw.text((x0,y),text,font=font,fill=color)
        return image

    def source(self,row):return {k:row[k] for k in ('camera','time_ns','file','image','pair_delta_ms')}

    def panel(self,items,name,columns=3,width=960):
        # Every sub-image retains aspect ratio; timestamps/provenance are explicit.
        cell_height=650;canvas=Image.new('RGB',(columns*width,math.ceil(len(items)/columns)*cell_height),'white')
        draw=ImageDraw.Draw(canvas);font=portable_font(28)
        sources=[]
        for i,(im,title,source) in enumerate(items):
            if isinstance(im,Path):im=Image.open(im).convert('RGB')
            im=im.copy();im.thumbnail((width-24,cell_height-76),Image.Resampling.LANCZOS)
            x=(i%columns)*width+(width-im.width)//2;y=(i//columns)*cell_height
            canvas.paste(im,(x,y+55));draw.text(((i%columns)*width+18,y+10),title,font=font,fill='#172b3a')
            sources.append(source)
        canvas.save(self.out/f'{name}.png',dpi=(300,300));self.sources[name]={'panels':sources}

    def trajectory(self,name='trajectories_3d',names=None):
        names=names or list(self.tracks)
        ground_only=all(n.startswith('Car') for n in names)
        fig=plt.figure(figsize=(10,5.5) if ground_only else (10,8));ax=fig.add_subplot(111,projection='3d')
        for n in names:
            p=self.tracks[n].copy();p[:,2]=self.ground-p[:,2]
            ax.plot(*p.T,c=COLORS[n],lw=1.8,label=n.replace('Drone','UAV ').replace('Car','UGV '))
            ax.scatter(*p[0],c='#05ab46',marker='o',s=42,depthshade=False)
            ax.scatter(*p[-1],c='#e22e36',marker='s',s=42,depthshade=False)
        allp=np.vstack([self.tracks[n] for n in names]).copy();allp[:,2]=self.ground-allp[:,2]
        low=allp.min(axis=0);high=allp.max(axis=0);mid=(low+high)/2
        span=np.maximum((high-low)*1.12,[5,5,3]);low=mid-span/2;high=mid+span/2
        if all(n.startswith('Car') for n in names):
            low[2]=min(-.25,float(allp[:,2].min())-.1);high[2]=max(3.,float(allp[:,2].max())+.1)
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),zlim=(low[2],high[2]))
        ax.set_box_aspect(high-low);ax.view_init(elev=28,azim=-60)
        from matplotlib.ticker import MaxNLocator
        for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.set_major_locator(MaxNLocator(nbins=5))
        if ground_only:
            ax.xaxis.set_major_locator(MaxNLocator(nbins=4));ax.yaxis.set_major_locator(MaxNLocator(nbins=3));ax.zaxis.set_major_locator(MaxNLocator(nbins=2))
        ax.tick_params(labelsize=10,pad=1)
        ax.set(xlabel='North (m)',ylabel='East (m)',title=self.task.title()+' | Recorded trajectories')
        ax.xaxis.labelpad=14 if ground_only else 8;ax.yaxis.labelpad=10
        fig.text(.96,.5,'Height above plaza (m)',rotation=90,ha='center',va='center',fontsize=11)
        ax.legend(loc='upper left',fontsize=10)
        fig.subplots_adjust(left=0,right=.91,bottom=.09,top=.9)
        fig.text(.5,.01,'Green circles: starts | Red squares: ends | Simulator poses; no vertical exaggeration',ha='center',fontsize=10)
        self.save(fig,name)

    def project(self,row,points):
        with np.load(self.folder/row['file']) as d:meta=json.loads(str(d['metadata_json']))['scene_camera']
        p,r=camera_pose(meta);local=(np.asarray(points)-p)@r
        focal=int(row['width'])/2 # Main robot cameras use 90 degree horizontal FOV.
        if row['camera'].startswith('Observer'):focal=int(row['width'])/(2*np.tan(np.deg2rad(80)/2))
        uv=np.c_[int(row['width'])/2+focal*local[:,1]/np.maximum(local[:,0],1e-6),
                 int(row['height'])/2+focal*local[:,2]/np.maximum(local[:,0],1e-6)]
        valid=(local[:,0]>.1)&np.isfinite(uv).all(axis=1)
        return uv,valid

    def overlay(self,row,paths):
        im=self.image(row);draw=ImageDraw.Draw(im)
        for points,color in paths:
            uv,valid=self.project(row,points)
            for i in range(len(uv)-1):
                if valid[i] and valid[i+1]:draw.line([tuple(uv[i]),tuple(uv[i+1])],fill=color,width=max(3,im.width//300))
            for i,fill,shape in [(0,'#00db50','circle'),(-1,'#f03434','square')]:
                if valid[i]:
                    x,y=uv[i];box=[x-6,y-6,x+6,y+6]
                    (draw.ellipse if shape=='circle' else draw.rectangle)(box,fill=fill,outline='white',width=2)
        return im

    def scene(self):
        names=set(self.tracks);candidates=[]
        for row in self.frames:
            if not row['camera'].startswith('Observer') or row['time']<self.t0+15:continue
            if not all(row['boxes'].get(n,{}).get('pixels',0)>=10 for n in names):continue
            if any(self.ground-self.position(n,[row['time']])[0,2]<7 for n in names if n.startswith('Drone')):continue
            candidates.append(row)
        if not candidates:raise ValueError('No recorded scene frame contains all controlled platforms clearly')
        row=max(candidates,key=lambda r:(r['camera']=='Observer/Overview',min(r['boxes'][n]['pixels'] for n in names)))
        shutil.copy2(self.folder/row['image'],self.out/'scene_overview.png')
        self.image(row,boxes=True).save(self.out/'scene_overview_labeled.png',dpi=(300,300))
        self.sources['scene_overview']=self.source(row);self.metrics['scene_all_platforms_visible']=True
        self.metrics['scene_time_sec']=row['time'];self.scene_row=row

    def navigation(self):
        path=np.array(json.loads((self.folder/'planned_path.json').read_text(encoding='utf-8')))
        actual=self.tracks['Car1'];times=self.track_times['Car1'];goal=np.array(self.config['navigation']['goal_ned'])
        controls=read_rows(self.folder/'controls.csv');start=min(float(r['time']) for r in controls if r['name']=='Car1' and r['goal_x'])
        observed=[r for r in self.frames if r['camera']=='Drone1' and self.ground-self.position('Drone1',[r['time']])[0,2]>20]
        if not observed:raise ValueError('No UAV first-person camera record during navigation')
        def coverage(frame):
            uv,valid=self.project(frame,path);w=int(frame['width']);h=int(frame['height'])
            inside=valid&(uv[:,0]>12)&(uv[:,0]<w-12)&(uv[:,1]>12)&(uv[:,1]<h-12)
            return (int(inside.sum()),int(inside[0])+int(inside[-1]),-abs(frame['time']-(start+2)))
        row=max(observed,key=coverage)
        overlay=self.overlay(row,[(path,'#79f13c')]);overlay.save(self.out/'uav_planned_path.png',dpi=(300,300))
        self.sources['uav_planned_path']={'frame':self.source(row),'path':'planned_path.json',
            'plan_ready_control_time_sec':start,'photograph_precedes_plan':row['time']<start,
            'overlay':'Offline projection of the final path through the recorded camera pose and configured 90 degree FOV; not a screenshot of an online planning display'}
        car_frames=[r for r in self.frames if r['camera']=='Car1' and start+5<r['time']<times[-1]-2]
        car=min(car_frames,key=lambda r:abs(r['time']-(start+times[-1])/2))
        shutil.copy2(self.folder/car['image'],self.out/'ugv_first_person.png');self.sources['ugv_first_person']=self.source(car)
        self.trajectory('ugv_trajectory_3d',['Car1']);self.trajectory('air_ground_trajectories_3d')
        self.panel([(overlay,f'(a) UAV + final plan overlay | t={row["time"]-self.t0:.1f} s',self.sources['uav_planned_path']),
                    (self.image(car),f'(b) UGV view | t={car["time"]-self.t0:.1f} s',self.source(car)),
                    (self.out/'ugv_trajectory_3d.png','(c) Recorded UGV path',{'data':'study_results/trajectories_topic.csv'})],'navigation_panel')
        distance=np.linalg.norm(np.diff(actual[:,:2],axis=0),axis=1).sum();error=cross_track(actual,path)
        self.metrics.update(goal_error_m=float(np.linalg.norm(actual[-1,:2]-goal[:2])),ugv_path_length_m=float(distance),
                            planned_length_m=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum()),
                            cross_track_error_m=statistics(error[times>=start]),navigation_duration_sec=float(times[-1]-start),
                            measured_vertical_range_m=float(np.ptp(actual[:,2])),collision_count=self.manifest['collision_count'])
        with np.load(self.folder/'navigation_grid.npz') as d:
            fig,ax=plt.subplots(figsize=(10,6));free=d['free'];obs=d['observed'];inflated=d['inflated_obstacles'];low=d['low_xy'];res=float(d['resolution_m'])
            image=np.ones((*free.shape,3))*.93;image[obs]=[.82,.9,.94];image[inflated]=[.4,.44,.48];image[free]=[.96,.98,.96]
            ax.imshow(np.transpose(image,(1,0,2)),origin='lower',extent=[low[0],low[0]+res*free.shape[0],low[1],low[1]+res*free.shape[1]],aspect='equal')
            ax.plot(path[:,0],path[:,1],'--',c='#3b9b12',lw=2,label='UAV-derived plan');ax.plot(actual[:,0],actual[:,1],c=COLORS['Car1'],lw=1.8,label='Executed UGV')
            ax.set(xlabel='North (m)',ylabel='East (m)',title='Observed ground, inflated obstacles and executed path');ax.legend()
            self.save(fig,'navigation_map',{'data':['navigation_grid.npz','planned_path.json','study_results/trajectories_topic.csv']})
        fig,axes=plt.subplots(1,2,figsize=(12,4.5));axes[0].plot(times-self.t0,np.linalg.norm(actual[:,:2]-goal[:2],axis=1),c=COLORS['Car1'])
        axes[0].axvline(start-self.t0,c='#888888',ls='--',label='Navigation starts');axes[0].set(xlabel='Simulation time (s)',ylabel='Distance (m)',title='Distance to requested goal');axes[0].legend()
        active=times>=start;axes[1].plot(times[active]-self.t0,error[active],c='#245bea');axes[1].set(xlabel='Simulation time (s)',ylabel='Cross-track error (m)',title='Executed path versus plan')
        for ax in axes:ax.grid(alpha=.2)
        fig.tight_layout();self.save(fig,'navigation_metrics',{'data':['planned_path.json','study_results/trajectories_topic.csv']})

    def tracking(self):
        rows=read_rows(self.folder/'tracking.csv');t=np.array([float(r['time']) for r in rows]);elapsed=t-self.t0
        truth=self.position('Target',t);air=self.position('Drone1',t);ground=self.position('Car1',t)
        air_distance=np.linalg.norm(air-truth,axis=1);ground_distance=np.linalg.norm(ground-truth,axis=1)
        air_standoff=np.linalg.norm(air[:,:2]-truth[:,:2],axis=1)
        ground_standoff=abs(np.linalg.norm(ground[:,:2]-truth[:,:2],axis=1)-self.config['tracking']['follow_distance_m'])
        line_of_sight=np.arctan2(truth[:,1]-ground[:,1],truth[:,0]-ground[:,0])
        yaw=np.interp(t,self.track_times['Car1'],self.yaws['Car1'])
        heading=abs(np.rad2deg(np.arctan2(np.sin(line_of_sight-yaw),np.cos(line_of_sight-yaw))))
        target_yaw=np.interp(t,self.track_times['Target'],self.yaws['Target'])
        alignment=abs(np.rad2deg(np.arctan2(np.sin(target_yaw-yaw),np.cos(target_yaw-yaw))))
        estimates={}
        for prefix in ('uav','ugv'):
            error=np.full(len(rows),np.nan)
            for i,r in enumerate(rows):
                if r[prefix+'_observation_time']:
                    observed=np.array([float(r[prefix+'_estimate_'+a]) for a in 'xy'])
                    reference=self.position('Target',[float(r[prefix+'_observation_time'])])[0,:2]
                    error[i]=np.linalg.norm(observed-reference)
            estimates[prefix]=error
        fusion=np.array([[float(r['fused_'+a]) if r['fused_'+a] else np.nan for a in 'xy'] for r in rows])
        estimates['fused']=np.linalg.norm(fusion-truth[:,:2],axis=1)
        visible_air=np.array([r['uav_visible']=='True' for r in rows]);visible_ground=np.array([r['ugv_visible']=='True' for r in rows])
        under=np.array([r.get('target_in_occlusion_region',r.get('target_origin_under_canopy'))=='True' for r in rows])
        complementary=under&~visible_air&visible_ground&np.array([r['uav_reason']=='target_not_in_image' for r in rows])
        self.metrics.update(target_path_length_m=float(np.linalg.norm(np.diff(self.tracks['Target'][:,:2],axis=0),axis=1).sum()),
            target_estimation_xy_error_m={k:statistics(v) for k,v in estimates.items()},
            horizontal_standoff_error_m={'UAV':statistics(air_standoff),'UGV':statistics(ground_standoff)},
            target_distance_3d_m={'UAV':statistics(air_distance),'UGV':statistics(ground_distance)},
            ugv_line_of_sight_heading_error_deg=statistics(heading),ugv_target_yaw_alignment_error_deg=statistics(alignment),
            uav_visibility_fraction=float(visible_air.mean()),ugv_visibility_fraction=float(visible_ground.mean()),
            ground_only_occlusion_samples=int(complementary.sum()),air_reacquired=self.manifest['air_reacquired'],
            error_reference='Surface-centroid estimates versus target actor origin; target truth interpolated at each observation time',
            standoff_definition='UAV: horizontal distance to target (nominal 0); UGV: absolute horizontal distance minus nominal 6 m. Controller follows an estimated trail, so turns affect radial distance.')
        evaluated=[]
        for i,r in enumerate(rows):
            evaluated.append({'time':t[i],'elapsed_sec':elapsed[i],'uav_estimation_xy_error_m':estimates['uav'][i],
                'ugv_estimation_xy_error_m':estimates['ugv'][i],'fusion_estimation_xy_error_m':estimates['fused'][i],
                'uav_standoff_xy_error_m':air_standoff[i],'ugv_standoff_xy_error_m':ground_standoff[i],
                'uav_target_distance_3d_m':air_distance[i],'ugv_target_distance_3d_m':ground_distance[i],
                'ugv_line_of_sight_heading_error_deg':heading[i],'ugv_target_yaw_alignment_error_deg':alignment[i],
                'uav_visible':bool(visible_air[i]),'ugv_visible':bool(visible_ground[i]),'ground_only_under_canopy':bool(complementary[i])})
        with (self.out/'tracking_evaluation.csv').open('w',encoding='utf-8',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(evaluated[0]));writer.writeheader();writer.writerows(evaluated)
        self.trajectory()
        fig,ax=plt.subplots(figsize=(10,5.5))
        ax.plot(elapsed,air_distance,c=COLORS['Drone1'],label='UAV to target');ax.plot(elapsed,ground_distance,c=COLORS['Car1'],label='UGV to target')
        ax.axhline(self.config['uav_altitude_agl_m'],c=COLORS['Drone1'],ls='--',lw=1,label='UAV nominal height 14 m')
        ax.axhline(self.config['tracking']['follow_distance_m'],c=COLORS['Car1'],ls='--',lw=1,label='UGV nominal standoff 6 m')
        ax.set(xlabel='Simulation time (s)',ylabel='3D distance (m)',title='Measured distances to the moving target');ax.grid(alpha=.2);ax.legend()
        self.save(fig,'tracking_distances',{'data':['study_results/trajectories_topic.csv','tracking.csv'],'includes_startup':True})
        fig,ax=plt.subplots(figsize=(8,5.5));values=[air_standoff,ground_standoff]
        bp=ax.boxplot(values,tick_labels=['UAV (nominal XY: 0 m)','UGV (nominal range: 6 m)'],patch_artist=True,showfliers=False)
        for box,color in zip(bp['boxes'],[COLORS['Drone1'],COLORS['Car1']]):box.set(facecolor=color,alpha=.3)
        for i,v in enumerate(values,1):
            jitter=np.linspace(-.07,.07,len(v));ax.scatter(i+jitter,v,s=4,alpha=.15,color=[COLORS['Drone1'],COLORS['Car1']][i-1])
            st=statistics(v);ax.text(i,float(v.max())+.25,f'Mean {st["mean"]:.2f}\nStd {st["std"]:.2f}\nMax {st["maximum"]:.2f}',ha='center',fontsize=10)
        ax.set(ylabel='Horizontal standoff error (m)',title='Tracking control geometry | Startup included');ax.grid(axis='y',alpha=.2)
        ax.set_ylim(0,max(float(v.max()) for v in values)*1.32+1)
        self.save(fig,'tracking_error_xy',{'data':['study_results/tracking_evaluation.csv'],'definition':self.metrics['standoff_definition']})
        fig,ax=plt.subplots(figsize=(10,5))
        for key,color,label in [('uav',COLORS['Drone1'],'UAV estimate'),('ugv',COLORS['Car1'],'UGV estimate'),('fused','#e88c18','Fused estimate')]:ax.plot(elapsed,estimates[key],color=color,lw=1.2,label=label)
        ax.set(xlabel='Simulation time (s)',ylabel='Target position error in XY (m)',title='Perception error | Surface centroid versus actor origin');ax.legend();ax.grid(alpha=.2)
        self.save(fig,'target_estimation_error',{'data':['tracking.csv','study_results/trajectories_topic.csv'],'truth_usage':'Evaluation only; linear interpolation at observation timestamps'})
        fig,ax=plt.subplots(figsize=(10,4.5));ax.plot(elapsed,heading,c=COLORS['Car1']);ax.axhline(heading.mean(),c='#3b4650',ls='--',label=f'Mean {heading.mean():.2f} deg')
        ax.set(xlabel='Simulation time (s)',ylabel='Absolute heading error (deg)',title='UGV heading versus target line of sight');ax.grid(alpha=.2);ax.legend()
        self.save(fig,'ugv_heading_error',{'data':['study_results/tracking_evaluation.csv'],'definition':'Absolute wrapped target bearing minus UGV yaw; not a paper metric substitution'})
        fig,ax=plt.subplots(figsize=(10,3.8));ax.step(elapsed,visible_air.astype(int),where='post',c=COLORS['Drone1'],label='UAV visible');ax.step(elapsed,visible_ground.astype(int)+1.3,where='post',c=COLORS['Car1'],label='UGV visible (+1.3)')
        ax.fill_between(elapsed,0,2.4,where=complementary,alpha=.15,color='#686868',label='UGV-only observation in occlusion region')
        ax.set(xlabel='Simulation time (s)',ylabel='Visibility',title='Recorded complementary observations');ax.legend(fontsize=9);ax.grid(alpha=.15)
        self.save(fig,'tracking_visibility',{'data':['tracking.csv']})
        self.occlusion_frames()
        # Project all three recorded tracks onto a real observer camera image.
        top=[r for r in self.frames if r['camera']=='Observer/Top']
        row=min(top,key=lambda r:abs(r['time']-np.median(t)))
        overlay=self.overlay(row,[(self.tracks[n],COLORS[n]) for n in ('Target','Drone1','Car1')])
        overlay.save(self.out/'tracking_map.png',dpi=(300,300))
        self.sources['tracking_map']={'frame':self.source(row),'data':'study_results/trajectories_topic.csv','note':'Recorded 3D tracks projected to a reference camera frame; UAV path is at actual altitude'}
        self.panel([(self.out/'trajectories_3d.png','(a) Recorded UAV, UGV and target paths',{'data':'study_results/trajectories_topic.csv'}),
                    (self.out/'tracking_error_xy.png','(b) Defined XY standoff errors',{'data':'study_results/tracking_evaluation.csv'}),
                    (self.out/'tracking_distances.png','(c) Measured 3D distances',{'data':'study_results/tracking_evaluation.csv'})],'tracking_metrics_panel')

    def occlusion_frames(self):
        air=sorted([r for r in self.frames if r['camera']=='Drone1'],key=lambda r:r['time'])
        ground=[r for r in self.frames if r['camera']=='Car1'];pairs=[]
        region=self.config['tracking'].get('occlusion_region',self.config['tracking'].get('canopy'))
        center=np.array(region['center_ned'][:2]);scale=np.array(region['scale'][:2])
        for a in air:
            g=min(ground,key=lambda r:abs(r['time']-a['time']))
            if abs(g['time']-a['time'])>.15:continue
            pose=self.position('Target',[a['time']])[0,:2];under=bool(np.all(abs(pose-center)<scale/2))
            av=a['boxes'].get('Target',{}).get('pixels',0)>=15;gv=g['boxes'].get('Target',{}).get('pixels',0)>=15
            pairs.append((a,g,av,gv,under))
        middle=[p for p in pairs if p[4] and not p[2] and p[3]]
        if not middle:raise ValueError('No synchronized photographic evidence of UAV occlusion with UGV visibility')
        mid=middle[len(middle)//2];before=[p for p in pairs if p[0]['time']<mid[0]['time']-1 and p[2] and p[3]]
        after=[p for p in pairs if p[0]['time']>mid[0]['time']+1 and p[2] and p[3] and not p[4]]
        if not before or not after:raise ValueError('Missing visible before/after photographic evidence')
        chosen=[before[-1],mid,after[0]];items=[]
        for platform,index in [('UAV',0),('UGV',1)]:
            for phase,pair in zip(['Before','Occluded','Reacquired'],chosen):
                row=pair[index];items.append((self.image(row,boxes=True,target_only=True),f'{platform} | {phase} | t={row["time"]-self.t0:.2f} s',self.source(row)))
        self.panel(items,'occlusion_sequence',columns=3)
        self.metrics['occlusion_camera_pair_max_delta_ms']=max(abs(p[0]['time']-p[1]['time'])*1000 for p in chosen)
        self.metrics['occlusion_figure_times_sec']=[p[0]['time'] for p in chosen]

    def formation(self):
        self.trajectory();routes=json.loads((self.folder/'reference_routes.json').read_text(encoding='utf-8'))
        self.metrics['cross_track_error_m']={}
        fig,ax=plt.subplots(figsize=(11,7))
        for name,points in self.tracks.items():
            path=np.array(routes[name]);ax.plot(path[:,0],path[:,1],'--',color=COLORS[name],lw=1,alpha=.5)
            ax.plot(points[:,0],points[:,1],color=COLORS[name],lw=1.6,label=name.replace('Drone','UAV ').replace('Car','UGV '))
            self.metrics['cross_track_error_m'][name]=statistics(cross_track(points,path))
        ax.set(xlabel='North (m)',ylabel='East (m)',title=f'{len(self.tracks)}-agent executed trajectories and reference paths',aspect='equal');ax.legend(ncol=2);ax.grid(alpha=.2)
        self.save(fig,'formation_top',{'data':['study_results/trajectories_topic.csv','reference_routes.json']})
        times=self.track_times['Car1'];positions=np.stack([self.position(n,times) for n in self.tracks],axis=1)
        distances=np.linalg.norm(positions[:,:,None,:]-positions[:,None,:,:],axis=3)
        for i in range(len(self.tracks)):distances[:,i,i]=np.inf
        minimum=distances.min(axis=(1,2));self.metrics['minimum_pairwise_distance_m']=float(minimum.min())
        fig,ax=plt.subplots(figsize=(10,4.5));ax.plot(times-self.t0,minimum,c='#416bb1');ax.axhline(self.config['minimum_separation_m'],ls='--',c='#dc3737',label='Configured stop threshold')
        ax.set(xlabel='Simulation time (s)',ylabel='Minimum 3D pair separation (m)',title=f'Measured separation of {len(self.tracks)} controlled platforms');ax.legend();ax.grid(alpha=.2)
        self.save(fig,'formation_separation',{'data':['study_results/trajectories_topic.csv'],'method':'Positions linearly interpolated onto Car1 timestamps'})

    def performance(self):
        rows=read_rows(self.folder/'performance.csv');counts=read_rows(self.folder/'sensor_counts.csv');rates=[]
        for stream in sorted({r['stream'] for r in counts}):
            subset=[r for r in counts if r['stream']==stream]
            wall=float(subset[-1]['wall_elapsed_sec'])-float(subset[0]['wall_elapsed_sec'])
            sim=float(subset[-1]['time'])-float(subset[0]['time'])
            delta=int(subset[-1]['messages_total'])-int(subset[0]['messages_total'])
            rates.append({'stream':stream,'messages_delta':delta,'wall_hz':delta/wall if wall>0 else None,'sim_hz':delta/sim if sim>0 else None})
        with (self.out/'sensor_rates.csv').open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rates[0]));writer.writeheader();writer.writerows(rates)
        summary={k:statistics([float(r[k]) for r in rows if r.get(k)]) for k in ('fps','memory_mb','cpu_total_percent','real_time_factor')}
        self.metrics['performance']=summary;self.metrics['performance']['mode']=self.config['study']['rendering_mode']
        fig,axes=plt.subplots(2,2,figsize=(12,7))
        for ax,key,title,unit in zip(axes.ravel(),('fps','memory_mb','cpu_total_percent','real_time_factor'),('UE frames','UE resident memory','UE CPU total across cores','Simulation / wall time'),('FPS','MiB','Percent','Ratio')):
            valid=[r for r in rows if r.get(key)]
            if valid:ax.plot([float(r['wall_elapsed_sec']) for r in valid],[float(r[key]) for r in valid],c='#315bb1')
            else:ax.text(.5,.5,'Unavailable (not substituted)',transform=ax.transAxes,ha='center')
            ax.set(xlabel='Wall time (s)',ylabel=unit,title=title);ax.grid(alpha=.2)
        fig.tight_layout();self.save(fig,'performance',{'data':['performance.csv','performance_source.json'],'scope':'Current camera and recording workload; not comparable to another machine'})
        text=['# Measured performance','',f'Run: `{self.folder.name}`. Status: `{self.manifest["status"]}`.','',
              'Actual SDK topic receipt rates; these are not ROS topic rates. Cameras and evidence recording are active.','',
              '| Metric | Mean | Std | Max |','| --- | ---: | ---: | ---: |']
        for key,result in summary.items():
            if not isinstance(result,dict):continue
            text.append(f'| {key} | '+(f'{result["mean"]:.3f} | {result["std"]:.3f} | {result["maximum"]:.3f} |' if result['samples'] else 'unavailable | — | — |'))
        text+=['','Per-stream wall-clock and simulation-clock rates are in `sensor_rates.csv`.','No paper benchmark values are reused as measurements.']
        (self.out/'performance_table.md').write_text('\n'.join(text),encoding='utf-8')

    def export(self):
        if self.manifest['status']!='completed':raise ValueError('Preserve failed trial; do not label its figures as successful results')
        self.scene();getattr(self,self.task)();self.performance()
        write_json(self.out/'metrics.json',self.metrics);write_json(self.out/'figure_sources.json',self.sources)
        root=Path(__file__).resolve().parents[1];hashes={}
        for relative in ('ag/study_figures.py','ag/math3d.py','ag/storage.py','export_cooperative_figures.py'):
            content=(root/relative).read_bytes();target=self.out/'source_snapshot'/relative
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(content)
            hashes[relative]=hashlib.sha256(content).hexdigest()
        write_json(self.out/'source_sha256.json',hashes)
        write_json(self.out/'processing.json',{'created_utc':datetime.now(timezone.utc).isoformat(),
            'python':platform.python_version(),'numpy':np.__version__,'matplotlib':matplotlib.__version__,
            'input_run':self.folder.name,'input_manifest_sha256':hashlib.sha256((self.folder/'manifest.json').read_bytes()).hexdigest(),
            'note':'Offline figure processing snapshot; original acquisition snapshot is in run/source_snapshot'})
        print(json.dumps(self.metrics,indent=2),flush=True)
