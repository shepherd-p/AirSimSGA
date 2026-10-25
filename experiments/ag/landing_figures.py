from .fonts import font as portable_font
"""Rebuild scientific figures from recorded samples and original camera frames."""
import csv
import json
import shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
from .storage import write_json


def rows(path):
    if not path.exists():return []
    with path.open(encoding='utf-8',newline='') as f:return list(csv.DictReader(f))


def export_trial(path):
    path=Path(path)
    data=rows(path/'trajectory.csv')
    manifest=json.loads((path/'manifest.json').read_text(encoding='utf-8'))
    trial_config=json.loads((path/'config.json').read_text(encoding='utf-8'))
    output=path/'derived';output.mkdir(exist_ok=True)
    if not data:
        write_json(output/'figures.json',{'status':'no_trajectory','reason':manifest.get('reason')})
        return
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,
                          'axes.spines.right':False,'savefig.dpi':240})
    def column(name):return np.array([float(r[name]) if r.get(name) else np.nan for r in data])
    t=column('elapsed_sec')
    xyz={key:np.column_stack([column(key+'_'+a) for a in 'xyz']) for key in ('uav','usv','deck','visual_deck')}
    reason=manifest.get('reason','unverified').replace('_',' ')
    verified_title='Verified moving-deck landing' if manifest['trial']['speed_mps']>0 else 'Verified stationary-deck landing'
    title=(verified_title if manifest.get('success') else 'Recorded trial: '+reason)
    if manifest.get('success') and not trial_config.get('require_rotor_verification'):
        title='Recorded contact: rotor shutdown unverified'
    colors={'uav':'#0072B2','usv':'#D55E00','deck':'#009E73','visual_deck':'#CC79A7'}
    labels={'uav':'UAV','usv':'USV centre','deck':'Deck reference (GT)','visual_deck':'Vision estimate'}
    # A shared, fixed height datum keeps the vessel, deck, and UAV comparable.
    height_datum=float(xyz['usv'][0,2])
    fig=plt.figure(figsize=(11,8));ax=fig.add_axes([.04,.13,.82,.79],projection='3d')
    for key in xyz:
        p=xyz[key];good=np.isfinite(p).all(axis=1)
        ax.plot(p[good,0],p[good,1],height_datum-p[good,2],label=labels[key],color=colors[key],
                lw=2.2 if key=='uav' else 1.4,ls='--' if key=='visual_deck' else '-')
        if key in ('usv','deck') and good.any():
            q=p[good][-1]
            ax.scatter(q[0],q[1],height_datum-q[2],s=45,color=colors[key],
                       marker='s' if key=='usv' else '^',depthshade=False)
    p=xyz['uav']
    ax.scatter(*[p[0,0],p[0,1],height_datum-p[0,2]],s=45,color=colors['uav'],marker='o')
    ax.scatter(*[p[-1,0],p[-1,1],height_datum-p[-1,2]],s=80,color=colors['uav'],marker='X')
    allp=np.concatenate([xyz[k] for k in ('uav','usv','deck')])
    extent=np.maximum(np.ptp(allp,axis=0)*1.1,[20.,20.,12.])
    center=(allp.max(0)+allp.min(0))/2
    ax.set_xlim(center[0]-extent[0]/2,center[0]+extent[0]/2)
    ax.set_ylim(center[1]-extent[1]/2,center[1]+extent[1]/2)
    ax.set_zlim(height_datum-center[2]-extent[2]/2,height_datum-center[2]+extent[2]/2)
    ax.set_box_aspect(extent);ax.set_proj_type('ortho')
    from matplotlib.ticker import MaxNLocator
    ax.xaxis.set_major_locator(MaxNLocator(3));ax.yaxis.set_major_locator(MaxNLocator(5));ax.zaxis.set_major_locator(MaxNLocator(4))
    ax.tick_params(labelsize=9,pad=2)
    ax.view_init(elev=26,azim=-58)
    ax.set(xlabel='North (m)',ylabel='East (m)',zlabel='',title=title)
    ax.text2D(1.03,.5,'Up (m)',transform=ax.transAxes,rotation=90,va='center')
    ax.xaxis.labelpad=12;ax.yaxis.labelpad=12;ax.zaxis.labelpad=10
    ax.legend(loc='upper left',fontsize=9)
    fig.text(.03,.035,'Measured samples; no trajectory smoothing. Height zero: initial USV origin.',fontsize=9)
    fig.text(.03,.015,'USV centre and deck centre are distinct references; squares / triangles mark their final positions.',fontsize=9)
    fig.savefig(output/'trajectory_3d.png',bbox_inches='tight');fig.savefig(output/'trajectory_3d.pdf',bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(figsize=(8.5,7))
    for key,p in xyz.items():
        ax.plot(p[:,1],p[:,0],color=colors[key],label=labels[key],lw=1.8,ls='--' if key=='visual_deck' else '-')
    ax.set(xlabel='East (m)',ylabel='North (m)',title=title);ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend()
    fig.savefig(output/'trajectory_top.png',bbox_inches='tight');plt.close(fig)
    fields=['horizontal_error_m','height_over_deck_m','relative_speed_mps','boat_speed_mps']
    units=['Horizontal error (m)','Height above deck (m)','Relative speed (m/s)','USV speed (m/s)']
    palette=['#0072B2','#009E73','#CC79A7','#D55E00']
    if 'rotor_total_thrust_n' in data[0]:
        fields+=['rotor_total_thrust_n'];units+=['Total rotor thrust (N)'];palette+=['#555555']
    fig,axes=plt.subplots(len(fields),1,figsize=(11,2.5*len(fields)),sharex=True)
    for ax,key,label,color in zip(axes,fields,units,palette):
        ax.plot(t,column(key),lw=1.5,color=color);ax.set_ylabel(label);ax.grid(alpha=.2)
    axes[3].plot(t,column('desired_speed_mps'),'k--',lw=1,label='Commanded speed');axes[3].legend()
    cuts=np.flatnonzero(column('motors_off')>0)
    if len(cuts):
        for ax in axes:ax.axvline(t[cuts[0]],color='black',ls=':',lw=1)
        axes[0].annotate('Motor cut after contact',xy=(t[cuts[0]],0),xytext=(.5,.85),textcoords='axes fraction',arrowprops={'arrowstyle':'->'})
    axes[0].set_title(title);axes[-1].set_xlabel('Elapsed simulation time (s)')
    fig.tight_layout();fig.savefig(output/'landing_metrics.png');fig.savefig(output/'landing_metrics.pdf');plt.close(fig)
    frames=rows(path/'frames.csv')
    recognized=[f for f in frames if f['camera']=='down' and f['visible']=='1']
    selected=[]
    for phase in dict.fromkeys(f['phase'] for f in recognized):
        group=[f for f in recognized if f['phase']==phase]
        selected.append(group[len(group)//2])
    if len(selected)<4 and recognized:
        selected.extend(recognized[i] for i in np.linspace(0,len(recognized)-1,min(4,len(recognized)),dtype=int))
    selected=list({f['time_ns']:f for f in selected}.values())
    selected=sorted(selected,key=lambda f:int(f['time_ns']))[:5]
    if selected:
        fig,axes=plt.subplots(len(selected),2,figsize=(12,4.3*len(selected)),squeeze=False)
        for aa,f in zip(axes,selected):
            raw=path/f['raw_image']
            from .landing_vision import annotate
            import cv2
            detection=json.loads((path/f['detection']).read_text(encoding='utf-8'))
            ann=output/f"recognition_{f['time_ns']}.png"
            cv2.imwrite(str(ann),annotate(cv2.imread(str(raw)),detection,f['phase']))
            for ax,img,label in zip(aa,[raw,ann],['Original camera RGB','Actual AprilTag detections']):
                ax.imshow(Image.open(img));ax.axis('off');ax.set_title(f"{label} | {f['phase']} | {int(f['time_ns'])/1e9:.2f} s",fontsize=10)
        fig.tight_layout();fig.savefig(output/'recognition_sequence.png',dpi=160,bbox_inches='tight');plt.close(fig)
    visibility=rows(path/'scene_visibility.csv')
    candidates=[]
    for v in visibility:
        boxes=json.loads(v['robots'])
        cfg=json.loads((path/'config.json').read_text(encoding='utf-8'))
        if cfg['uav'] in boxes and cfg['usv'] in boxes:
            candidates.append((min(boxes[cfg['uav']]['pixels'],boxes[cfg['usv']]['pixels']),v,boxes))
    if candidates:
        _,v,boxes=max(candidates,key=lambda item:item[0])
        original=path/'frames/overview'/f"{v['time_ns']}.png"
        shutil.copyfile(original,output/'scene_original.png')
        im=Image.open(original).convert('RGB');draw=ImageDraw.Draw(im)
        try:font=portable_font(24)
        except OSError:font=ImageFont.load_default()
        for name,box in boxes.items():
            x0,y0,x1,y1=box['bbox'];color='#32d2ff' if name==cfg['uav'] else '#ffac4d'
            draw.rectangle([x0-5,y0-5,x1+5,y1+5],outline=color,width=3)
            draw.text((max(5,x0),max(5,y0-35)),name,font=font,fill=color,stroke_width=1,stroke_fill='black')
        im.save(output/'scene_labeled.png')
        write_json(output/'scene_provenance.json',{'source':str(original.relative_to(path)),
                   'time_ns':int(v['time_ns']),'boxes':boxes,
                   'label_source':'Simulator segmentation, for scene identity only, not vision detector output'})
    write_json(output/'figures.json',{'source':'trajectory.csv and original camera frames',
          'success':manifest.get('success',False),'reason':manifest.get('reason'),
          'physical_rotor_shutdown_required':bool(trial_config.get('require_rotor_verification')),
          'height_datum_ned_m':height_datum,
          'recognition_frames':[f['time_ns'] for f in selected],
          'images':[p.name for p in output.glob('*.png')]})
    from .landing_evidence import digest
    from .storage import ROOT
    from datetime import datetime,timezone
    snapshot=output/'source_snapshot/ag';snapshot.mkdir(parents=True,exist_ok=True)
    hashes={}
    for source in (ROOT/'ag').glob('*.py'):
        target=snapshot/source.name;shutil.copyfile(source,target)
        hashes['ag/'+source.name]=digest(target)
    write_json(output/'processing.json',{'exported_at_utc':datetime.now(timezone.utc).isoformat(),
       'input_manifest_sha256':digest(path/'manifest.json'),'figure_code_sha256':hashes,
       'figure_files_sha256':{p.name:digest(p) for p in output.glob('*') if p.suffix in ('.png','.pdf')},
       'note':'Derived figures may be regenerated; the sealed raw evidence inventory is not rewritten'})


def export_suite(root):
    root=Path(root)
    trials=[root] if (root/'manifest.json').exists() else sorted(root.glob('trial_*'))
    for trial in trials:export_trial(trial)
    import html
    blocks=[]
    for trial in trials:
        manifest=json.loads((trial/'manifest.json').read_text(encoding='utf-8'))
        config=json.loads((trial/'config.json').read_text(encoding='utf-8'))
        qualification='' if config.get('require_rotor_verification') else '；旧版记录未验证实际旋翼停转，不能作为关桨着陆结论'
        rel=trial.relative_to(root).as_posix()
        images=''.join(f'<figure><img src="{rel}/derived/{p.name}" loading="lazy"><figcaption>{html.escape(p.stem)}</figcaption></figure>' for p in (trial/'derived').glob('*.png') if p.name!='scene_original.png')
        blocks.append(f'<section><h2>{html.escape(trial.name)}</h2><p>{html.escape(manifest.get("reason",""))} — recorded success: {manifest.get("success",False)}{qualification}</p>{images}</section>')
    page='<!doctype html><meta charset="utf-8"><title>Moving-deck experiment evidence</title><style>body{font:16px system-ui;margin:30px auto;max-width:1250px;background:#f5f7fa;color:#1c2a38}section{background:white;padding:24px;margin:24px 0}img{width:100%;height:auto}figure{margin:24px 0}h1{font-size:28px}</style><h1>无人机—移动无人艇降落实验记录</h1><p>原始仿真相机、实际 AprilTag 检测及测量轨迹。每次试验的成功或失败按其真实状态展示。</p>'+''.join(blocks)
    (root/'index.html').write_text(page,encoding='utf-8')
    return root/'index.html'
