"""Paper-style trajectories and framed full-data renders, without mesh decimation."""
import csv
import json
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image,ImageDraw,ImageFont


def frame_image(source,destination,title,note):
    im=Image.open(source).convert('RGB')
    yy,xx=np.nonzero(np.any(np.asarray(im)<245,axis=2))
    if not len(xx):raise ValueError('Empty rendering: '+str(source))
    im=im.crop((max(0,int(xx.min())-30),max(0,int(yy.min())-30),
                min(im.width,int(xx.max())+31),min(im.height,int(yy.max())+31)))
    canvas=Image.new('RGB',(im.width+100,im.height+205),'white');canvas.paste(im,(50,125))
    draw=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',45)
    small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',26)
    draw.text((50,25),title,font=font,fill='#18242e')
    draw.text((50,85),note,font=small,fill='#526170')
    canvas.save(destination,dpi=(300,300))


def frame_structural_figures(folder):
    out=folder/'plaza_results'
    for name,title,note in [
        ('pointcloud_structure','CityPark plaza | Structural LiDAR view',
         'UAV blue / UGV green | Neighborhood support filter + flat-area thinning | Single-pixel points'),
        ('pointcloud_structure_detail','CityPark plaza | Structural LiDAR detail',
         'Central plaza crop | Same filtered points | No surface mesh or synthetic contours')]:
        source=out/f'{name}_render.png'
        if source.exists():frame_image(source,out/f'{name}.png',title,note)


def plot(folder):
    out=folder/'plaza_results'
    old=out/'figures_before_v2';old.mkdir(exist_ok=True)
    for name in ('pointcloud','colored_surface','trajectories_3d','trajectories_top'):
        src=out/f'{name}.png';dst=old/src.name
        if src.exists() and not dst.exists():shutil.copy2(src,dst)
    rows=list(csv.DictReader((folder/'trajectories.csv').open(encoding='utf-8')))
    config=json.loads((folder/'config.json').read_text(encoding='utf-8'))
    ground=float(config['ground_z_ned'])
    tracks={n:np.array([[float(r['x']),float(r['y']),ground-float(r['z'])] for r in rows if r['name']==n]) for n in ('Car1','Drone1')}
    plt.rcParams.update({'font.size':12,'figure.facecolor':'white','savefig.facecolor':'white','font.family':'DejaVu Sans'})
    for kind in ('trajectories_3d','trajectories_context','trajectories_top'):
        top=kind.endswith('top')
        fig=plt.figure(figsize=(10,8.5))
        ax=fig.add_subplot(111) if top else fig.add_subplot(111,projection='3d',computed_zorder=False)
        if kind!='trajectories_3d':
            with np.load(out/'plot_points.npz') as d:
                for n in d.files:
                    p=d[n][::8]*[1,1,-1];p[:,2]+=ground
                    if top:ax.scatter(p[:,0],p[:,1],s=.15,c='#98a5b0',alpha=.4,zorder=0)
                    else:ax.scatter(*p.T,s=.07,c='#a9b3bd',alpha=.3,depthshade=False,zorder=0)
        for n,color,label,start,end in [('Car1','#1746f5','UGV','#00b42a','#f52222'),
                                        ('Drone1','#ed2224','UAV','#00d8e5','#e600de')]:
            p=tracks[n];q=p[:,:2] if top else p
            ax.plot(*q.T,color=color,lw=1.9,label=label,zorder=4)
            for index,c,m,word in [(0,start,'o','Start'),(-1,end,'s','End')]:
                kwargs={'depthshade':False} if not top else {}
                ax.scatter(*q[index],c=c,s=75,marker=m,label=label+' '+word,zorder=5,**kwargs)
        ax.set_xlabel('X / North (m)',labelpad=10);ax.set_ylabel('Y / East (m)',labelpad=10)
        if top:ax.set_aspect('equal');ax.grid(alpha=.2)
        else:
            ax.set_zlabel('Z above plaza (m)',labelpad=10)
            ax.view_init(elev=20,azim=-60)
            if kind=='trajectories_3d':
                allp=np.vstack(list(tracks.values()))
                lo=allp.min(axis=0)-[3,3,1];hi=allp.max(axis=0)+[3,3,1]
                ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(0,30)
                ax.set_box_aspect([hi[0]-lo[0],hi[1]-lo[1],30])
            else:ax.set_box_aspect([70,63,40])
            for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
                axis.pane.fill=True;axis.pane.set_facecolor((.97,.97,.97,1))
                axis._axinfo['grid'].update(color=(.7,.7,.7,.5),linewidth=.6)
        ax.legend(loc='upper left',fontsize=10,framealpha=1)
        ax.set_title('CityPark plaza | Measured air-ground trajectories',pad=16)
        fig.text(.5,.025,'Actual samples, including takeoff and approach. Height origin: plaza ground.',ha='center',fontsize=10,color='#53616d')
        fig.subplots_adjust(left=.02,right=.93,top=.90,bottom=.09)
        fig.savefig(out/f'{kind}.png',dpi=330)
        if kind=='trajectories_3d':
            fig.savefig(out/f'{kind}.pdf');fig.savefig(out/f'{kind}.svg')
        plt.close(fig)
    renders=[('pointcloud','CityPark plaza | LiDAR geometry',
              'Full measured points in view | Height colors: navy (low) to orange (high) | Opaque depth buffering'),
             ('pointcloud_sources','CityPark plaza | LiDAR by platform',
              'Full measured points in view | UAV blue / UGV green | Occlusion-aware rendering'),
             ('colored_surface','CityPark plaza | Colored surface',
              'Full-resolution TSDF mesh | Interpolated vertex RGB | No preview-mesh decimation'),
             ('colored_surface_detail','CityPark plaza | Surface detail',
              'Central plaza close-up | Same measured mesh and colors | No synthetic texture')]
    for name,title,note in renders:
        source=out/f'{name}_render.png'
        if source.exists():frame_image(source,out/f'{name}.png',title,note)
    frame_structural_figures(folder)
