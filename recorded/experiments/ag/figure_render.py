"""Full-resolution orthographic figures; no browser decimation or painter sorting.

Mesh pixels use nearest triangle hits and barycentric vertex color. LiDAR pixels
use a depth buffer, preserving occlusion without transparent overdraw.
"""
import json
import numpy as np
import open3d as o3d


LOW=np.array([-5.,25.,-18.])
HIGH=np.array([65.,88.,30.])


def camera(width,height,elevation=32,azimuth=-115,world_width=90):
    el,az=np.deg2rad([elevation,azimuth])
    backward=np.array([np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)])
    right=np.cross([0,0,1],backward);right/=np.linalg.norm(right)
    up=np.cross(backward,right)
    center=(LOW+HIGH)/2;center[2]=1.
    scale=world_width/max(width,1)
    return center,backward,right,up,scale


def crop(points):
    return np.all((points>=LOW)&(points<=HIGH),axis=1)


def mesh_image(mesh,width=3200,height=2600,**angles):
    vertices=np.asarray(mesh.vertices)
    triangles=np.asarray(mesh.triangles)
    mesh=o3d.geometry.TriangleMesh(mesh)
    mesh.remove_triangles_by_mask(~np.all(crop(vertices)[triangles],axis=1))
    mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
    vertices=np.asarray(mesh.vertices);triangles=np.asarray(mesh.triangles)
    colors=np.asarray(mesh.vertex_colors);normals=np.asarray(mesh.vertex_normals)
    scene=o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    center,backward,right,up,scale=camera(width,height,**angles)
    rgb=np.full((height,width,3),255,np.uint8)
    light=np.array([-.35,-.25,1.]);light/=np.linalg.norm(light)
    for start in range(0,height,100):
        y,x=np.mgrid[start:min(start+100,height),:width]
        origin=center+backward*180+(x[...,None]-(width-1)/2)*scale*right+((height-1)/2-y[...,None])*scale*up
        rays=np.concatenate([origin,np.broadcast_to(-backward,origin.shape)],axis=-1).astype('f4')
        hit=scene.cast_rays(o3d.core.Tensor(rays))
        ids=hit['primitive_ids'].numpy();valid=ids<len(triangles)
        uv=hit['primitive_uvs'].numpy()[valid]
        w=np.c_[1-uv.sum(axis=1),uv]
        ids=triangles[ids[valid]]
        c=(colors[ids]*w[:,:,None]).sum(axis=1)
        n=(normals[ids]*w[:,:,None]).sum(axis=1);n/=np.maximum(np.linalg.norm(n,axis=1,keepdims=True),1e-6)
        shade=.74+.26*np.abs(n@light)
        rgb[start:start+len(y)][valid]=np.clip(c*shade[:,None]*255,0,255).astype('u1')
    return rgb,{'rendered_triangles':len(triangles),'rendered_vertices':len(vertices),
                'color':'barycentric interpolation of measured vertex RGB','geometry':'full mesh, no decimation'}


def point_image(points,colors,width=3200,height=2600,splat=True,**angles):
    center,backward,right,up,scale=camera(width,height,**angles)
    q=points-center
    x=np.rint(q@right/scale+(width-1)/2).astype('i4')
    y=np.rint((height-1)/2-q@up/scale).astype('i4')
    depth=-(q@backward)
    keep=(x>=1)&(x<width-1)&(y>=1)&(y<height-1)
    x,y,depth,colors=x[keep],y[keep],depth[keep],colors[keep]
    zbuffer=np.full(width*height,np.inf)
    rgb=np.full((width*height,3),255,np.uint8)
    # Subtle 3-pixel cross splats preserve thin structures at 4K without fog.
    offsets=[(0,0),(-1,0),(1,0),(0,-1),(0,1)] if splat else [(0,0)]
    for dx,dy in offsets:
        pixels=(y+dy)*width+x+dx
        np.minimum.at(zbuffer,pixels,depth)
    # Farthest first makes tie behavior deterministic; nearer points overwrite.
    order=np.argsort(depth)[::-1]
    for dx,dy in offsets:
        pixels=(y+dy)*width+x+dx
        valid=order[depth[order]<=zbuffer[pixels[order]]+1e-7]
        rgb[pixels[valid]]=np.clip(colors[valid]*255,0,255).astype('u1')
    return rgb.reshape(height,width,3)


def height_colors(points):
    # Perceptually separated heights, fixed limits shared by all views.
    knots=np.array([[.09,.20,.40],[.06,.51,.62],[.36,.69,.39],[.95,.71,.16],[.82,.28,.18]])
    t=np.clip((points[:,2]+7)/22,0,1)*4
    i=np.minimum(t.astype(int),3);f=t-i
    return knots[i]*(1-f[:,None])+knots[i+1]*f[:,None]


def render(folder):
    out=folder/'plaza_results'
    points=[];sources=[];counts={}
    for name,color in [('Drone1',[.10,.35,.88]),('Car1',[.08,.62,.26])]:
        with np.load(out/f'lidar_{name}_timed.npz') as data:p=data['points_ned']*[1,1,-1]
        p=p[crop(p)];points.append(p);sources.append(np.broadcast_to(color,p.shape));counts[name]=len(p)
    points=np.vstack(points);sources=np.vstack(sources)
    print(f'Rendering {len(points):,} measured LiDAR points without subsampling',flush=True)
    for filename,colors in [('pointcloud',height_colors(points)),('pointcloud_sources',sources)]:
        rgb=point_image(points,colors)
        o3d.io.write_image(str(out/f'{filename}_render.png'),o3d.geometry.Image(rgb))
    mesh=o3d.io.read_triangle_mesh(str(out/'surface/surface_tsdf_neu.ply'))
    print(f'Rendering full surface: {len(mesh.triangles):,} triangles',flush=True)
    rgb,stats=mesh_image(mesh)
    o3d.io.write_image(str(out/'colored_surface_render.png'),o3d.geometry.Image(rgb))
    # A closer view makes local resolving power inspectable, without sharpening.
    original_low,original_high=LOW.copy(),HIGH.copy()
    LOW[:]=[16,44,-15];HIGH[:]=[51,70,22]
    # Keep the same renderer but render a larger image before cropping its center.
    rgb,detail=mesh_image(mesh,width=4800,height=3600,elevation=46,azimuth=-115)
    rgb=np.ascontiguousarray(rgb[700:3000,1300:3500])
    o3d.io.write_image(str(out/'colored_surface_detail_render.png'),o3d.geometry.Image(rgb))
    LOW[:]=original_low;HIGH[:]=original_high
    report={'input_run':folder.name,'point_counts_in_figure':counts,'surface':stats,
            'output_pixels':[3200,2600],'elevation_deg':32,'azimuth_deg':-115,
            'limitations':'Rendering cannot restore camera texture detail lost during acquisition or TSDF fusion.'}
    (out/'figure_quality.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
