"""NED metres, quaternion wxyz; sensor capture poses, never arrival-time poses."""
import math
import numpy as np


def xyz(value):
    return np.array([value[k] for k in ('x', 'y', 'z')], dtype=float)


def rotation(q):
    if isinstance(q, dict):
        q = [q[k] for k in ('w', 'x', 'y', 'z')]
    q = np.asarray(q, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid capture quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_parts(pose):
    return xyz(pose.get('position', pose.get('translation'))), rotation(pose.get('orientation', pose.get('rotation')))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def lidar_world(message):
    points = np.asarray(message['point_cloud'], dtype=float).reshape(-1, 3)
    origin, rot = pose_parts(message['pose'])
    # No-return zeros and non-finite returns cannot be evidence of a surface.
    points = points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > .05)]
    return points @ rot.T + origin


def camera_pose(message):
    p = np.array([message[f'pos_{k}'] for k in 'xyz'], dtype=float)
    r = rotation([message[f'rot_{k}'] for k in 'wxyz'])
    if not np.isfinite(p).all():
        raise ValueError('Image has no valid capture position')
    return p, r


def depth_world(depth_m, message, fov=90, stride=4, mask=None, pixel_signs=(1, 1)):
    """Perspective depth is RANGE, not axial depth. Camera frame: forward/right/down."""
    h, w = depth_m.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    d = depth_m[::stride, ::stride]
    good = np.isfinite(d) & (d > .1) & (d < 65.0)
    if mask is not None:
        good &= mask[::stride, ::stride]
    f = w / (2 * math.tan(math.radians(fov)/2))
    rays = np.stack([np.ones_like(u), pixel_signs[0]*(u-w/2)/f,
                     pixel_signs[1]*(v-h/2)/f], axis=-1)[good]
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    p, r = camera_pose(message)
    return (rays * d[good, None]) @ r.T + p


def voxel(points, size=.3):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        return points
    _, ids = np.unique(np.floor(points/size).astype(np.int64), axis=0, return_index=True)
    return points[np.sort(ids)]


def icp(source, target, gate=2., iterations=30, limit=1800):
    """Return the actual rigid transform as well as post-transform NN residuals."""
    a, b = voxel(source), voxel(target)
    a = a[np.linspace(0, len(a)-1, min(len(a), limit), dtype=int)] if len(a) else a
    b = b[np.linspace(0, len(b)-1, min(len(b), limit), dtype=int)] if len(b) else b
    result = {'status': 'insufficient_points', 'source_points': len(a), 'target_points': len(b)}
    if min(len(a), len(b)) < 10:
        return result, np.eye(4)

    def nearest(p):
        ids, distances = [], []
        for chunk in np.array_split(p, max(1, math.ceil(len(p)/200))):
            d = ((chunk[:, None] - b[None])**2).sum(axis=2)
            j = d.argmin(axis=1)
            ids.extend(j)
            distances.extend(d[np.arange(len(j)), j])
        return np.array(ids), np.sqrt(distances)

    transform = np.eye(4)
    old = float('inf')
    for step in range(iterations):
        moved = a @ transform[:3, :3].T + transform[:3, 3]
        j, d = nearest(moved)
        keep = d < gate
        if keep.sum() < 10:
            result['status'] = 'no_correspondences'
            return result, transform
        p, q = moved[keep], b[j[keep]]
        pc, qc = p.mean(axis=0), q.mean(axis=0)
        u, s, vt = np.linalg.svd((p-pc).T @ (q-qc))
        if s[1] < 1e-8:
            result['status'] = 'degenerate_geometry'
            return result, transform
        r = vt.T @ u.T
        if np.linalg.det(r) < 0:
            vt[-1] *= -1
            r = vt.T @ u.T
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = r, qc-r@pc
        transform = delta @ transform
        _, d = nearest(a @ transform[:3, :3].T + transform[:3, 3])
        keep = d < gate
        rmse = float(np.sqrt(np.mean(d[keep]**2))) if keep.any() else float('inf')
        if abs(old-rmse) < 1e-5:
            break
        old = rmse
    result.update(status='ok', rmse_m=rmse, fitness=float(keep.mean()), iterations=step+1,
                  transform=transform.tolist(), gate_m=gate)
    return result, transform
