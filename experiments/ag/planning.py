"""Sensor-built layered surface graph. Unknown cells are never implicitly free."""
import heapq
import math
from collections import defaultdict
import numpy as np


def clear_ego_footprint(points, position, yaw, surface_z, resolution=.75):
    """Standard ego-footprint prior, explicitly separate from observed map evidence.

    Remove the current robot's above-ground returns and mark only its occupied
    footprint as ground. Caller must verify a settled, non-colliding robot first.
    No unknown cells outside the robot footprint are filled.
    """
    points = np.asarray(points).reshape(-1,3)
    delta = points[:,:2]-np.asarray(position)[:2]
    r = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
    local = delta@r.T
    body = (abs(local[:,0])<2.9)&(abs(local[:,1])<1.35)&(points[:,2]<surface_z-.3)&(points[:,2]>surface_z-2.5)
    prior=[]
    for x in np.arange(-2.9,2.9,resolution/2):
        for y in np.arange(-1.35,1.35,resolution/2):
            xy = np.array([x,y])@r+np.asarray(position)[:2]
            prior.append([xy[0],xy[1],surface_z])
    return np.vstack([points[~body],prior]), np.asarray(prior)


class SurfaceMap:
    def __init__(self, points, resolution=.75, height_bin=.35, max_step=.25,
                 max_slope=.30, clearance=1.6, radius=1.1, min_points=2):
        self.resolution, self.max_step, self.max_slope = resolution, max_step, max_slope
        groups = defaultdict(list)
        for p in np.asarray(points).reshape(-1, 3):
            if np.isfinite(p).all():
                groups[tuple(np.floor(p/[resolution, resolution, height_bin]).astype(int))].append(p)
        self.nodes = {k: np.mean(v, axis=0) for k, v in groups.items() if len(v) >= min_points}
        self.columns = defaultdict(list)
        for k in self.nodes:
            self.columns[k[:2]].append(k)
        # Points above a surface within the vehicle footprint block that surface.
        blocked = set()
        cells = math.ceil(radius/resolution)
        for k, p in self.nodes.items():
            for dx in range(-cells, cells+1):
                for dy in range(-cells, cells+1):
                    for other in self.columns.get((k[0]+dx, k[1]+dy), []):
                        q = self.nodes[other]
                        if np.linalg.norm(q[:2]-p[:2]) <= radius and .4 < p[2]-q[2] < clearance:
                            blocked.add(k)
        self.nodes = {k: p for k, p in self.nodes.items() if k not in blocked}
        self.columns = defaultdict(list)
        for k in self.nodes:
            self.columns[k[:2]].append(k)

    def near(self, position, horizontal=2., vertical=1.0):
        p = np.asarray(position)
        choices = [(np.linalg.norm(q-p), k) for k, q in self.nodes.items()
                   if np.linalg.norm(q[:2]-p[:2]) <= horizontal and abs(q[2]-p[2]) <= vertical]
        if not choices:
            raise ValueError('Start/goal has no observed traversable surface at its height')
        return min(choices)[1]

    def neighbors(self, key):
        p = self.nodes[key]
        # Four-connected edges avoid diagonal corner cutting through unobserved cells.
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for other in self.columns.get((key[0]+dx, key[1]+dy), []):
                q = self.nodes[other]
                dxy = np.linalg.norm(q[:2]-p[:2])
                if abs(q[2]-p[2]) <= self.max_step + self.max_slope*dxy:
                    yield other, float(np.linalg.norm(q-p))

    def route(self, start, goal):
        a, b = self.near(start), self.near(goal)
        queue, cost, parent = [(0., a)], {a: 0.}, {}
        while queue:
            _, current = heapq.heappop(queue)
            if current == b:
                keys = [b]
                while keys[-1] != a:
                    keys.append(parent[keys[-1]])
                return np.array([self.nodes[k] for k in reversed(keys)])
            for nxt, length in self.neighbors(current):
                score = cost[current]+length
                if score < cost.get(nxt, float('inf')):
                    cost[nxt], parent[nxt] = score, current
                    heapq.heappush(queue, (score+np.linalg.norm(self.nodes[nxt]-self.nodes[b]), nxt))
        raise ValueError('No observed connected route: scan missing ground or choose reachable goal')


def rover_command(position, yaw, target, speed, desired_speed=1., max_engine=.22):
    delta = np.asarray(target)[:2]-np.asarray(position)[:2]
    angle = math.atan2(delta[1], delta[0])-yaw
    angle = math.atan2(math.sin(angle), math.cos(angle))
    if np.linalg.norm(delta) < .3:
        return 0., 0., 1.
    # Existing fast-physics uses curvature = sin(steering) / wheelbase (2.45m).
    curvature = 2*math.sin(angle)/max(np.linalg.norm(delta),1.5)
    steering = float(np.clip(math.asin(np.clip(2.45*curvature,-1.,1.)), -1.05, 1.05))
    desired = desired_speed * max(.2, math.cos(angle))
    engine = float(np.clip(.09*(desired-speed), 0., max_engine))
    brake = float(np.clip((speed-desired)*.2, 0., .7))
    return engine, steering, brake


class Tracker:
    """Timestamp-aware alpha-beta fusion; expired observations cannot update state."""
    def __init__(self, max_age=.4, prediction_horizon=2.):
        self.max_age, self.horizon = max_age, prediction_horizon
        self.p, self.v, self.t, self.last_seen = None, np.zeros(3), None, None

    def update(self, now, observations):
        fresh = [o for o in observations if o is not None and 0 <= now-o['time'] <= self.max_age]
        if self.t is not None and now < self.t:
            raise ValueError('Simulation time moved backwards')
        dt = max(0., now-self.t) if self.t is not None else 0.
        predicted = self.p+self.v*dt if self.p is not None else None
        if fresh:
            weights = np.array([max(.001, o['confidence']) for o in fresh])
            observed = np.average([o['position'] for o in fresh], axis=0, weights=weights)
            if predicted is None:
                self.p = observed
            else:
                residual = observed-predicted
                self.p = predicted+.75*residual
                if dt > .01:
                    self.v += .15*residual/dt
                    norm = np.linalg.norm(self.v)
                    if norm > 10:
                        self.v *= 10/norm
            self.last_seen = now
        else:
            self.p = predicted
        self.t = now
        valid = self.p is not None and now-self.last_seen <= self.horizon
        return (self.p.copy() if valid else None), bool(fresh)
