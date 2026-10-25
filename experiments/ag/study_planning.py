"""Observed flat-ground planning for the existing fixed-height rover physics."""
import heapq
import math
import numpy as np


class ObservedGroundPlanner:
    def __init__(self,points,start,options,ground_z):
        import cv2
        self.opt=options;self.step=options['resolution_m'];self.z=ground_z
        self.low=np.asarray(options['roi_xy'][0],float);high=np.asarray(options['roi_xy'][1],float)
        self.shape=tuple(np.ceil((high-self.low)/self.step).astype(int)+1)
        self.ground=np.zeros(self.shape,np.int32);self.obstacle=np.zeros(self.shape,np.int32)
        p=np.asarray(points);idx=np.floor((p[:,:2]-self.low)/self.step).astype(int)
        inside=np.all((idx>=0)&(idx<self.shape),axis=1);p=p[inside];idx=idx[inside]
        height=ground_z-p[:,2]
        floor=abs(height)<options['floor_tolerance_m']
        obs=(height>options['obstacle_height_range_m'][0])&(height<options['obstacle_height_range_m'][1])
        np.add.at(self.ground,(idx[floor,0],idx[floor,1]),1)
        np.add.at(self.obstacle,(idx[obs,0],idx[obs,1]),1)
        self.observed=self.ground>=options['minimum_cell_points']
        blocked=(self.obstacle>=options['minimum_cell_points'])&(self.obstacle/(self.ground+self.obstacle+1)>.08)
        radius=options['vehicle_radius_m'];n=math.ceil(radius/self.step)
        yy,xx=np.mgrid[-n:n+1,-n:n+1];kernel=((xx*self.step)**2+(yy*self.step)**2<=radius**2).astype('u1')
        self.inflated=cv2.dilate(blocked.astype('u1'),kernel)>0
        self.free=self.observed&~self.inflated
        self.clearance=cv2.distanceTransform(self.free.astype('u1'),cv2.DIST_L2,5)*self.step

    def cell(self,p):return tuple(np.floor((np.asarray(p)[:2]-self.low)/self.step).astype(int))

    def position(self,key):return np.r_[self.low+(np.array(key)+.5)*self.step,self.z]

    def visible(self,a,b):
        samples=np.linspace(a,b,max(2,int(np.linalg.norm((np.array(b)-a)))*3+1))
        cells=np.rint(samples).astype(int)
        return bool(np.all(self.free[cells[:,0],cells[:,1]]))

    def route(self,start,goal):
        def nearest(p):
            candidates=np.argwhere(self.free)
            distance=np.linalg.norm(self.low+(candidates+.5)*self.step-np.asarray(p)[:2],axis=1)
            if not len(distance) or distance.min()>1.1:raise ValueError('Start/goal lacks observed traversable ground')
            return tuple(candidates[distance.argmin()])
        a,b=nearest(start),nearest(goal);q=[(0.,a)];cost={a:0.};parent={};closed=set()
        while q:
            _,current=heapq.heappop(q)
            if current in closed:continue
            closed.add(current)
            if current==b:break
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                nxt=(current[0]+dx,current[1]+dy)
                if not(0<=nxt[0]<self.shape[0] and 0<=nxt[1]<self.shape[1] and self.free[nxt]):continue
                if dx and dy and not(self.free[current[0]+dx,current[1]] and self.free[current[0],current[1]+dy]):continue
                score=cost[current]+math.hypot(dx,dy)*(1+.4/max(float(self.clearance[nxt]),.25))
                if score<cost.get(nxt,float('inf')):
                    cost[nxt]=score;parent[nxt]=current
                    heapq.heappush(q,(score+math.hypot(nxt[0]-b[0],nxt[1]-b[1]),nxt))
        if b not in cost:raise ValueError('No connected route through observed ground')
        keys=[b]
        while keys[-1]!=a:keys.append(parent[keys[-1]])
        keys.reverse();simplified=[keys[0]];index=0
        while index<len(keys)-1:
            j=min(index+8,len(keys)-1)
            while j>index+1 and not self.visible(keys[index],keys[j]):j-=1
            simplified.append(keys[j]);index=j
        return np.array([self.position(k) for k in simplified])


def trailing_point(history,distance):
    """Arc-length target from estimated observations only; never the truth route."""
    pts=np.asarray(history)
    if len(pts)==0:return None
    for i in range(len(pts)-1,0,-1):
        length=np.linalg.norm(pts[i,:2]-pts[i-1,:2])
        if length>=distance and length>1e-9:return pts[i]+(pts[i-1]-pts[i])*(distance/length)
        distance-=length
    return pts[0].copy()


def path_lookahead(path,position,lookahead=2.5):
    path=np.asarray(path);position=np.asarray(position)
    starts=path[:-1,:2];segments=np.diff(path[:,:2],axis=0)
    lengths=np.linalg.norm(segments,axis=1)
    fraction=np.clip(np.sum((position[:2]-starts)*segments,axis=1)/np.maximum(lengths**2,1e-12),0,1)
    nearest=starts+fraction[:,None]*segments
    index=int(np.linalg.norm(nearest-position[:2],axis=1).argmin())
    target=path[index]+fraction[index]*(path[index+1]-path[index])
    remaining=lookahead
    for end in path[index+1:]:
        distance=np.linalg.norm(end[:2]-target[:2])
        if distance>=remaining and distance>1e-9:return target+(end-target)*remaining/distance,index
        remaining-=distance;target=end
    return path[-1].copy(),len(path)-1
