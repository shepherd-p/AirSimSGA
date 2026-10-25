"""Closed plaza mission using measured poses and physical vehicle controls."""
import asyncio
import time
import numpy as np
from .experiments import Experiment
from .scan_capture import ScanRecorder


class PlazaExperiment(Experiment):
    async def execute(self):
        status, error, recorder = 'failed', None, None
        try:
            self.b.connect()
            self.run.json('previous_scene_ids.json',self.b.previous_scene_ids)
            self.b.load()
            self.run.json('loaded_scene.json',self.b.world.get_configuration())
            await self.b.ready()
            await self.b.settle()
            for n,sid in self.c['dense_capture']['robot_segmentation_ids'].items():
                if not self.b.world.set_segmentation_id_by_name(n,sid,False,True):
                    raise RuntimeError('Cannot assign robot mask: '+n)
            self.initial={n:self.b.pose(n)['position'] for n in self.b.kinds}
            self.run.json('initial_poses.json',self.initial)
            self.configure_routes()
            recorder=ScanRecorder(self.run.path)
            self.b.buffer.sink=recorder.receive
            self.start_wall=time.monotonic()
            self.start_sim=self.b.now()
            self._find_process()
            # Sampling during takeoff also retains the actual beginning of both tracks.
            takeoff=asyncio.create_task(self.b.start())
            try:
                while not takeoff.done():
                    self.record({n:self.b.pose(n) for n in self.b.kinds})
                    await asyncio.sleep(.2)
                await takeoff
            finally:
                if not takeoff.done():
                    takeoff.cancel()
                    await asyncio.gather(takeoff,return_exceptions=True)
            status=await self.orbit()
        except asyncio.CancelledError:
            status,error='interrupted','Cancelled by user'
            raise
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
            print(error,flush=True)
        finally:
            cleanup_errors=await self.b.cleanup()
            self.b.buffer.sink=None
            if recorder:
                try:
                    recorder.close()
                except Exception as exc:
                    status,error='incomplete_capture',str(exc)
            for w in self.video_writers.values(): w.release()
            self.run.finish(status,error=error,cleanup_errors=cleanup_errors,
                            collision_count=self.collision_count,path_lengths_m=self.path_lengths,
                            horizontal_path_lengths_m=self.horizontal_lengths,
                            route_indices=self.indices,rgbd_frame_counts=self.dense_recorder.counts,
                            sensor_message_counts=dict(self.b.buffer.counts))
        return self.run.path,status

    async def orbit(self):
        deadline=time.monotonic()+self.c['duration_sec']*4+30
        last_log=-10
        while self.b.now()-self.start_sim < self.c['duration_sec']:
            tick=time.monotonic()
            if tick>deadline: raise TimeoutError('Plaza loop wall time watchdog')
            poses={n:self.b.pose(n) for n in self.b.kinds}
            self.record(poses)
            self.safety(poses)
            goals={n:self.route_goal(n,p['position']) for n,p in poses.items()}
            for n,g in goals.items():
                self.run.row('controls',{'time':poses[n]['time'],'name':n,'route_index':self.indices[n],
                                        'goal_x':None if g is None else g[0],
                                        'goal_y':None if g is None else g[1],
                                        'goal_z':None if g is None else g[2]})
            await asyncio.gather(*(self.go(n,poses[n],g) for n,g in goals.items()))
            elapsed=self.b.now()-self.start_sim
            if elapsed-last_log>5:
                last_log=elapsed
                print(f'{elapsed:.1f}s routes {self.indices} positions '+str({n:np.round(p['position'],1).tolist() for n,p in poses.items()}),flush=True)
                self.performance(self.b.now())
            if all(g is None for g in goals.values()):
                self.run.json('loop_completion.json',{'all_routes_completed':True,
                    'closure_errors_m':{n:float(np.linalg.norm(poses[n]['position'][:2]-self.routes[n][-1,:2])) for n in poses}})
                return 'completed'
            await asyncio.sleep(max(0,1/self.c['control_hz']-(time.monotonic()-tick)))
        return 'incomplete_loop_timeout'
