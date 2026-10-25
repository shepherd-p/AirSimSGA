import unittest
import numpy as np
from ag.math3d import lidar_world, depth_world, icp
from ag.planning import SurfaceMap, Tracker, rover_command, clear_ego_footprint
from ag.backend import Buffer


class Algorithms(unittest.TestCase):
    def test_lidar_capture_extrinsic(self):
        s = np.sqrt(.5)
        msg={'point_cloud':[1,0,0,0,0,0], 'pose':{'position':{'x':10,'y':20,'z':-3},
             'orientation':{'w':s,'x':0,'y':0,'z':s}}}
        np.testing.assert_allclose(lidar_world(msg),[[10,21,-3]],atol=1e-8)

    def test_depth_perspective_range(self):
        msg = dict(pos_x=0,pos_y=0,pos_z=0,rot_w=1,rot_x=0,rot_y=0,rot_z=0)
        p=depth_world(np.full((4,4),10.),msg,stride=1)
        np.testing.assert_allclose(np.linalg.norm(p,axis=1),10)
        np.testing.assert_allclose(p[10],[10,0,0])

    def test_depth_pixel_axis_convention_is_opt_in(self):
        msg = dict(pos_x=0,pos_y=0,pos_z=0,rot_w=1,rot_x=0,rot_y=0,rot_z=0)
        depth = np.full((4,4),10.)
        original = depth_world(depth,msg,stride=1)
        mirrored = depth_world(depth,msg,stride=1,pixel_signs=(-1,-1))
        np.testing.assert_allclose(original[:,0],mirrored[:,0])
        np.testing.assert_allclose(original[:,1:],-mirrored[:,1:])

    def test_icp_recovers_known_rigid_translation(self):
        a=np.random.default_rng(4).uniform(-3,3,(300,3))
        b=a+[.12,-.08,.07]
        result,t=icp(a,b,gate=.8)
        self.assertEqual(result['status'],'ok')
        np.testing.assert_allclose(t[:3,3],[.12,-.08,.07],atol=.03)

    def test_icp_rejects_nonoverlap(self):
        a=np.random.default_rng(4).normal(size=(30,3))
        result,_=icp(a,a+100,gate=1)
        self.assertEqual(result['status'],'no_correspondences')

    def test_layers_do_not_teleport_between_bridge_and_ground(self):
        points=np.array([[x+.1,y+.1,z] for x in range(5) for y in range(3) for z in (0,-5)])
        grid=SurfaceMap(points,resolution=1,min_points=1,radius=0)
        route=grid.route([.1,.1,0],[4.1,2.1,0])
        np.testing.assert_allclose(route[:,2],0)
        with self.assertRaises(ValueError):
            grid.route([.1,.1,0],[4.1,2.1,-5])

    def test_unknown_gap_is_not_free(self):
        points=np.array([[x+.1,y+.1,0] for x in (0,1,3,4) for y in range(3)])
        grid=SurfaceMap(points,resolution=1,min_points=1,radius=0)
        with self.assertRaises(ValueError):
            grid.route([.1,.1,0],[4.1,2.1,0])

    def test_tracker_expiry_and_prediction(self):
        tracker=Tracker(max_age=.2,prediction_horizon=1)
        p,used=tracker.update(1,[{'time':1,'position':[2,3,4],'confidence':1}])
        self.assertTrue(used)
        p,used=tracker.update(1.5,[{'time':1,'position':[100,100,100],'confidence':1}])
        self.assertFalse(used)
        np.testing.assert_allclose(p,[2,3,4])
        self.assertIsNone(tracker.update(2.1,[])[0])

    def test_buffer_capture_time_pairing(self):
        buf=Buffer()
        buf.callback('d')('',{'time_stamp':1_000_000_000})
        buf.callback('s')('',{'time_stamp':1_300_000_000})
        self.assertIsNone(buf.pair('d','s',1,.1))
        buf.callback('s')('',{'time_stamp':1_050_000_000})
        self.assertIsNotNone(buf.pair('d','s',1,.1))

    def test_rover_stops_at_goal_and_brakes_overspeed(self):
        self.assertEqual(rover_command([0,0],0,[0,0],2),(0.,0.,1.))
        self.assertGreater(rover_command([0,0],0,[10,0],5)[2],0)

    def test_ego_prior_does_not_remove_external_obstacle(self):
        p=np.array([[0,0,-1.],[4,0,-1.],[10,0,0]])
        cloud,prior=clear_ego_footprint(p,[0,0,0],0,0)
        self.assertFalse(np.any(np.all(cloud==[0,0,-1.],axis=1)))
        self.assertTrue(np.any(np.all(cloud==[4,0,-1.],axis=1)))
        self.assertLessEqual(abs(prior[:,0]).max(),2.9)

    def test_rover_can_approach_navigation_tolerance(self):
        engine,_,brake=rover_command([0,0],0,[.65,0],0)
        self.assertGreater(engine,0)
        self.assertEqual(brake,0)


if __name__ == '__main__':
    unittest.main()
