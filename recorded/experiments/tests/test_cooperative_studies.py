import json
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
from ag.study_planning import ObservedGroundPlanner,trailing_point,path_lookahead
from ag.cooperative_study import CooperativeStudy
from ag.study_audit import decode,segmentation_boxes,member


class ObservedPlanningTests(unittest.TestCase):
    def grid(self,points,radius=0):
        options={'resolution_m':1.,'roi_xy':[[0,0],[8,8]],'floor_tolerance_m':.2,
                 'obstacle_height_range_m':[.4,2.8],'minimum_cell_points':1,'vehicle_radius_m':radius}
        return ObservedGroundPlanner(points,[.5,.5,0],options,0)

    def test_unknown_gap_cannot_be_crossed(self):
        points=[[x+.5,y+.5,0] for x in [0,1,2,4,5,6] for y in range(7)]
        grid=self.grid(points)
        with self.assertRaises(ValueError):grid.route([.5,3.5,0],[6.5,3.5,0])

    def test_obstacle_inflation_and_detour(self):
        points=[[x+.5,y+.5,0] for x in range(9) for y in range(9)]+[[4.5,4.5,-1]]
        grid=self.grid(points,radius=1.1)
        self.assertFalse(grid.free[4,4]);self.assertFalse(grid.free[5,4])
        route=grid.route([.5,4.5,0],[8.5,4.5,0])
        self.assertGreater(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1).sum(),8)
        for a,b in zip(route[:-1],route[1:]):self.assertTrue(grid.visible(grid.cell(a),grid.cell(b)))

    def test_no_diagonal_path_through_blocked_corner(self):
        grid=self.grid([[.5,.5,0],[1.5,1.5,0]])
        with self.assertRaises(ValueError):grid.route([.5,.5,0],[1.5,1.5,0])

    def test_trailing_point_follows_observed_corner(self):
        points=[[0,0,0],[5,0,0],[5,4,0]]
        np.testing.assert_allclose(trailing_point(points,6),[3,0,0])
        np.testing.assert_allclose(trailing_point(points,30),[0,0,0])
        self.assertIsNone(trailing_point([],1))

    def test_lookahead_crosses_segment_and_stops_at_end(self):
        path=np.array([[0,0,0],[5,0,0],[5,5,0]])
        np.testing.assert_allclose(path_lookahead(path,[4,1,0],2)[0],[5,1,0])
        np.testing.assert_allclose(path_lookahead(path,[5,4.5,0],2)[0],[5,5,0])


class CompletionTests(unittest.TestCase):
    def test_repeated_air_lap_must_finish_after_cars(self):
        study=CooperativeStudy.__new__(CooperativeStudy)
        study.b=SimpleNamespace(kinds={'Drone1':'uav','Car1':'ugv'})
        study.phase_change=lambda *args:None;study.routes={'Drone1':[[0,0,0],[1,0,0]]}
        study.indices={};study.laps={'Drone1':1};study.route_completion_marked=set()
        study.route_goal=lambda name,pos:np.array([1,0,0]) if name=='Drone1' else None
        poses={name:{'position':[0,0,0]} for name in study.b.kinds}
        _,finished=study.formation_goals(1,poses);self.assertFalse(finished)
        study.route_goal=lambda name,pos:None
        _,finished=study.formation_goals(2,poses);self.assertTrue(finished)
        self.assertEqual(study.laps,{'Drone1':2,'Car1':1})
        study.formation_goals(3,poses);self.assertEqual(study.laps['Drone1'],2)


class EvidenceTests(unittest.TestCase):
    def test_raw_bgr_and_depth_roundtrip(self):
        depth=np.array([[120,5400],[65535,0]],dtype='<u2')
        np.testing.assert_array_equal(decode(depth.view('u1').reshape(-1),{'height':2,'width':2,'encoding':'16UC1'}),depth)
        rgb=np.array([[[7,8,9],[1,2,3]],[[1,2,3],[7,8,9]]],dtype='u1')
        image=decode(rgb.reshape(-1),{'height':2,'width':2,'encoding':'BGR'})
        self.assertEqual(segmentation_boxes(image,{'Target':[7,8,9]}),{'Target':{'pixels':2,'bbox':[0,0,1,1]}})

    def test_index_cannot_escape_capture_directory(self):
        root=Path(__file__).resolve().parents[1]/'verification'
        with self.assertRaises(ValueError):member(root,'../run_study.ps1')


if __name__=='__main__':unittest.main()
