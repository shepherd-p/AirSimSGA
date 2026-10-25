import csv
from pathlib import Path
import tempfile
import unittest
import numpy as np
from ag.scan_capture import ScanRecorder


class ArchiveTests(unittest.TestCase):
    def packet(self,i):
        return {'time_stamp':i,'point_cloud':[1.,0,0,0,0,0,2,0,0],
                'segmentation_cloud':[3,3,240],
                'pose':{'position':{'x':10,'y':20,'z':30},'orientation':{'w':1,'x':0,'y':0,'z':0}}}

    def test_drain_filter_pose_and_duplicate(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp);r=ScanRecorder(path)
            for i in range(40):r.receive('Drone1/lidar',self.packet(i))
            r.receive('Drone1/lidar',self.packet(0))
            result=r.close()
            self.assertEqual(result['saved'],40)
            self.assertEqual(result['duplicates'],1)
            self.assertEqual(result['queue_drops'],0)
            with np.load(path/'raw/Drone1/0.npz') as d:np.testing.assert_allclose(d['world_points'],[[11,20,30]])
            r.receive('Drone1/lidar',self.packet(99))
            self.assertFalse((path/'raw/Drone1/99.npz').exists())

    def test_worker_failure_is_reported(self):
        with tempfile.TemporaryDirectory() as temp:
            r=ScanRecorder(Path(temp));r.receive('Car1/lidar',{'time_stamp':0})
            with self.assertRaises(RuntimeError):r.close()


if __name__=='__main__':unittest.main()
