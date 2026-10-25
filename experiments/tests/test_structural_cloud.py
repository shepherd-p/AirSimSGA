import unittest
import numpy as np
from render_structural_cloud import structural_display


class StructuralCloudTests(unittest.TestCase):
    def test_filters_isolated_noise_without_moving_or_synthesizing_points(self):
        x,y=np.meshgrid(np.arange(-1,1.001,.08),np.arange(-1,1.001,.08))
        floor=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        y,z=np.meshgrid(np.arange(-1,1.001,.08),np.arange(.6,2.001,.08))
        wall=np.c_[np.full(y.size,1.5),y.ravel(),z.ravel()]
        points=np.vstack([floor,floor+[.02,.02,.003],wall,[[30,30,30]]])
        source=np.r_[np.zeros(len(floor)),np.ones(len(floor)),np.zeros(len(wall)+1)].astype('u1')
        shown,labels,normals,indices,report=structural_display(points,source)
        np.testing.assert_array_equal(shown,points[indices])
        np.testing.assert_array_equal(labels,source[indices])
        self.assertNotIn(len(points)-1,indices)
        self.assertTrue(np.all(np.isfinite(normals)))
        self.assertGreater(report['flat_points'],len(floor))
        self.assertLess(report['flat_points_shown'],report['flat_points']//2)
        interior=np.flatnonzero((points[:,0]==1.5)&(abs(points[:,1])<.7)&
                               (points[:,2]>.9)&(points[:,2]<1.8))
        self.assertGreater(np.isin(interior,indices).mean(),.95)
        self.assertEqual(set(labels.tolist()),{0,1})


if __name__=='__main__':unittest.main()
