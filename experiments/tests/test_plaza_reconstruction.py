import unittest
import numpy as np
import open3d as o3d
from ag.reconstruction import constrain_surface,cloud
from ag.plaza_reconstruction import reduce_voxels
from prepare_plaza_configs import rounded_loop


class PlazaTests(unittest.TestCase):
    def test_surface_rejects_unobserved_sheet_without_filling(self):
        vertices=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,10],[1,0,10],[0,1,10]],dtype=float)
        mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                       o3d.utility.Vector3iVector([[0,1,2],[3,4,5]]))
        result=constrain_surface(mesh,cloud(vertices[:3]))
        self.assertEqual(result['rejected_triangles'],1)
        self.assertEqual(len(mesh.triangles),1)
        np.testing.assert_allclose(np.asarray(mesh.vertices)[:,2],0)

    def test_weighted_batches_preserve_centroid_and_time(self):
        p=np.array([[.01,.02,.03],[.07,.08,.09],[1.01,0,0]])
        a,w,t=reduce_voxels(p[:1],np.ones(1),np.array([5.]),.1)
        a,w,t=reduce_voxels(np.vstack([a,p[1:]]),np.r_[w,[1,1]],np.r_[t,[3,7]],.1)
        np.testing.assert_allclose(a,[[.04,.05,.06],[1.01,0,0]])
        np.testing.assert_allclose(w,[2,1]);np.testing.assert_allclose(t,[3,7])

    def test_loop_closed_and_corners_clear(self):
        p=np.array(rounded_loop(21,48,50.5,63,4,4.48))
        np.testing.assert_allclose(p[0],p[-1])
        self.assertTrue(np.all(p[:,0]>=21));self.assertTrue(np.all(p[:,0]<=48))
        self.assertTrue(np.all(p[:,1]>=50.5));self.assertTrue(np.all(p[:,1]<=63))


if __name__=='__main__':unittest.main()
