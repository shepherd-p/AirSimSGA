import importlib.util
import unittest
import numpy as np


@unittest.skipUnless(importlib.util.find_spec('open3d'), 'Run with isolated reconstruction Python')
class ReconstructionGeometry(unittest.TestCase):
    def test_range_to_axial_and_camera_extrinsic(self):
        from ag.reconstruction import rgbd_geometry
        # Constant forward-plane X=3 in FRD, all rays have different ranges.
        h,w=48,64
        v,u=np.mgrid[:h,:w];f=w/2
        depth=3*np.sqrt(1+((u-w/2)/f)**2+((v-h/2)/f)**2)
        position=np.array([10.,20.,-5.])
        z,k,e=rgbd_geometry(depth,90,position,np.eye(3),20)
        np.testing.assert_allclose(z,3,atol=1e-6)
        np.testing.assert_allclose(np.linalg.inv(e)@np.array([0.,0.,3.,1.]),[13,20,-5,1])
        np.testing.assert_allclose(np.linalg.inv(e)@np.array([1.,2.,3.,1.]),[13,21,-3,1])

    def test_tsdf_recovers_observed_plane_without_filling_missing_half(self):
        import open3d as o3d
        from ag.reconstruction import rgbd_geometry
        h,w=48,64;v,u=np.mgrid[:h,:w];f=w/2
        depth=(3*np.sqrt(1+((u-w/2)/f)**2+((v-h/2)/f)**2)).astype(np.float32)
        depth[:, :w//2]=0  # No data in the negative-right half.
        z,k,e=rgbd_geometry(depth,90,np.zeros(3),np.eye(3),10)
        rgb=np.zeros((h,w,3),dtype=np.uint8);rgb[:]=[200,100,50]
        frame=o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb),
              o3d.geometry.Image(z),depth_scale=1,depth_trunc=10,convert_rgb_to_intensity=False)
        vol=o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=.05,sdf_trunc=.15,
                         color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        for _ in range(3):vol.integrate(frame,k,e)
        mesh=vol.extract_triangle_mesh();p=np.asarray(mesh.vertices)
        self.assertGreater(len(p),100)
        self.assertLess(np.percentile(np.abs(p[:,0]-3),95),.025)
        self.assertGreater(p[:,1].min(),-.1)
        np.testing.assert_allclose(np.asarray(mesh.vertex_colors).mean(axis=0),[200/255,100/255,50/255],atol=.01)


if __name__=='__main__':
    unittest.main()
