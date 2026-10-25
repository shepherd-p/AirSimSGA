import unittest
import numpy as np
import open3d as o3d
from ag.figure_render import camera,point_image,mesh_image


class FigureRenderTests(unittest.TestCase):
    def test_single_pixel_cloud_does_not_dilate_measurements(self):
        center,*_=camera(101,101)
        points=np.array([center]);colors=np.array([[0,0,1]])
        thin=point_image(points,colors,101,101,splat=False)
        original=point_image(points,colors,101,101)
        self.assertEqual(np.any(thin<255,axis=2).sum(),1)
        self.assertEqual(np.any(original<255,axis=2).sum(),5)

    def test_points_keep_nearest_surface_independent_of_order(self):
        center,back,right,up,scale=camera(101,101)
        p=np.array([center,center+back])
        colors=np.array([[1,0,0],[0,1,0]])
        a=point_image(p,colors,101,101);b=point_image(p[::-1],colors[::-1],101,101)
        np.testing.assert_array_equal(a,b)
        np.testing.assert_array_equal(a[50,50],[0,255,0])

    def test_mesh_interpolates_vertex_color_instead_of_flat_mean(self):
        center,back,right,up,scale=camera(101,101)
        vertices=np.array([center-10*right-10*up,center+10*right-10*up,center+10*up])
        mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),o3d.utility.Vector3iVector([[0,1,2]]))
        mesh.vertex_colors=o3d.utility.Vector3dVector([[1,0,0],[0,1,0],[0,0,1]])
        im,report=mesh_image(mesh,101,101)
        self.assertEqual(report['rendered_triangles'],1)
        self.assertGreater(int(im[50,50,2]),int(im[50,50,0]))
        self.assertGreater(int(im[53,44,0]),int(im[53,56,0]))


if __name__=='__main__':unittest.main()
