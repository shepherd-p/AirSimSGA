import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
from airsimsga import SurfaceVehicle
from ag.portable import resolve_config, zonegraph_path, sdk_path
from tools.materialize_backend import materialize
ROOT=Path(__file__).resolve().parents[1]

class SurfaceVehicleTests(unittest.TestCase):
    def setUp(self):
        self.client=Mock()
        self.world=Mock(parent_topic='/Sim/Example')
        self.vessel=SurfaceVehicle(self.client,self.world,'Vessel1')

    def test_routes_forces_to_selected_robot_service(self):
        self.vessel.set_thruster_forces([0,-125,-125])
        self.client.request.assert_called_once_with({'method':'/Sim/Example/robots/Vessel1/SetThrusterForces','params':{'forces':[0.,-125.,-125.]},'version':1.0})

    def test_invalid_force_never_sends_a_command(self):
        for forces in [[1,2],[0,float('nan'),0],[0,float('inf'),0]]:
            with self.assertRaises(ValueError):self.vessel.set_thruster_forces(forces)
        self.client.request.assert_not_called()

    def test_azimuth_is_explicitly_unsupported(self):
        with self.assertRaises(NotImplementedError):self.vessel.set_thruster_azimuths([0,0,0])
        self.client.request.assert_not_called()

    def test_stop_sends_zero_to_each_thruster(self):
        self.vessel.stop()
        self.assertEqual(self.client.request.call_args[0][0]['params']['forces'],[0.,0.,0.])

class PortabilityTests(unittest.TestCase):
    def test_missing_sdk_does_not_fall_back_to_author_machine(self):
        with patch.dict('os.environ',{},clear=True):
            with self.assertRaises(ValueError):sdk_path()

    def test_city_project_path_resolves_zonegraph(self):
        with patch.dict('os.environ',{'AIRSIMSGA_CITYSAMPLE_PROJECT':str(ROOT/'QA/local-scene')},clear=True):
            self.assertEqual(zonegraph_path(),ROOT/'QA/local-scene/Saved/AGCitySampleZoneGraph.json')

    def test_landing_requires_receipt_placeholder(self):
        config=json.loads((ROOT/'experiments/configs/landing/study.json').read_text(encoding='utf-8'))
        with patch.dict('os.environ',{'AIRSIMSGA_SDK_PATH':'irrelevant','AIRSIMSGA_LANDING_BACKEND':'irrelevant'},clear=True):
            with self.assertRaisesRegex(ValueError,'AIRSIMSGA_LANDING_BUILD_RECEIPT'):resolve_config(config)

    def test_backend_assembly_refuses_existing_directory(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'QA') as folder:
            with self.assertRaises(FileExistsError):materialize(ROOT/'missing-upstream',Path(folder),'main')

    def test_scene_robot_references_are_present(self):
        cases=[('configs','scene_mapping_plaza.jsonc'),('configs/landing','scene_landing.jsonc'),('citysample_scene','generated/scene_navigation_aerial_route.jsonc'),('citysample_scene','generated/scene_pursuit.jsonc'),('citysample_scene','generated/scene_pursuit_views.jsonc')]
        for directory,filename in cases:
            base=ROOT/'experiments'/directory
            data=json.loads((base/filename).read_text(encoding='utf-8'))
            names=[actor['name'] for actor in data['actors']]
            self.assertEqual(len(names),len(set(names)))
            for actor in data['actors']:
                self.assertTrue((base/actor['robot-config']).is_file(),actor['robot-config'])

if __name__=='__main__':unittest.main()
