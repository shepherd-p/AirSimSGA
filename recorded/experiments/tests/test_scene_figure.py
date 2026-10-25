import copy
import unittest
from ag.scene_figure import select_capture


class SceneFigureTests(unittest.TestCase):
    def frame(self):
        return {'image':'both.png','capture_metadata':{'width':2560,'height':1440},
                'visible_robots':{'Drone1':{'pixels':500,'bbox':[1200,300,1250,325]},
                                 'Car1':{'pixels':4000,'bbox':[1000,900,1200,1100]}}}

    def test_rejects_missing_or_clipped_robot(self):
        absent=self.frame();del absent['visible_robots']['Car1']
        clipped=self.frame();clipped['visible_robots']['Drone1']['bbox'][1]=0
        with self.assertRaises(ValueError):select_capture([absent,clipped])

    def test_selects_larger_drone_only_among_complete_views(self):
        first=self.frame();better=copy.deepcopy(first);better['image']='closer.png'
        better['visible_robots']['Drone1']['pixels']=1800
        invalid=copy.deepcopy(better);invalid['visible_robots']['Car1']['pixels']=20
        invalid['visible_robots']['Drone1']['pixels']=5000
        self.assertEqual(select_capture([first,better,invalid])['image'],'closer.png')


if __name__=='__main__':unittest.main()
