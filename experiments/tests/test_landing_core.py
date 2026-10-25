import unittest
import numpy as np
from ag.landing_core import ContactGate, VisualTargetFilter, deck_position, deck_relative, rotor_measurement


class LandingTests(unittest.TestCase):
    def gate(self,moving=True):
        return ContactGate({'collision_max_age_sec':.3,'max_sample_gap_sec':.4,
           'contact_horizontal_radius_m':.5,'contact_height_range_m':[-.05,.65],
           'contact_relative_speed_mps':.2,'min_moving_speed_mps':.15,'hold_sec':1.},moving)

    def feed(self,gate,**kw):
        out=(False,0.)
        for now in np.arange(0,1.21,.1):
            args=dict(now=now,horizontal=.1,height=.2,relative_speed=.01,boat_speed=.5,
                      contact_time=now,vessel_contact=True,motors_off=True)
            args.update(kw);out=gate.update(**args)
        return out

    def test_moving_contact_dwell(self):self.assertTrue(self.feed(self.gate())[0])
    def test_stopped_boat_is_not_moving_landing(self):self.assertFalse(self.feed(self.gate(),boat_speed=0.)[0])
    def test_hover_is_not_touchdown(self):self.assertFalse(self.feed(self.gate(),motors_off=False)[0])
    def test_missing_contact(self):self.assertFalse(self.feed(self.gate(),vessel_contact=False)[0])
    def test_stale_collision_is_not_support(self):self.assertFalse(self.feed(self.gate(),contact_time=-5.)[0])
    def test_sliding_is_not_stable(self):self.assertFalse(self.feed(self.gate(),relative_speed=.6)[0])
    def test_static_baseline_can_pass(self):self.assertTrue(self.feed(self.gate(False),boat_speed=0.)[0])
    def test_explicit_support_loss_invalidates_stable_pose(self):
        self.assertFalse(self.feed(self.gate(),continuous_support=False)[0])
    def test_continuous_support_validates_moving_dwell(self):
        self.assertTrue(self.feed(self.gate(),continuous_support=True)[0])

    def test_accepted_disarm_with_powered_rotors_fails(self):
        self.assertFalse(self.feed(self.gate(),rotors_stopped=False)[0])

    def test_stopped_rotor_measurements(self):
        message={'time_stamp':1_000_000_000,'rotor_info_vec':[
            {'rotor_id':str(i),'speed':0.,'thrust':0.} for i in range(4)]}
        self.assertTrue(rotor_measurement(message,1.1,{})['stopped'])
        self.assertFalse(rotor_measurement(message,2.,{})['stopped'])
        message['rotor_info_vec'][0]['thrust']=2.4
        self.assertFalse(rotor_measurement(message,1.1,{})['stopped'])
        message['rotor_info_vec']=message['rotor_info_vec'][:3]
        self.assertFalse(rotor_measurement(message,1.1,{})['stopped'])

    def test_full_rotation_offset(self):
        q=[np.sqrt(.5),0,0,np.sqrt(.5)]
        np.testing.assert_allclose(deck_position([3,4,5],q,[2,0,-1]),[3,6,4],atol=1e-8)
        np.testing.assert_allclose(deck_relative([3,6,3],[3,4,5],q,[2,0,-1]),[0,0,-1],atol=1e-8)

    def test_filter_rejects_duplicate_and_outlier(self):
        f=VisualTargetFilter(1.)
        self.assertTrue(f.update(1.,[0,0,0]))
        self.assertFalse(f.update(1.,[50,0,0]))
        self.assertTrue(f.update(1.1,[.05,0,0]))
        self.assertFalse(f.update(1.2,[50,0,0]))
        np.testing.assert_allclose(f.predict(1.3),[.15,0,0],atol=1e-8)

    def test_sampling_gap_resets_dwell(self):
        g=self.gate()
        g.update(0,.1,.2,0,.5,0,True,True)
        self.assertEqual(g.update(.8,.1,.2,0,.5,.8,True,True),(False,0.))

    def test_event_only_contact_can_be_followed_by_stable_dwell(self):
        g=self.gate(False)
        for now in np.arange(0,1.21,.1):
            result=g.update(now,.1,.2,0,0,0,True,True)
        self.assertTrue(result[0])


if __name__=='__main__':unittest.main()
