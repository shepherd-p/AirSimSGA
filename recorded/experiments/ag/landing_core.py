"""Pure measurement/control/evaluation logic, independent of the simulator."""
from collections import deque
import math
import numpy as np
from .math3d import rotation


def deck_position(position, quaternion, offset):
    return np.asarray(position, float) + rotation(quaternion) @ np.asarray(offset, float)


def deck_relative(uav_position, usv_position, usv_quaternion, offset):
    return rotation(usv_quaternion).T @ (np.asarray(uav_position)-deck_position(usv_position, usv_quaternion, offset))


def rotor_measurement(message, now, config):
    """An accepted Disarm request is not evidence of stopped physical rotors."""
    result={'time_ns': None, 'max_speed_radps': None, 'total_thrust_n': None, 'stopped': False}
    if not message:return result
    result['time_ns']=int(message.get('time_stamp',0))
    rotors=message.get('rotor_info_vec',[])
    if len(rotors)!=config.get('expected_rotor_count',4):return result
    if len({r.get('rotor_id') for r in rotors})!=len(rotors):return result
    speed=np.array([r.get('speed',np.nan) for r in rotors],float)
    thrust=np.array([r.get('thrust',np.nan) for r in rotors],float)
    if not np.isfinite(speed).all() or not np.isfinite(thrust).all():return result
    result['max_speed_radps']=float(np.max(abs(speed)))
    result['total_thrust_n']=float(np.sum(abs(thrust)))
    age=now-result['time_ns']/1e9
    result['stopped']=bool(0<=age<=config.get('rotor_max_age_sec',.3) and
        result['max_speed_radps']<=config.get('stopped_rotor_max_speed_radps',1.) and
        result['total_thrust_n']<=config.get('stopped_rotor_total_thrust_n',.01))
    return result


class VisualTargetFilter:
    def __init__(self, alpha=.25):
        self.alpha = alpha
        self.time = None
        self.position = None
        self.velocity = np.zeros(3)

    def update(self, timestamp, position):
        p = np.asarray(position, float)
        if not np.isfinite(p).all() or (self.time is not None and timestamp <= self.time):
            return False
        if self.time is not None:
            dt = timestamp-self.time
            if dt > 1.:
                self.velocity[:] = 0.
            else:
                velocity = (p-self.position)/dt
                if np.linalg.norm(velocity) > 6.:
                    return False
                self.velocity += self.alpha*(velocity-self.velocity)
        self.time, self.position = timestamp, p
        return True

    def predict(self, timestamp, horizon=0.):
        return self.position + self.velocity*max(0., timestamp-self.time+horizon)


class ContactGate:
    """A stale impact or a stable hover never satisfies motor-off deck dwell."""
    def __init__(self, config, moving):
        self.c = config
        self.moving = moving
        self.samples = deque()
        self.last_time = None
        self.last_contact = None
        self.contact_latched = False

    def update(self, now, horizontal, height, relative_speed, boat_speed,
               contact_time, vessel_contact, motors_off, continuous_support=None, rotors_stopped=None):
        fresh = vessel_contact and contact_time is not None and 0 <= now-contact_time <= self.c['collision_max_age_sec']
        if fresh:
            self.last_contact = now
        gap = self.last_time is not None and (now <= self.last_time or now-self.last_time > self.c['max_sample_gap_sec'])
        self.last_time = now
        if gap:self.contact_latched=False
        if fresh and motors_off:self.contact_latched=True
        good = (motors_off and self.contact_latched and not gap and continuous_support is not False and rotors_stopped is not False and
                horizontal <= self.c['contact_horizontal_radius_m'] and
                self.c['contact_height_range_m'][0] <= height <= self.c['contact_height_range_m'][1] and
                relative_speed <= self.c['contact_relative_speed_mps'] and
                (not self.moving or boat_speed >= self.c['min_moving_speed_mps']))
        if not good:
            self.samples.clear()
            self.contact_latched=False
            return False, 0.
        self.samples.append(now)
        dwell = now-self.samples[0]
        return dwell >= self.c['hold_sec'], dwell


def control_velocity(c, ego, target_filter, now, controller='predictive'):
    horizon = c['prediction_sec'] if controller == 'predictive' else 0.
    target = target_filter.predict(now, horizon)
    error = target-np.asarray(ego['position'])
    ff = c['feedforward'] if controller == 'predictive' else 0.
    v = c['kp']*error + ff*target_filter.velocity
    horizontal = np.linalg.norm(v[:2])
    if horizontal > c['max_speed_mps']:
        v[:2] *= c['max_speed_mps']/horizontal
    relspeed = np.linalg.norm(np.asarray(ego['velocity'])[:2]-target_filter.velocity[:2])
    centered = np.linalg.norm(error[:2]) <= c['descent_error_m'] and relspeed <= c['descent_relative_speed_mps']
    height = error[2]
    v[2] = (c['flare_rate_mps'] if height < c['flare_height_m'] else c['descend_rate_mps']) if centered else 0.
    return v, ('flare' if height < c['flare_height_m'] else 'descend') if centered else 'align'
