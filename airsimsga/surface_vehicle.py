"""Small, tested adapter for the USV service used by the landing experiment."""
import math


class SurfaceVehicle:
    def __init__(self, client, world, robot_id='Vessel1', thruster_count=3):
        if not robot_id or '/' in robot_id or thruster_count < 1:
            raise ValueError('Provide a robot ID without slash and positive thruster_count')
        self.client, self.world = client, world
        self.robot_id, self.thruster_count = robot_id, thruster_count

    def set_thruster_forces(self, forces):
        values = [float(value) for value in forces]
        if len(values) != self.thruster_count or not all(math.isfinite(value) for value in values):
            raise ValueError('Finite forces matching thruster_count are required')
        return self.client.request({
            'method': self.world.parent_topic + '/robots/' + self.robot_id + '/SetThrusterForces',
            'params': {'forces': values}, 'version': 1.0})

    def stop(self):
        return self.set_thruster_forces([0.] * self.thruster_count)

    def get_pose(self):
        return self.world.get_object_pose(self.robot_id)

    def set_thruster_azimuths(self, azimuths):
        raise NotImplementedError('Azimuth dynamics are not implemented by the released USV bridge')
