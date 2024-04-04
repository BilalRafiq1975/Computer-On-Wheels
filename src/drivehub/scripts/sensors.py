import rospy
from carla_msgs.srv import SpawnObject
from geometry_msgs.msg import Pose, Point, Quaternion


class SpawnSensor:
    def __init__(self, attach_id, *args):
        self.spawn_object_client(attach_id, *args)

    @classmethod
    def spawn_object_client(cls, attach_id, *sensors):
        global response
        rospy.wait_for_service('/carla/spawn_object')
        try:
            spawn_object = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
            key_value = []
            point = Point(0, 0, 0)
            quaternion = Quaternion(0, 0, 0, 0)
            pose = Pose(point, quaternion)
            random_pos = False
            attach = attach_id

            for sensor in sensors:
                if sensor == 'camera':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.camera.rgb', 'rgb_camera_sensor', key_value, pose, attach,
                                            random_pos)
                elif sensor == 'lidar':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.lidar.ray_cast', 'lidar_sensor', key_value, pose, attach,
                                            random_pos)
                elif sensor == 'lidar_semantic':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.lidar.ray_cast_semantic', 'lidar_sensor', key_value, pose, attach,
                                            random_pos)
                elif sensor == 'odometer':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.pseudo.odom', 'odometer_sensor', key_value, pose, attach,
                                            random_pos)
                elif sensor == 'speedometer':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.pseudo.speedometer', 'speedometer_sensor', key_value, pose, attach,
                                            random_pos)
                elif sensor == 'control':
                    point = Point(0, 0, 0)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('actor.pseudo.control', 'control', key_value, pose, attach, random_pos)
                elif sensor == 'radar':
                    point = Point(0, 0, 2.4)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.other.radar', 'radar_sensor', key_value, pose, attach, random_pos)
                elif sensor == 'gnss':
                    point = Point(2, 0, 0)
                    quaternion = Quaternion(0, 0, 0, 0)
                    pose = Pose(point, quaternion)
                    response = spawn_object('sensor.other.gnss', 'gnss_sensor', key_value, pose, attach, random_pos)
                elif sensor == 'imu':
                    response = spawn_object('sensor.other.imu', 'imu_sensor', key_value, pose, attach, random_pos)
                elif sensor == 'object_sensor':
                    response = spawn_object('sensor.pseudo.objects', 'objects', key_value, pose, attach, random_pos)
                elif sensor == 'actor_list_sensor':
                    response = spawn_object('sensor.pseudo.actor_list', 'actor_list', key_value, pose, attach,
                                            random_pos)

            return response

        except rospy.ServiceException as e:
            return e


if __name__ == '__main__':
    rospy.init_node("AV_Drive")
    spawn_sensor = SpawnSensor(24, "lidar")
