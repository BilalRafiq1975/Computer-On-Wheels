import rospy
from carla_msgs.srv import SpawnObject, SpawnObjectRequest
from geometry_msgs.msg import Transform, Vector3, Quaternion, Pose


def spawn_vehicle(actor_type, spawn_point, spawn_rotation):
    rospy.wait_for_service('/carla/spawn_object')
    try:
        spawn_actor = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
        request = SpawnObjectRequest()
        # print(request)
        request.type = actor_type
        request.id = 'ego'
        request.transform = Transform(spawn_point, spawn_rotation)
        request.random_pose = False
        print(request)
        ego_pose = Pose(spawn_point, spawn_rotation)
        response = spawn_actor(request.type, request.id, request.attributes, ego_pose, request.attach_to,
                               request.random_pose)
        rospy.loginfo(f"Spawned actor {response.id} at spawn point: {spawn_point}")
        return response.id
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    rospy.init_node('carla_spawn_vehicle', anonymous=True, log_level=rospy.DEBUG)

    # Example spawn point (adjust as needed)
    spawn_point = Vector3(x=-65.90, y=61.191028093181394, z=10.0)
    spawn_rotation = Quaternion(0, 0, 0, 0)

    # Example actor type (adjust as needed)
    actor_type = 'vehicle.tesla.model3'

    actor_id = spawn_vehicle(actor_type, spawn_point, spawn_rotation)

    # Keep the script running
    rospy.spin()