#!/usr/bin/env python

import rospy
from carla_msgs.srv import SpawnObject, GetBlueprints

def get_available_blueprints():
    rospy.wait_for_service('/carla/get_blueprints')

    try:
        get_blueprints_proxy = rospy.ServiceProxy('/carla/get_blueprints', GetBlueprints)
        response = get_blueprints_proxy()
        return response.available_blueprints

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return []

def spawn_vehicle():
    rospy.init_node('spawn_vehicle_node', anonymous=True)

    # Get available vehicle blueprints
    available_blueprints = get_available_blueprints()

    if not available_blueprints:
        rospy.logerr("No available vehicle blueprints.")
        return

    # Choose a vehicle blueprint (modify as needed)
    vehicle_blueprint = 'vehicle.tesla.model3'

    # Spawn the vehicle
    rospy.wait_for_service('/carla/spawn_object')

    try:
        spawn_object_proxy = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
        response = spawn_object_proxy(vehicle_blueprint, rospy.get_param('/carla/ego_vehicle/kinematics'))

        # Print information about the spawned vehicle
        rospy.loginfo(f"Vehicle spawned with ID: {response.id}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    spawn_vehicle()

