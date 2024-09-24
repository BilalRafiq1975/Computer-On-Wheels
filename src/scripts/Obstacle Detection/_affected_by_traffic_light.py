import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def _affected_by_traffic_light(self, lights_list=None, max_distance=None):
    """
    Method to check if the vehicle is affected by a red light (ROS version).
    """
    ego_vehicle_location = self._vehicle.get_location()  # Substitute with ROS topic or service
    ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)  # Handle waypoints via ROS, if needed

    if not lights_list:
        return False

    if max_distance is None:
        max_distance = self._proximity_threshold

    for traffic_light in lights_list:
        object_waypoint = self._map.get_waypoint(traffic_light.get_location())
        if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
            continue

        if ego_vehicle_waypoint.transform.location.distance(traffic_light.get_location()) > max_distance:
            continue

        if traffic_light.state == "RED":  # Assuming traffic light state is communicated via ROS topics
            return True

    return False
