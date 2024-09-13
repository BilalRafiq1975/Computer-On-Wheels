import rospy
import math
from geometry_msgs.msg import Point, Pose
from some_custom_msgs.msg import Vehicle  # Assuming a custom Vehicle message or substitute with correct type

class VehicleObstacleDetector:
    def __init__(self):
        self.ignore_vehicles = False
        self.base_vehicle_threshold = 20.0  # Example threshold for vehicle obstacle detection
        self.vehicle = rospy.get_param('~vehicle_pose_topic', '/ego_vehicle/pose')
        self.map = None  # You will need a map or waypoint service here, e.g., from a navigation stack

    def compute_distance(self, point1, point2):
        """Utility to compute Euclidean distance between two 3D points."""
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

    def is_within_distance(self, object_pose, ego_pose, max_distance, angle_range):
        """Utility to check if object is within distance and angle range."""
        distance = self.compute_distance(object_pose.position, ego_pose.position)
        if distance > max_distance:
            return False

        # Calculate angle difference between object and ego vehicle
        heading_diff = self.calculate_heading_difference(ego_pose, object_pose)
        return angle_range[0] <= heading_diff <= angle_range[1]

    def calculate_heading_difference(self, ego_pose, object_pose):
        """Calculate heading difference between two poses."""
        # Assuming 2D case for simplicity. You can extend this to 3D if needed.
        ego_yaw = math.atan2(ego_pose.orientation.y, ego_pose.orientation.x)
        object_yaw = math.atan2(object_pose.orientation.y, object_pose.orientation.x)
        return math.degrees(object_yaw - ego_yaw)

    def vehicle_obstacle_detected(self, vehicle_list, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle in front of the agent blocking its path.
        :param vehicle_list: List of Vehicle messages (assuming we have custom Vehicle message)
        :param max_distance: max distance to check for obstacles
        """
        if self.ignore_vehicles:
            return False, None, -1

        if not vehicle_list:
            return False, None, -1  # No vehicles to process

        if not max_distance:
            max_distance = self.base_vehicle_threshold

        # Get ego vehicle pose
        ego_pose = rospy.wait_for_message(self.vehicle, Pose)

        for target_vehicle in vehicle_list:
            # Get target vehicle's pose (assuming Vehicle message has a 'pose' field)
            target_pose = target_vehicle.pose

            # Check distance between ego and target vehicle
            distance = self.compute_distance(ego_pose.position, target_pose.position)
            if distance > max_distance:
                continue  # Skip if target vehicle is beyond max distance

            # Check if the vehicle is within the angular range
            if self.is_within_distance(target_pose, ego_pose, max_distance, [low_angle_th, up_angle_th]):
                return True, target_vehicle, distance

        return False, None, -1
