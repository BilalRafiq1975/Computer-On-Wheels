#!/usr/bin/env python
import rospy
from carla_msgs.msg import CarlaLidarMeasurement, CarlaWorldInfo
from carla_msgs.srv import SpawnObject, DestroyObject
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2

class LidarObstacleDetector:
    def __init__(self):
        rospy.init_node('lidar_obstacle_detector')

        # Service to spawn and destroy objects in CARLA
        rospy.wait_for_service('/carla/spawn_object')
        rospy.wait_for_service('/carla/destroy_object')

        self.spawn_service = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
        self.destroy_service = rospy.ServiceProxy('/carla/destroy_object', DestroyObject)

        # Subscriber to LiDAR data
        self.lidar_subscriber = rospy.Subscriber('/carla/lidar', PointCloud2, self.lidar_callback)
        self.world_info_sub = rospy.Subscriber('/carla/world_info', CarlaWorldInfo, self.world_info_callback)

        self.current_lidar_data = None

        # LiDAR specifications (customize the pose based on your environment)
        self.lidar_id = None
        self.lidar_pose = Pose()
        self.lidar_pose.position.x = 0.0
        self.lidar_pose.position.y = 0.0
        self.lidar_pose.position.z = 2.5  # Height of the LiDAR sensor
        self.lidar_pose.orientation.x = 0.0
        self.lidar_pose.orientation.y = 0.0
        self.lidar_pose.orientation.z = 0.0
        self.lidar_pose.orientation.w = 1.0

        # LiDAR parameters
        self.spawn_lidar()

    def spawn_lidar(self):
        """
        Spawns a LiDAR sensor in CARLA.
        """
        try:
            # LiDAR type is 'sensor.lidar.ray_cast'
            lidar_request = SpawnObject()
            lidar_request.type = 'sensor.lidar.ray_cast'
            lidar_request.id = 'lidar1'
            lidar_request.attach_to = 0  # Attach to ego vehicle, use 0 for world attachment
            lidar_request.transform = self.lidar_pose

            response = self.spawn_service(lidar_request)
            self.lidar_id = response.id
            rospy.loginfo(f"LiDAR spawned successfully with ID: {self.lidar_id}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def lidar_callback(self, lidar_data):
        """
        Callback for processing LiDAR data.
        """
        self.current_lidar_data = lidar_data
        rospy.loginfo("Received LiDAR data.")

        # Parse the point cloud data
        cloud_points = pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True)
        for point in cloud_points:
            x, y, z = point
            distance = (x**2 + y**2 + z**2) ** 0.5  # Euclidean distance
            rospy.loginfo(f"Obstacle detected at [x={x}, y={y}, z={z}, distance={distance}]")

    def world_info_callback(self, data):
        """
        Callback for world information, can be used to sync or handle world changes.
        """
        rospy.loginfo(f"World Time: {data.elapsed_seconds}s")

    def cleanup(self):
        """
        Destroy LiDAR sensor when shutting down.
        """
        if self.lidar_id:
            try:
                self.destroy_service(self.lidar_id)
                rospy.loginfo(f"LiDAR {self.lidar_id} destroyed successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to destroy LiDAR: {e}")

if __name__ == '__main__':
    try:
        lidar_detector = LidarObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        lidar_detector.cleanup()
