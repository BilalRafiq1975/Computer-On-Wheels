#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from carla_msgs.msg import CarlaRadarMeasurement
from carla_msgs.srv import SpawnObject, DestroyObject
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
import numpy as np

class SensorFusionObstacleDetector:
    def __init__(self):
        rospy.init_node('sensor_fusion_obstacle_detector')

        # Service to spawn and destroy sensors in CARLA
        rospy.wait_for_service('/carla/spawn_object')
        rospy.wait_for_service('/carla/destroy_object')

        self.spawn_service = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
        self.destroy_service = rospy.ServiceProxy('/carla/destroy_object', DestroyObject)

        # Radar and LiDAR sensors data
        self.lidar_data = None
        self.radar_data = None

        # LiDAR and Radar topic subscribers
        self.lidar_subscriber = rospy.Subscriber('/carla/lidar', PointCloud2, self.lidar_callback)
        self.radar_subscriber = rospy.Subscriber('/carla/radar', CarlaRadarMeasurement, self.radar_callback)

        # Spawn LiDAR and Radar sensors
        self.lidar_id = None
        self.radar_id = None
        self.lidar_pose = Pose()
        self.radar_pose = Pose()

        # Position and orientation of LiDAR and radar (customize this as needed)
        self.lidar_pose.position.x = 0.0
        self.lidar_pose.position.y = 0.0
        self.lidar_pose.position.z = 2.5  # LiDAR height
        self.radar_pose.position.x = 0.0
        self.radar_pose.position.y = 0.0
        self.radar_pose.position.z = 2.5  # Radar height

        # Spawn sensors
        self.spawn_lidar()
        self.spawn_radar()

    def spawn_lidar(self):
        """
        Spawns a LiDAR sensor in CARLA.
        """
        try:
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

    def spawn_radar(self):
        """
        Spawns a Radar sensor in CARLA.
        """
        try:
            radar_request = SpawnObject()
            radar_request.type = 'sensor.other.radar'
            radar_request.id = 'radar1'
            radar_request.attach_to = 0  # Attach to ego vehicle, use 0 for world attachment
            radar_request.transform = self.radar_pose

            response = self.spawn_service(radar_request)
            self.radar_id = response.id
            rospy.loginfo(f"Radar spawned successfully with ID: {self.radar_id}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def lidar_callback(self, lidar_data):
        """
        Callback for processing LiDAR data.
        """
        self.lidar_data = lidar_data
        rospy.loginfo("Received LiDAR data.")
        self.fuse_data()

    def radar_callback(self, radar_data):
        """
        Callback for processing radar data.
        """
        self.radar_data = radar_data
        rospy.loginfo("Received Radar data.")
        self.fuse_data()

    def fuse_data(self):
        """
        Fuse radar and LiDAR data to detect obstacles.
        This will process both sensor inputs and output fused obstacle information.
        """
        if self.lidar_data is None or self.radar_data is None:
            return

        rospy.loginfo("Performing sensor fusion...")

        # Convert LiDAR point cloud data to a numpy array for easier processing
        lidar_points = np.array(list(pc2.read_points(self.lidar_data, field_names=("x", "y", "z"), skip_nans=True)))

        # Go through radar detections
        for detection in self.radar_data.detections:
            radar_x = detection.depth * np.cos(detection.azimuth)
            radar_y = detection.depth * np.sin(detection.azimuth)
            radar_z = detection.depth * np.sin(detection.altitude)

            radar_point = np.array([radar_x, radar_y, radar_z])

            # Find the closest LiDAR point to the radar point for fusion (nearest neighbor)
            distances = np.linalg.norm(lidar_points - radar_point, axis=1)
            closest_idx = np.argmin(distances)
            closest_lidar_point = lidar_points[closest_idx]

            # If the distance between the radar point and LiDAR point is small enough, consider it the same object
            fusion_threshold = 1.0  # Customize the fusion threshold distance
            if distances[closest_idx] < fusion_threshold:
                fused_point = (radar_point + closest_lidar_point) / 2.0
                rospy.loginfo(f"Fused obstacle detected at {fused_point} "
                              f"from Radar [x={radar_x}, y={radar_y}, z={radar_z}] "
                              f"and LiDAR [x={closest_lidar_point[0]}, y={closest_lidar_point[1]}, z={closest_lidar_point[2]}]")

    def cleanup(self):
        """
        Destroy sensors when shutting down.
        """
        if self.lidar_id:
            try:
                self.destroy_service(self.lidar_id)
                rospy.loginfo(f"LiDAR {self.lidar_id} destroyed successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to destroy LiDAR: {e}")

        if self.radar_id:
            try:
                self.destroy_service(self.radar_id)
                rospy.loginfo(f"Radar {self.radar_id} destroyed successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to destroy Radar: {e}")

if __name__ == '__main__':
    try:
        fusion_detector = SensorFusionObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        fusion_detector.cleanup()
