#!/usr/bin/env python
import rospy
from carla_msgs.msg import CarlaRadarMeasurement, CarlaWorldInfo
from carla_msgs.srv import SpawnObject, DestroyObject
from geometry_msgs.msg import Pose

class RadarObstacleDetector:
    def __init__(self):
        rospy.init_node('radar_obstacle_detector')

        # Service to spawn and destroy objects in CARLA
        rospy.wait_for_service('/carla/spawn_object')
        rospy.wait_for_service('/carla/destroy_object')

        self.spawn_service = rospy.ServiceProxy('/carla/spawn_object', SpawnObject)
        self.destroy_service = rospy.ServiceProxy('/carla/destroy_object', DestroyObject)

        # Subscriber to radar data
        self.radar_subscriber = rospy.Subscriber('/carla/radar', CarlaRadarMeasurement, self.radar_callback)
        self.world_info_sub = rospy.Subscriber('/carla/world_info', CarlaWorldInfo, self.world_info_callback)

        self.current_radar_data = None

        # Radar specifications (customize the pose based on your environment)
        self.radar_id = None
        self.radar_pose = Pose()
        self.radar_pose.position.x = 0.0
        self.radar_pose.position.y = 0.0
        self.radar_pose.position.z = 2.5  # Height of the radar sensor
        self.radar_pose.orientation.x = 0.0
        self.radar_pose.orientation.y = 0.0
        self.radar_pose.orientation.z = 0.0
        self.radar_pose.orientation.w = 1.0

        # Radar parameters
        self.spawn_radar()

    def spawn_radar(self):
        """
        Spawns a radar sensor in CARLA.
        """
        try:
            # Radar type is 'sensor.other.radar'
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

    def radar_callback(self, radar_data):
        """
        Callback for processing radar data.
        """
        self.current_radar_data = radar_data
        rospy.loginfo("Received radar data.")
        for detection in radar_data.detections:
            velocity = detection.velocity
            altitude = detection.altitude
            azimuth = detection.azimuth
            depth = detection.depth  # Distance to the detected obstacle

            # Log the obstacle data
            rospy.loginfo(f"Obstacle detected: Velocity={velocity}, Depth={depth}, Altitude={altitude}, Azimuth={azimuth}")

    def world_info_callback(self, data):
        """
        Callback for world information, can be used to sync or handle world changes.
        """
        rospy.loginfo(f"World Time: {data.elapsed_seconds}s")

    def cleanup(self):
        """
        Destroy radar sensor when shutting down.
        """
        if self.radar_id:
            try:
                self.destroy_service(self.radar_id)
                rospy.loginfo(f"Radar {self.radar_id} destroyed successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to destroy radar: {e}")

if __name__ == '__main__':
    try:
        radar_detector = RadarObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        radar_detector.cleanup()
