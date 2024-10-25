#!/usr/bin/env python

import carla
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class CarlaCameraNode:
    def __init__(self):
        rospy.init_node('carla_camera_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/carla/camera/image_raw', Image, queue_size=1)

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Spawn a vehicle
        vehicle_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        spawn_point = self.world.get_map().get_spawn_points()[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)

        # Spawn the camera
        self.camera = self.spawn_camera(self.vehicle)

        # Listen to camera's images
        self.camera.listen(self.process_image)

    def spawn_camera(self, vehicle):
        # Define the camera's position and rotation
        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=0.7),  # Position in front of the vehicle
            carla.Rotation(pitch=0, yaw=0, roll=0)
        )

        # Spawn the camera
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        # Return the camera object
        return camera

    def process_image(self, image):
        # Convert CARLA image to OpenCV format
        image_np = np.frombuffer(image.raw_data, dtype=np.uint8)
        image_np = image_np.reshape((image.height, image.width, 4))  # RGBA
        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGBA2BGR)  # Convert to BGR

        # Publish the image
        ros_image = self.bridge.cv2_to_imgmsg(image_np, encoding='bgr8')
        self.image_pub.publish(ros_image)

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == "__main__":
    try:
        camera_node = CarlaCameraNode()
        camera_node.run()
    except rospy.ROSInterruptException:
        pass
