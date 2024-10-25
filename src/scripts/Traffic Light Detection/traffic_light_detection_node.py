#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import yaml

class TrafficLightDetector:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('traffic_light_detection_node', anonymous=True)

        # Load HSV thresholds from YAML
        self.hsv_config = rospy.get_param('~hsv_config')
        with open(self.hsv_config, 'r') as file:
            hsv_data = yaml.safe_load(file)['hsv_thresholds']
            self.red_ranges = [(np.array(hsv_data['red1']), np.array(hsv_data['red2'])),
                               (np.array(hsv_data['red3']), np.array(hsv_data['red4']))]
            self.yellow_range = (np.array(hsv_data['yellow'][0:3]), np.array(hsv_data['yellow'][3:6]))
            self.green_range = (np.array(hsv_data['green'][0:3]), np.array(hsv_data['green'][3:6]))

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber('/carla/ego_vehicle/rgb_camera/image_raw', Image, self.image_callback)

        # Publisher for traffic light state
        self.traffic_light_pub = rospy.Publisher('/traffic_light_state', String, queue_size=10)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        # Detect traffic light state
        traffic_light_state = self.detect_traffic_light(cv_image)

        # Publish traffic light state
        if traffic_light_state:
            rospy.loginfo(f"Detected Traffic Light: {traffic_light_state}")
            self.traffic_light_pub.publish(traffic_light_state)

    def detect_traffic_light(self, cv_image):
        # Convert to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Detect Red Light
        mask_red = sum([cv2.inRange(hsv_image, low, high) for low, high in self.red_ranges])
        red_detected = self.detect_light(mask_red, "Red")

        # Detect Yellow Light
        mask_yellow = cv2.inRange(hsv_image, *self.yellow_range)
        yellow_detected = self.detect_light(mask_yellow, "Yellow")

        # Detect Green Light
        mask_green = cv2.inRange(hsv_image, *self.green_range)
        green_detected = self.detect_light(mask_green, "Green")

        # Determine the state based on detected colors
        if red_detected:
            return "Red"
        elif yellow_detected:
            return "Yellow"
        elif green_detected:
            return "Green"
        else:
            return "Unknown"

    def detect_light(self, mask, color_name):
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check for significant contours
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Adjust area threshold based on detection needs
                rospy.loginfo(f"{color_name} light detected with area: {area}")
                return True
        return False

if __name__ == '__main__':
    try:
        TrafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

