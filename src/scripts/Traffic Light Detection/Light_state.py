#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from styx_msgs.msg import TrafficLight

class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_detector')

        self.bridge = CvBridge()
        self.camera_image = None
        self.has_image = False
        
        rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=14400000)
        self.light_state_pub = rospy.Publisher('/traffic_light_state', Int32, queue_size=1)

        rospy.spin()

    def image_cb(self, msg):
        self.has_image = True
        self.camera_image = msg
        state = self.get_light_state()
        self.light_state_pub.publish(Int32(state))

    def get_light_state(self):
        """Determines the current color of the traffic light using color detection."""
        if not self.has_image:
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        cv_image = cv2.resize(cv_image, (800, 600), interpolation=cv2.INTER_AREA)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        # Define color ranges for red, yellow, and green
        red_lower = np.array([0, 100, 100])
        red_upper = np.array([10, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        green_lower = np.array([40, 100, 100])
        green_upper = np.array([70, 255, 255])

        # Create masks for each color
        red_mask = cv2.inRange(hsv_image, red_lower, red_upper)
        yellow_mask = cv2.inRange(hsv_image, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv_image, green_lower, green_upper)

        # Count the number of pixels for each color
        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        # Determine the state based on the count of pixels
        if red_count > yellow_count and red_count > green_count:
            return TrafficLight.RED
        elif yellow_count > red_count and yellow_count > green_count:
            return TrafficLight.YELLOW
        elif green_count > red_count and green_count > yellow_count:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TrafficLightDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic light detector node.')
