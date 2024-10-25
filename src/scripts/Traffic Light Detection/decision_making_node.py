#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class DecisionMakingNode:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('decision_making_node', anonymous=True)

        # Subscribe to traffic light state
        self.traffic_light_sub = rospy.Subscriber('/traffic_light_state', String, self.traffic_light_callback)

    def traffic_light_callback(self, msg):
        traffic_light_state = msg.data
        if traffic_light_state == "Red":
            self.stop_vehicle()
        elif traffic_light_state == "Yellow":
            self.prepare_to_stop()
        elif traffic_light_state == "Green":
            self.proceed()

    def stop_vehicle(self):
        rospy.loginfo("Stopping vehicle.")

    def prepare_to_stop(self):
        rospy.loginfo("Preparing to stop.")

    def proceed(self):
        rospy.loginfo("Proceeding through intersection.")

if __name__ == '__main__':
    try:
        DecisionMakingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

