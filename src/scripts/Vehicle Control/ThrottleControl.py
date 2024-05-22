#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class ThrottleController:
    def __init__(self):
        rospy.init_node('throttle_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.target_linear_speed = 0.0
        self.current_linear_speed = 0.0
        self.acceleration = 0.1  # Acceleration rate (m/s^2)
        self.deceleration = 0.2  # Deceleration rate (m/s^2)
        self.max_speed = 1.0  # Maximum linear speed (m/s)
        self.min_speed = -0.5  # Minimum linear speed (m/s)
        self.twist_msg = Twist()

    def control_loop(self):
        while not rospy.is_shutdown():
            # Update current speed towards the target speed
            if self.target_linear_speed > self.current_linear_speed:
                self.current_linear_speed = min(self.current_linear_speed + self.acceleration / 10.0, self.target_linear_speed)
            elif self.target_linear_speed < self.current_linear_speed:
                self.current_linear_speed = max(self.current_linear_speed - self.deceleration / 10.0, self.target_linear_speed)
            
            # Update twist message with current speed
            self.twist_msg.linear.x = self.current_linear_speed
            
            # Publish twist message
            self.cmd_vel_pub.publish(self.twist_msg)
            
            # Sleep to maintain control rate
            self.rate.sleep()

    def set_target_linear_speed(self, speed):
        # Set target linear speed within the allowable range
        self.target_linear_speed = max(min(speed, self.max_speed), self.min_speed)

    def shutdown(self):
        rospy.loginfo("Stopping the throttle controller...")
        self.twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        controller = ThrottleController()
        controller.set_target_linear_speed(0.5)  # Set initial target speed
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass

