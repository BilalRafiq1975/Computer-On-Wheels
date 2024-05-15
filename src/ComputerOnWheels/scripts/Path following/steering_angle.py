#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from math import atan2


class SteeringControllerNode:
    def __init__(self):
        rospy.init_node('steering_controller_node', anonymous=True)

        # Subscribe to the current position of the vehicle
        rospy.Subscriber('/carla/ego_vehicle/pose', PoseStamped, self.pose_callback)

        # Publisher for the steering angle
        self.steering_pub = rospy.Publisher('/carla/ego_vehicle/steering_angle', Float32, queue_size=10)

        # Set the wheelbase of the vehicle (distance between the front and rear axles)
        self.wheelbase = 2.5  # Example wheelbase in meters

    def pose_callback(self, pose_msg):
        # Extract the vehicle's position from the pose message
        vehicle_position = pose_msg.pose.position.x  # Assuming the vehicle's position is along the x-axis

        # Calculate the desired lane center (for example purposes, you might obtain this from a map)
        desired_lane_center = 0.0  # Assuming the lane center is at x=0

        # Calculate the steering angle using Ackermann steering control
        steering_angle = self.calculate_steering_angle(vehicle_position, desired_lane_center)

        # Publish the steering angle
        steering_msg = Float32()
        steering_msg.data = steering_angle
        self.steering_pub.publish(steering_msg)

    def calculate_steering_angle(self, vehicle_position, desired_lane_center):
        # Calculate the error (deviation from the desired lane center)
        error = desired_lane_center - vehicle_position

        # Calculate the steering angle using Ackermann steering geometry
        steering_angle = self.calculate_ackermann_steering(error)

        return steering_angle

    def calculate_ackermann_steering(self, error):
        # Implement Ackermann steering geometry
        # Calculate the steering angle based on the error (deviation from the desired lane center)

        # Calculate the steering angle using Ackermann steering geometry
        steering_angle = atan2(2.0 * error * self.wheelbase, self.wheelbase ** 2)

        return steering_angle


if __name__ == '__main__':
    try:
        # Initialize the ROS node
        node = SteeringControllerNode()

        # Spin the node
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
