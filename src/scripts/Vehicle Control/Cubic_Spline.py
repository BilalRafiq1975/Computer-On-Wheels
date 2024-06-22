#!/usr/bin/env python

import rospy
import numpy as np
from carla_ros_bridge.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion

class CubicSpline:
    def __init__(self, points):
        self.points = points
        self.n = len(points)
        self.a = [p[1] for p in points]
        self.b, self.c, self.d = [0]*(self.n-1), [0]*self.n, [0]*(self.n-1)
        self.h = [points[i+1][0] - points[i][0] for i in range(self.n-1)]
        self._calculate_spline_coefficients()

    def _calculate_spline_coefficients(self):
        A = np.zeros((self.n, self.n))
        b = np.zeros(self.n)
        A[0, 0] = A[-1, -1] = 1

        for i in range(1, self.n-1):
            A[i, i-1] = self.h[i-1]
            A[i, i] = 2 * (self.h[i-1] + self.h[i])
            A[i, i+1] = self.h[i]
            b[i] = 3 * ((self.a[i+1] - self.a[i]) / self.h[i] - (self.a[i] - self.a[i-1]) / self.h[i-1])

        self.c = np.linalg.solve(A, b)
        
        for i in range(self.n-1):
            self.b[i] = (self.a[i+1] - self.a[i]) / self.h[i] - self.h[i] * (2*self.c[i] + self.c[i+1]) / 3
            self.d[i] = (self.c[i+1] - self.c[i]) / (3*self.h[i])

    def get_spline_points(self, delta_t=0.01):
        spline_points = []
        for i in range(self.n-1):
            t = 0
            while t <= 1:
                x = self.points[i][0] + t * self.h[i]
                y = self.a[i] + self.b[i]*t + self.c[i]*t**2 + self.d[i]*t**3
                spline_points.append((x, y))
                t += delta_t
        return spline_points

class PathPlanner:
    def __init__(self):
        self.waypoints = []

    def add_waypoint(self, x, y):
        self.waypoints.append((x, y))

    def generate_smooth_path(self):
        spline = CubicSpline(self.waypoints)
        return spline.get_spline_points()

class CarlaVehicleController:
    def __init__(self):
        rospy.init_node('carla_vehicle_controller')
        self.control_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)
        self.path_pub = rospy.Publisher('/smoothed_path', Path, queue_size=1)
        self.status_sub = rospy.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, self.status_callback)
        self.path = []
        self.current_pose = None

    def status_callback(self, data):
        self.current_pose = (data.pose.position.x, data.pose.position.y)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'

        for point in path:
            pose = PoseStamped()
            pose.header = Header()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def follow_path(self, path):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose:
                control_msg = CarlaEgoVehicleControl()
                target_x, target_y = path[0]
                control_msg.steer = self.calculate_steer(target_x, target_y)
                control_msg.throttle = 0.5
                control_msg.brake = 0.0
                self.control_pub.publish(control_msg)
                if self.reached_waypoint(target_x, target_y):
                    path.pop(0)
            rate.sleep()

    def calculate_steer(self, target_x, target_y):
        angle_to_target = np.arctan2(target_y - self.current_pose[1], target_x - self.current_pose[0])
        yaw = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[2]
        return angle_to_target - yaw

    def reached_waypoint(self, target_x, target_y):
        return np.hypot(target_x - self.current_pose[0], target_y - self.current_pose[1]) < 1.0

if __name__ == "__main__":
    planner = PathPlanner()
    planner.add_waypoint(0, 0)
    planner.add_waypoint(10, 10)
    planner.add_waypoint(20, 0)
    planner.add_waypoint(30, 10)
    
    smooth_path = planner.generate_smooth_path()

    controller = CarlaVehicleController()
    controller.publish_path(smooth_path)
    controller.follow_path(smooth_path)
