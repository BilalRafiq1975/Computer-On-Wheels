#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
import Light_state
from styx_msgs.msg import Lane
from std_msgs.msg import Float32
import math
import yaml

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.lights = []
        self.speed = 0
        self.current_state_pub = rospy.Publisher('/current_state', Float32, queue_size=1)  # Publish current state

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/vehicle/speed', Float32, self.speed_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string, Loader=yaml.FullLoader)

        rospy.Timer(rospy.Duration(0.1), self.process_traffic_lights)  # Periodically check traffic lights
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def speed_cb(self, msg):
        self.speed = msg.data

    def get_euclidean_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

    def get_closest_waypoint(self, pose):
        min_dist = float('inf')
        closest_wp_index = -1
        for i, waypoint in enumerate(self.waypoints):
            dist = self.get_euclidean_distance(pose.position, waypoint.pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_wp_index = i
        return closest_wp_index

    def process_traffic_lights(self, event):
        light_state = self.get_traffic_light_state()
        stop_line_positions = self.config['stop_line_positions']
        car_wp = self.get_closest_waypoint(self.pose.pose) if self.pose else 0

        light_wp = -1
        min_dist = float('inf')

        for i, position in enumerate(stop_line_positions):
            stop_line_pose = Pose()
            stop_line_pose.position.x = position[0]
            stop_line_pose.position.y = position[1]
            stop_line_wp = self.get_closest_waypoint(stop_line_pose)

            dist_car_light = stop_line_wp - car_wp

            if 0 < dist_car_light < min_dist:
                min_dist = dist_car_light
                light_wp = stop_line_wp

        if light_wp != -1:
            if light_state == 'yellow':
                self.handle_yellow_light(min_dist)
            elif light_state == 'red':
                self.stop()
            elif light_state == 'green':
                self.proceed()

    def get_traffic_light_state(self):
        if self.lights:
            
            for light in self.lights:
                if light.state == TrafficLight.RED:
                    return 'red'
                elif light.state == TrafficLight.YELLOW:
                    return 'yellow'
                elif light.state == TrafficLight.GREEN:
                    return 'green'
        return 'unknown'

    def handle_yellow_light(self, distance_to_stop_line):
        threshold_distance = 30  # Distance in meters to decide whether to stop or proceed
        time_until_red = 3  # Assumed time until the light turns red
        speed_mps = self.speed / 3.6  # Convert speed from km/h to m/s
        stopping_distance = speed_mps * time_until_red

        if distance_to_stop_line <= threshold_distance:
            if stopping_distance < distance_to_stop_line:
                self.proceed()
            else:
                self.stop()
        else:
            self.proceed()  # Proceed if far enough from the stop line

    def proceed(self):
        rospy.loginfo("Proceeding through the traffic light.")
        target_speed = self.speed
        self.current_state_pub.publish(target_speed)

        if target_speed > MAX_SPEED:
          target_speed = MAX_SPEED

        self.current_state_pub.publish(target_speed)

    def stop(self):
        rospy.loginfo("Stopping at the traffic light.")
        stop_speed = 0  # Target speed for stopping
        self.current_state_pub.publish(stop_speed)  # Publish the stop command
        self.stop_speed

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic light detector node.')
