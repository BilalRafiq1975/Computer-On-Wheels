#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaTrafficLightInfo

# Other existing imports
import random
import numpy as np
import carla
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption
from agents.navigation.behavior_types import Cautious, Aggressive, Normal
from agents.tools.misc import get_speed, positive, is_within_distance, compute_distance

class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles.
    """

    def __init__(self, vehicle, behavior='normal'):
        """
        Constructor method.

        :param vehicle: actor to apply local planner logic to
        :param behavior: type of agent to apply
        """
        super(BehaviorAgent, self).__init__(vehicle)
        self._look_ahead_steps = 0
        self._speed = 0
        self._speed_limit = 0
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5

        # Initialize ROS node
        rospy.init_node('behavior_agent_node', anonymous=True)

        # ROS Publishers
        self.speed_pub = rospy.Publisher('/agent/speed', Float32, queue_size=10)
        self.control_pub = rospy.Publisher('/carla/ego_vehicle/control_cmd', CarlaEgoVehicleControl, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber('/carla/traffic_lights', CarlaTrafficLightInfo, self.traffic_light_callback)

        # Behavior selection
        if behavior == 'cautious':
            self._behavior = Cautious()
        elif behavior == 'normal':
            self._behavior = Normal()
        elif behavior == 'aggressive':
            self._behavior = Aggressive()

    def _update_information(self):
        """
        Updates the information about the ego vehicle.
        """
        self._speed = get_speed(self._vehicle)
        self._speed_limit = self._vehicle.get_speed_limit()
        self._local_planner.set_speed(self._speed_limit)
        self._direction = self._local_planner.target_road_option
        if self._direction is None:
            self._direction = RoadOption.LANEFOLLOW

        self._look_ahead_steps = int((self._speed_limit) / 10)
        self._incoming_waypoint, self._incoming_direction = self._local_planner.get_incoming_waypoint_and_direction(
            steps=self._look_ahead_steps)
        if self._incoming_direction is None:
            self._incoming_direction = RoadOption.LANEFOLLOW

        # Publish vehicle speed to ROS topic
        self.speed_pub.publish(self._speed)

    def traffic_light_callback(self, msg):
        """
        Callback function to handle traffic light information received from ROS.
        """
        # Process traffic light data from ROS topic
        if msg.state == CarlaTrafficLightInfo.RED:
            rospy.loginfo("Red light ahead! Stopping.")
            self.emergency_stop()

    def traffic_light_manager(self):
        """
        Manage behaviors for red lights using ROS.
        """
        return False  # Traffic light data now comes from ROS

    # The rest of the methods would remain similar but can now interact with ROS topics
    # such as publishing control commands or subscribing to other sensors.

    def run_step(self, debug=False):
        """
        Execute one step of navigation and publish control commands via ROS.
        """
        self._update_information()

        control = self._local_planner.run_step(debug=debug)

        # Create a ROS message for the vehicle control
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = control.throttle
        control_msg.brake = control.brake
        control_msg.steer = control.steer

        # Publish the control message to ROS
        self.control_pub.publish(control_msg)

        return control

    def emergency_stop(self):
        """
        Perform an emergency stop and publish the control message via ROS.
        """
        control = carla.VehicleControl()
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = True

        # Create and publish the emergency stop control via ROS
        control_msg = CarlaEgoVehicleControl()
        control_msg.throttle = 0.0
        control_msg.brake = 1.0
        control_msg.steer = 0.0
        self.control_pub.publish(control_msg)

        return control
