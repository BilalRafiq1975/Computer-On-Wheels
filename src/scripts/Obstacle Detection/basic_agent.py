"""
This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights.
It can also make use of the global route planner to follow a specified route.
ROS integration is added to publish control commands and receive vehicle state.
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from enum import Enum
from shapely.geometry import Polygon

from agents.navigation.local_planner import LocalPlanner
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.tools.misc import get_speed, is_within_distance, get_trafficlight_trigger_location, compute_distance


class BasicAgent(object):
    """
    BasicAgent implements an agent that navigates the scene.
    This agent respects traffic lights and other vehicles, but ignores stop signs.
    It has several functions available to specify the route that the agent must follow,
    as well as to change its parameters in case a different driving mode is desired.
    """

    def __init__(self, vehicle_id, target_speed=20, opt_dict={}):
        """
        Initialization of the agent parameters, the local, and the global planner.
        ROS integration initializes publishers and subscribers.

            :param vehicle_id: id of the vehicle to apply agent logic to
            :param target_speed: speed (in Km/h) at which the vehicle will move
            :param opt_dict: dictionary in case some of its parameters want to be changed.
                This also applies to parameters related to the LocalPlanner.
        """
        self._vehicle_id = vehicle_id
        self._target_speed = target_speed

        # ROS publishers and subscribers
        rospy.init_node('basic_agent_node', anonymous=True)
        self.control_pub = rospy.Publisher('/carla/{}/vehicle_control_cmd'.format(self._vehicle_id), CarlaEgoVehicleControl, queue_size=1)
        rospy.Subscriber('/carla/{}/vehicle_status'.format(self._vehicle_id), CarlaEgoVehicleStatus, self.vehicle_status_callback)

        # Base parameters
        self._ignore_traffic_lights = True
        self._ignore_stop_signs = False
        self._ignore_vehicles = False
        self._sampling_resolution = 2.0
        self._base_tlight_threshold = 5.0  # meters
        self._base_vehicle_threshold = 5.0  # meters
        self._max_brake = 0.5

        # Change parameters according to the dictionary
        opt_dict['target_speed'] = target_speed
        if 'ignore_traffic_lights' in opt_dict:
            self._ignore_traffic_lights = opt_dict['ignore_traffic_lights']
        if 'ignore_stop_signs' in opt_dict:
            self._ignore_stop_signs = opt_dict['ignore_stop_signs']
        if 'ignore_vehicles' in opt_dict:
            self._ignore_vehicles = opt_dict['ignore_vehicles']
        if 'sampling_resolution' in opt_dict:
            self._sampling_resolution = opt_dict['sampling_resolution']
        if 'base_tlight_threshold' in opt_dict:
            self._base_tlight_threshold = opt_dict['base_tlight_threshold']
        if 'base_vehicle_threshold' in opt_dict:
            self._base_vehicle_threshold = opt_dict['base_vehicle_threshold']
        if 'max_brake' in opt_dict:
            self._max_brake = opt_dict['max_brake']

        # Initialize the planners
        self._local_planner = LocalPlanner(self._vehicle_id, opt_dict=opt_dict)
        self._global_planner = GlobalRoutePlanner(self._sampling_resolution)

        self.current_speed = 0

    def vehicle_status_callback(self, msg):
        """Callback to update the vehicle's current speed from ROS topic."""
        self.current_speed = msg.velocity * 3.6  # m/s to km/h

    def add_emergency_stop(self, control):
        """
        Overwrites the throttle and brake values of a control to perform an emergency stop.
        The steering is kept the same to avoid going out of the lane when stopping during turns.

            :param control (CarlaEgoVehicleControl): control to be modified
        """
        control.throttle = 0.0
        control.brake = self._max_brake
        control.hand_brake = False
        return control

    def set_target_speed(self, speed):
        """
        Changes the target speed of the agent.
            :param speed (float): target speed in Km/h
        """
        self._local_planner.set_speed(speed)

    def follow_speed_limits(self, value=True):
        """
        If active, the agent will dynamically change the target speed according to the speed limits.

            :param value (bool): whether or not to activate this behavior
        """
        self._local_planner.follow_speed_limits(value)

    def get_local_planner(self):
        """Get method for the local planner."""
        return self._local_planner

    def get_global_planner(self):
        """Get method for the global planner."""
        return self._global_planner

    def set_destination(self, end_location, start_location=None):
        """
        Creates a list of waypoints between a start and end location based on the global router's route.
        Adds it to the local planner.

            :param end_location (carla.Location): final location of the route
            :param start_location (carla.Location): starting location of the route
        """
        if not start_location:
            start_location = self._local_planner.target_waypoint.transform.location
            clean_queue = True
        else:
            start_location = self._vehicle_id.get_location()
            clean_queue = False

        start_waypoint = self._global_planner.get_waypoint(start_location)
        end_waypoint = self._global_planner.get_waypoint(end_location)

        route_trace = self.trace_route(start_waypoint, end_waypoint)
        self._local_planner.set_global_plan(route_trace, clean_queue=clean_queue)

    def set_global_plan(self, plan, stop_waypoint_creation=True, clean_queue=True):
        """
        Adds a specific plan to the agent.

            :param plan: list of [carla.Waypoint, RoadOption] representing the route to be followed
            :param stop_waypoint_creation: stops the automatic random creation of waypoints
            :param clean_queue: resets the current agent's plan
        """
        self._local_planner.set_global_plan(
            plan,
            stop_waypoint_creation=stop_waypoint_creation,
            clean_queue=clean_queue
        )

    def trace_route(self, start_waypoint, end_waypoint):
        """
        Calculates the shortest route between a starting and ending waypoint.

            :param start_waypoint (carla.Waypoint): initial waypoint
            :param end_waypoint (carla.Waypoint): final waypoint
        """
        start_location = start_waypoint.transform.location
        end_location = end_waypoint.transform.location
        return self._global_planner.trace_route(start_location, end_location)

    def run_step(self):
        """
        Execute one step of navigation, detecting hazards and publishing control commands.
        """
        hazard_detected = False

        # Check for possible vehicle obstacles
        max_vehicle_distance = self._base_vehicle_threshold + self.current_speed
        affected_by_vehicle, _, _ = self._vehicle_obstacle_detected(max_vehicle_distance)
        if affected_by_vehicle:
            hazard_detected = True

        # Check if the vehicle is affected by a red traffic light
        max_tlight_distance = self._base_tlight_threshold + self.current_speed
        affected_by_tlight, _ = self._affected_by_traffic_light(max_tlight_distance)
        if affected_by_tlight:
            hazard_detected = True

        # Get control from local planner
        control = self._local_planner.run_step()
        if hazard_detected:
            control = self.add_emergency_stop(control)

        # Publish control to ROS topic
        self.control_pub.publish(control)

        return control

    def done(self):
        """Check whether the agent has reached its destination."""
        return self._local_planner.done()

    def ignore_traffic_lights(self, active=True):
        """(De)activates the checks for traffic lights"""
        self._ignore_traffic_lights = active

    def ignore_stop_signs(self, active=True):
        """(De)activates the checks for stop signs"""
        self._ignore_stop_signs = active

    def ignore_vehicles(self, active=True):
        """(De)activates the checks for vehicles"""
        self._ignore_vehicles = active

    def _affected_by_traffic_light(self, max_distance=None):
        """
        Method to check if there is a red light affecting the vehicle.

            :param max_distance (float): max distance for traffic lights to be considered relevant.
                If None, the base threshold value is used
        """
        if self._ignore_traffic_lights:
            return False, None

        ego_vehicle_location = self._vehicle_id.get_location()

        # Add logic to check traffic light state using ROS (replace with ROS traffic light messages if available)
        # For now, keeping the same logic as before.
        lights_list = self._world.get_actors().filter("*traffic_light*")

        for traffic_light in lights_list:
            if traffic_light.state == carla.TrafficLightState.Red:
                trigger_location = get_trafficlight_trigger_location(traffic_light)
                if is_within_distance(trigger_location, ego_vehicle_location, traffic_light.get_transform().rotation.yaw, max_distance):
                    return True, traffic_light

        return False, None

    def _vehicle_obstacle_detected(self, max_distance=None, up_angle_th=90, low_angle_th=0, lane_offset=0):
        """
        Method to check if there is a vehicle in front blocking our path.

            :param max_distance (float): Maximum distance to check for vehicle obstacles
            :param up_angle_th (float): upper threshold of the view angle (in degrees)
            :param low_angle_th (float): lower threshold of the view angle (in degrees)
            :param lane_offset (float): lane offset for lateral checks (in meters)
        """
        if self._ignore_vehicles:
            return False, None, -1

        ego_vehicle_location = self._vehicle_id.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        vehicle_list = self._world.get_actors().filter("*vehicle*")
        for target_vehicle in vehicle_list:
            if target_vehicle.id == self._vehicle_id.id:
                continue

            target_vehicle_location = target_vehicle.get_location()
            target_vehicle_waypoint = self._map.get_waypoint(target_vehicle_location)

            if ego_vehicle_waypoint.road_id != target_vehicle_waypoint.road_id or \
               ego_vehicle_waypoint.lane_id != target_vehicle_waypoint.lane_id:
                continue

            if is_within_distance(target_vehicle_location, ego_vehicle_location, self._vehicle_id.get_transform().rotation.yaw, max_distance):
                return True, target_vehicle, compute_distance(target_vehicle_location, ego_vehicle_location)

        return False, None, -1