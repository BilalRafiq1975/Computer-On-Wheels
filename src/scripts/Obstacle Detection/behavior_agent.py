""" This module implements an agent that roams around a track following random
waypoints and avoiding other vehicles. The agent also responds to traffic lights,
traffic signs, and has different possible configurations. """

import random
import numpy as np
import carla
import rospy  # Import ROS library
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption
from agents.navigation.behavior_types import Cautious, Aggressive, Normal
from agents.tools.misc import get_speed, positive, is_within_distance, compute_distance

class BehaviorAgent(BasicAgent):
    """
    BehaviorAgent implements an agent that navigates scenes to reach a given
    target destination, by computing the shortest possible path to it.
    This agent can correctly follow traffic signs, speed limitations,
    traffic lights, while also taking into account nearby vehicles. Lane changing
    decisions can be taken by analyzing the surrounding environment such as tailgating avoidance.
    Adding to these are possible behaviors, the agent can also keep safety distance
    from a car in front of it by tracking the instantaneous time to collision
    and keeping it in a certain range. Finally, different sets of behaviors
    are encoded in the agent, from cautious to a more aggressive ones.
    """

    def __init__(self, vehicle, behavior='normal'):
        """
        Constructor method.

            :param vehicle: actor to apply to local planner logic onto
            :param ignore_traffic_light: boolean to ignore any traffic light
            :param behavior: type of agent to apply
        """
        super(BehaviorAgent, self).__init__(vehicle)
        rospy.init_node('behavior_agent', anonymous=True)  # Initialize ROS node
        self._look_ahead_steps = 0

        # Vehicle information
        self._speed = 0
        self._speed_limit = 0
        self._direction = None
        self._incoming_direction = None
        self._incoming_waypoint = None
        self._min_speed = 5
        self._behavior = None
        self._sampling_resolution = 4.5

        # Parameters for agent behavior
        if behavior == 'cautious':
            self._behavior = Cautious()
        elif behavior == 'normal':
            self._behavior = Normal()
        elif behavior == 'aggressive':
            self._behavior = Aggressive()

        # ROS Publisher for vehicle control commands
        self.control_pub = rospy.Publisher('vehicle_control', carla.VehicleControl, queue_size=10)

    def _update_information(self):
        """
        This method updates the information regarding the ego
        vehicle based on the surrounding world.
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

    def traffic_light_manager(self):
        """
        This method is in charge of behaviors for red lights.
        """
        actor_list = self._world.get_actors()
        lights_list = actor_list.filter("*traffic_light*")
        affected, _ = self._affected_by_traffic_light(lights_list)

        return affected

    def _tailgating(self, waypoint, vehicle_list):
    """
    This method is in charge of tailgating behaviors.
    :param location: current location of the agent
    :param waypoint: current waypoint of the agent
    :param vehicle_list: list of all the nearby vehicles
    """
    # Retrieves whether lane change is allowed on the left lane
    left_turn = waypoint.left_lane_marking.lane_change
    # Retrieves whether lane change is allowed on the right lane
    right_turn = waypoint.right_lane_marking.lane_change

    # Gets the waypoint of the left lane from the current waypoint
    left_wpt = waypoint.get_left_lane()
    # Gets the waypoint of the right lane from the current waypoint
    right_wpt = waypoint.get_right_lane()

    # Checks for obstacles from the nearby vehicles in a specified proximity and angle range
    behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max(
        self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, low_angle_th=160)
    
    # If an obstacle is detected behind and the vehicle is slower than the one behind
    if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
        # If lane change to the right is possible, and the right lane is suitable
        if (right_turn == carla.LaneChange.Right or right_turn ==
                carla.LaneChange.Both) and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            # Check if there are no vehicles in the right lane in the proximity
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
            # If no vehicle is detected in the right lane
            if not new_vehicle_state:
                print("Tailgating, moving to the right!")  # Prints action
                # Sets the destination to the right lane
                end_waypoint = self._local_planner.target_waypoint
                self._behavior.tailgate_counter = 200  # Sets tailgating counter
                # Set destination to the right lane
                self.set_destination(end_waypoint.transform.location,
                                     right_wpt.transform.location)
        # If lane change to the left is possible, and the left lane is suitable
        elif left_turn == carla.LaneChange.Left and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
            # Check if there are no vehicles in the left lane in the proximity
            new_vehicle_state, _, _ = self._vehicle_obstacle_detected(vehicle_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
            # If no vehicle is detected in the left lane
            if not new_vehicle_state:
                print("Tailgating, moving to the left!")  # Prints action
                # Sets the destination to the left lane
                end_waypoint = self._local_planner.target_waypoint
                self._behavior.tailgate_counter = 200  # Sets tailgating counter
                # Set destination to the left lane
                self.set_destination(end_waypoint.transform.location,
                                     left_wpt.transform.location)

    def collision_and_car_avoid_manager(self, waypoint):
    """
    This module is in charge of warning in case of a collision
    and managing possible tailgating chances.
    :param location: current location of the agent
    :param waypoint: current waypoint of the agent
    :return vehicle_state: True if there is a vehicle nearby, False if not
    :return vehicle: nearby vehicle
    :return distance: distance to nearby vehicle
    """

    # Retrieves a list of all vehicles in the world
    vehicle_list = self._world.get_actors().filter("*vehicle*")
    # Calculates distance of a vehicle from the current waypoint
    def dist(v): return v.get_location().distance(waypoint.transform.location)
    # Filters vehicles to get only those within a 45-meter radius of the current location
    vehicle_list = [v for v in vehicle_list if dist(v) < 45 and v.id != self._vehicle.id]

    # If the direction is to change lane to the left
    if self._direction == RoadOption.CHANGELANELEFT:
        # Checks for obstacles to the left
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=-1)
    # If the direction is to change lane to the right
    elif self._direction == RoadOption.CHANGELANERIGHT:
        # Checks for obstacles to the right
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, lane_offset=1)
    else:
        # Otherwise, checks for obstacles in the current lane
        vehicle_state, vehicle, distance = self._vehicle_obstacle_detected(
            vehicle_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=30)

        # If no vehicle is detected and the agent is following the lane and speeding
        # Tailgating is also not happening
        if not vehicle_state and self._direction == RoadOption.LANEFOLLOW \
                and not waypoint.is_junction and self._speed > 10 \
                and self._behavior.tailgate_counter == 0:
            # If all conditions are met, manage tailgating
            self._tailgating(waypoint, vehicle_list)

    # Returns whether there is a vehicle nearby, the vehicle, and its distance
    return vehicle_state, vehicle, distance


    def pedestrian_avoid_manager(self, waypoint):
        """
        This module is in charge of warning in case of a collision
        with any pedestrian.

            :param location: current location of the agent
            :param waypoint: current waypoint of the agent
            :return vehicle_state: True if there is a walker nearby, False if not
            :return vehicle: nearby walker
            :return distance: distance to nearby walker
        """

        walker_list = self._world.get_actors().filter("*walker.pedestrian*")
        def dist(w): return w.get_location().distance(waypoint.transform.location)
        walker_list = [w for w in walker_list if dist(w) < 10]

        if self._direction == RoadOption.CHANGELANELEFT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=-1)
        elif self._direction == RoadOption.CHANGELANERIGHT:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=90, lane_offset=1)
        else:
            walker_state, walker, distance = self._vehicle_obstacle_detected(walker_list, max(
                self._behavior.min_proximity_threshold, self._speed_limit / 3), up_angle_th=90)

        return walker_state, walker, distance

    def run_step(self, debug=False):
        """
        This method executes the control for the agent based on the current world state.

            :param debug: boolean flag to enable debug information
        """
        self._update_information()

        # Manage traffic lights
        if self.traffic_light_manager():
            print("Stopping for traffic light!")
            control = carla.VehicleControl(throttle=0, brake=1)
        else:
            control = self._local_planner.run_step()

        # Handle collision and pedestrian avoidance
        waypoint = self._local_planner.get_next_waypoint()
        vehicle_state, vehicle, distance = self.collision_and_car_avoid_manager(waypoint)
        walker_state, walker, distance = self.pedestrian_avoid_manager(waypoint)

        # Adjust speed and control based on detected vehicles and pedestrians
        if vehicle_state:
            control.brake = 1.0
            control.throttle = 0.0
            print("Braking due to nearby vehicle!")
        elif walker_state:
            control.brake = 1.0
            control.throttle = 0.0
            print("Braking due to nearby pedestrian!")
        else:
            control.throttle = self._behavior.max_throttle
            control.brake = 0.0

        # Publish vehicle control command to ROS
        self.control_pub.publish(control)

        return control
