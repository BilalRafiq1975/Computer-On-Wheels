def _tailgating(self, waypoint, vehicle_list):
    """
    This method is in charge of tailgating behaviors.

        :param location: current location of the agent
        :param waypoint: current waypoint of the agent
        :param vehicle_list: list of all the nearby vehicles
    """

    left_turn = waypoint.left_lane_marking.lane_change
    right_turn = waypoint.right_lane_marking.lane_change

    left_wpt = waypoint.get_left_lane()
    right_wpt = waypoint.get_right_lane()

    behind_vehicle_state, behind_vehicle, _ = self._vehicle_obstacle_detected(vehicle_list, max(
        self._behavior.min_proximity_threshold, self._speed_limit / 2), up_angle_th=180, low_angle_th=160)
    if behind_vehicle_state and self._speed < get_speed(behind_vehicle):
        if (right_turn == carla.LaneChange.Right or right_turn ==
                carla.LaneChange.Both) and waypoint.lane_id * right_wpt.lane_id > 0 and right_wpt.lane_type == carla.LaneType.Driving:
            return 1  # Right shift
        elif (left_turn == carla.LaneChange.Left or left_turn ==
              carla.LaneChange.Both) and waypoint.lane_id * left_wpt.lane_id > 0 and left_wpt.lane_type == carla.LaneType.Driving:
            return -1  # Left shift
        else:
            return 0
    else:
        return 0
