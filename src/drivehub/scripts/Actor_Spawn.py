#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
import rospy
import carla_common.transforms as trans

from carla_ros_bridge.pseudo_actor import PseudoActor

from geometry_msgs.msg import TransformStamped  # pylint: disable=import-error


class Actor(PseudoActor):


    def __init__(self, uid, name, parent, node, carla_actor):

        super(Actor, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    node=node)
        self.carla_actor = carla_actor
        self.carla_actor_id = carla_actor.id

    def destroy(self):

        self.carla_actor = None
        super(Actor, self).destroy()

    def get_current_ros_pose(self):

        return trans.carla_transform_to_ros_pose(
            self.carla_actor.get_transform())

    def get_current_ros_transform(self):
 
        return trans.carla_transform_to_ros_transform(
            self.carla_actor.get_transform())

    def get_current_ros_twist_rotated(self):
 
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

    def get_current_ros_twist(self):
 
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity())

    def get_current_ros_accel(self):

        return trans.carla_acceleration_to_ros_accel(
            self.carla_actor.get_acceleration())

    def get_id(self):
 
        return self.carla_actor_id
