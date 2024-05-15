
import os
import rospy
import ros_compatibility as roscomp

from carla_msgs.msg import CarlaStatus  # pylint: disable=import-error

class CarlaStatusPublisher(object):

    def __init__(self, synchronous_mode, fixed_delta_seconds, node):
        """
        Constructor

        """
        self.synchronous_mode = synchronous_mode
        self.synchronous_mode_running = True
        self.fixed_delta_seconds = fixed_delta_seconds
        self.node = node
        if self.fixed_delta_seconds is None:
            self.fixed_delta_seconds = 0.
        self.frame = 0

        callback_group = roscomp.callback_groups.ReentrantCallbackGroup()
        self.carla_status_publisher = self.node.new_publisher(CarlaStatus, "/carla/status", qos_profile=10,
                                                              callback_group=callback_group)
        self.publish()

    def destroy(self):
        self.node.destroy_publisher(self.carla_status_publisher)

    def publish(self):

        status_msg = CarlaStatus()
        status_msg.frame = self.frame
        status_msg.synchronous_mode = self.synchronous_mode
        status_msg.synchronous_mode_running = self.synchronous_mode_running
        status_msg.fixed_delta_seconds = self.fixed_delta_seconds
        self.carla_status_publisher.publish(status_msg)

    def set_synchronous_mode_running(self, running):

        if self.synchronous_mode_running != running:
            self.synchronous_mode_running = running
            self.publish()

    def set_frame(self, frame):

        if self.frame != frame:
            self.frame = frame
            self.publish()
