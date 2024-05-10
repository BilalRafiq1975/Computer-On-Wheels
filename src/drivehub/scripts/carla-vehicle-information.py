import rospy
from carla_msgs.msg import CarlaWorldInfo
from geometry_msgs.msg import PoseStamped, TwistStamped


class VehicleStatePublisher:
    def __init__(self):
        self.vehicle_position_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.vehicle_velocity_pub = rospy.Publisher('/current_velocity', TwistStamped, queue_size=1)

    def world_info_callback(self, world_info):
        current_pose = PoseStamped()
        current_pose.header.stamp = rospy.Time.now()
        current_pose.pose.position.x = world_info.current_x
        current_pose.pose.position.y = world_info.current_y
        current_pose.pose.position.z = world_info.current_z
        current_pose.pose.orientation.x = world_info.current_orientation_x
        current_pose.pose.orientation.y = world_info.current_orientation_y
        current_pose.pose.orientation.z = world_info.current_orientation_z
        current_pose.pose.orientation.w = world_info.current_orientation_w

        current_velocity = TwistStamped()
        current_velocity.header.stamp = rospy.Time.now()
        current_velocity.twist.linear.x = world_info.current_velocity_x
        current_velocity.twist.linear.y = world_info.current_velocity_y
        current_velocity.twist.linear.z = world_info.current_velocity_z

        self.vehicle_position_pub.publish(current_pose)
        self.vehicle_velocity_pub.publish(current_velocity)


def main():
    rospy.init_node('vehicle_state_publisher')
    vehicle_state_publisher = VehicleStatePublisher()
    rospy.Subscriber('/carla/world_info', CarlaWorldInfo, vehicle_state_publisher.world_info_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
