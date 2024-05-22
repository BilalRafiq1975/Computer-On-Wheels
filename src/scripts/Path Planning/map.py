import rospy
from carla_msgs.msg import CarlaWorldInfo


def world_info_callback(world_info):
    map_name = world_info.map_name
    rospy.loginfo("Current map name: %s", map_name)


if __name__ == '__main__':
    rospy.init_node('get_map_name_node', anonymous=True)

    # Subscribe to the /carla/world_info topic
    rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)

    # Keep node running to receive messages
    rospy.spin()
