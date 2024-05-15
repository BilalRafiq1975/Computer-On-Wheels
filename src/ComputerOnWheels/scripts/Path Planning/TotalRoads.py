import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET


class RoadCounter:
    def __init__(self):
        self.total_roads = 0
        rospy.init_node('road_counter_node', anonymous=True)
        rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.world_info_callback)

    def world_info_callback(self, msg):
        self.total_roads = self.count_roads(msg.opendrive)
        rospy.loginfo("Total roads in OpenDRIVE map: %d" % self.total_roads)

    def count_roads(self, opendrive_data):
        # Parse OpenDRIVE XML data
        root = ET.fromstring(opendrive_data)
        # Count the number of road elements
        total_roads = len(root.findall(".//road"))
        return total_roads

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        road_counter = RoadCounter()
        road_counter.run()
    except rospy.ROSInterruptException:
        pass
