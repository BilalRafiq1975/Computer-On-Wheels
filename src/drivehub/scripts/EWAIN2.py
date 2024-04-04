import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

# Global variables to store map name and opendrive XML
map_name = None
opendrive_xml = None
opendrive_data = None


def parse_opendrive(opendrive_xml):
    try:
        root = ET.fromstring(opendrive_xml)

        # Extract start and end coordinates for each road
        for road in root.findall('.//road'):
            road_name = road.get('name')

            if road_name:
                print("------------------------------------------------------------------------")
                rospy.loginfo(f"Road Name: {road_name}")

                # Count lanes for the road and display the driving lanes
                lane_count, driving_lanes = count_lanes(road)
                rospy.loginfo(f"Driving Lanes: {driving_lanes}")
                rospy.loginfo("\n")

                # Find successors and predecessors for each driving lane
                for lane_id in driving_lanes:
                    successor_lane, successor_road = find_successor_lane(root, road, lane_id)
                    predecessor_lane, predecessor_road = find_predecessor_lane(root, road, lane_id)
                    rospy.loginfo(f"Successor of Lane {lane_id} on Road {road_name}: {successor_lane} on Road {successor_road}")
                    rospy.loginfo(f"Predecessor of Lane {lane_id} on Road {road_name}: {predecessor_lane} on Road {predecessor_road}")
                    rospy.loginfo("\n")

        return {'road_count': len(root.findall('.//road'))}  # Return dictionary with road count
    except Exception as e:
        rospy.logerr("Error parsing OpenDRIVE XML: %s", str(e))
        return None


def count_lanes(road_element):
    total_lanes = 0
    driving_lanes = []
    try:
        lane_sections = road_element.findall("./lanes/laneSection")
        for lane_section in lane_sections:
            left_lanes = lane_section.findall("./left/lane")
            center_lanes = lane_section.findall("./center/lane")
            right_lanes = lane_section.findall("./right/lane")
            total_lanes += len(left_lanes) + len(center_lanes) + len(right_lanes)
            for lane in left_lanes + center_lanes + right_lanes:
                if lane.get('type') == 'driving':
                    driving_lanes.append(lane.get('id'))
        return total_lanes, driving_lanes
    except Exception as e:
        rospy.logerr("Error counting lanes: %s", str(e))
        return 0, []


def find_successor_lane(root, road, lane_id):
    try:
        successor = road.find(f"./lanes/laneSection/*/*[@id='{lane_id}']/link/successor")
        if successor is not None and successor.get('elementId') is not None:
            successor_road = root.find(f".//*[@id='{successor.get('elementId')}']")
            if successor_road is not None:
                successor_road_name = successor_road.get('name')
                return successor.get('elementId'), successor_road_name
        return "None", "None"
    except Exception as e:
        rospy.logerr("Error finding successor: %s", str(e))
        return "None", "None"


def find_predecessor_lane(root, road, lane_id):
    try:
        predecessor = road.find(f"./lanes/laneSection/*/*[@id='{lane_id}']/link/predecessor")
        if predecessor is not None and predecessor.get('elementId') is not None:
            predecessor_road = root.find(f".//*[@id='{predecessor.get('elementId')}']")
            if predecessor_road is not None:
                predecessor_road_name = predecessor_road.get('name')
                return predecessor.get('elementfId'), predecessor_road_name
        return "None", "None"
    except Exception as e:
        rospy.logerr("Error finding predecessor: %s", str(e))
        return "None", "None"


class RoadCounter:
    def __init__(self):
        rospy.init_node('road_counter', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, self.world_info_callback)

    def world_info_callback(self, msg):
        global opendrive_data
        if msg.opendrive:
            opendrive_data = msg.opendrive
            parse_opendrive(opendrive_data)


if __name__ == '__main__':
    try:
        counter = RoadCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
