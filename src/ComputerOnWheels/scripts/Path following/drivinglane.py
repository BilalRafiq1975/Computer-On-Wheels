import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

# Global variables to store map name and opendrive XML
map_name = None
opendrive_xml = None
opendrive_data = None
total_roads = 0
total_lanes = 0


def world_info_callback(world_info):
    global map_name, opendrive_xml, opendrive_data, total_roads, total_lanes
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive

    rospy.loginfo("Received map name: %s", map_name)
    print("\n")

    # Parse the OpenDRIVE XML and store the data
    opendrive_data = parse_opendrive(opendrive_xml)
    total_roads, total_lanes = count_roads_and_lanes(opendrive_xml)


def parse_opendrive(opendrive_xml):
    try:
        root = ET.fromstring(opendrive_xml)

        # Count the number of road elements
        road_elements = root.findall('.//road')
        road_count = len(road_elements)

        #rospy.loginfo("Total roads in OpenDRIVE map: %d" % road_count)

        # Extract start and end coordinates for each road
        for road in road_elements:
            road_name = road.get('name')
            start_coord, end_coord = get_start_and_end_coordinates(road)
            if start_coord and end_coord:
                print("------------------------------------------------------------------------")
                rospy.loginfo(f"Road Name: {road_name}")
                rospy.loginfo(f"Start Coordinates: {start_coord}")
                rospy.loginfo(f"End Coordinates: {end_coord}")

                # Count lanes for the road and display the driving lanes
                lane_count, driving_lanes = count_lanes(road)
                rospy.loginfo(f"Number of lanes: {lane_count}")
                rospy.loginfo(f"Driving Lanes: {driving_lanes}")
                rospy.loginfo("\n")

        return {'road_count': road_count}  # Return dictionary with road count
    except Exception as e:
        rospy.logerr("Error parsing OpenDRIVE XML: %s", str(e))
        return None


def get_start_and_end_coordinates(road_element):
    geometry_points = road_element.findall('.//planView/geometry')
    if not geometry_points:
        return None, None

    start_point = geometry_points[0]
    end_point = geometry_points[-1]

    start_x, start_y = float(start_point.get('x', 0)), float(start_point.get('y', 0))
    end_x, end_y = float(end_point.get('x', 0)), float(end_point.get('y', 0))

    return (start_x, start_y), (end_x, end_y)


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


def count_roads_and_lanes(opendrive_data):
    total_roads = 0
    total_lanes = 0
    try:
        root = ET.fromstring(opendrive_data)
        # Count the number of road elements
        roads = root.findall(".//road")
        total_roads = len(roads)
        # Count the number of lanes in each road
        for road in roads:
            lanes_count, _ = count_lanes(road)
            total_lanes += lanes_count
        return total_roads, total_lanes
    except Exception as e:
        rospy.logerr("Error counting roads and lanes: %s", str(e))
        return 0, 0


def get_opendrive_data():
    global opendrive_data
    return opendrive_data


class RoadCounter:
    def __init__(self):
        self.total_roads = 0
        self.total_lanes = 0
        rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.world_info_callback)

    def world_info_callback(self, msg):
        self.total_roads, self.total_lanes = count_roads_and_lanes(msg.opendrive)
        rospy.loginfo("Total roads in OpenDRIVE map: %d" % self.total_roads)
        rospy.loginfo("Total lanes in OpenDRIVE map: %d" % self.total_lanes)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('get_map_info_node', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
        road_counter = RoadCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
