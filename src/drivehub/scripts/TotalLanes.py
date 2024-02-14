import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

# Global variables to store map name and opendrive data
map_name = None
opendrive_data = None
total_roads = 0
total_lanes = 0


def world_info_callback(world_info):
    global map_name, opendrive_data, total_roads, total_lanes
    map_name = world_info.map_name
    opendrive_data = world_info.opendrive

    print("Received map name:", map_name)

    # Parse the OpenDRIVE XML and store the data
    total_roads, total_lanes = count_roads_and_lanes(opendrive_data)
    print("Total roads in OpenDRIVE map:", total_roads)
    print("Total lanes in OpenDRIVE map:", total_lanes)

    # Extract and print the start and end coordinates of each road segment
    if opendrive_data:
        road_elements = ET.fromstring(opendrive_data).findall('.//road')
        for road_element in road_elements:
            start_coordinates, end_coordinates = get_start_and_end_coordinates(road_element)
            if start_coordinates is not None and end_coordinates is not None:
                print("Start Coordinates:", start_coordinates)
                print("End Coordinates:", end_coordinates)


def extract_geometry(road_element):
    geometry = road_element.find('.//geometry')
    if geometry is not None:
        return geometry.findall('.//point')
    return []


def extract_coordinates(point_element):
    x = float(point_element.get('x', 0))
    y = float(point_element.get('y', 0))
    return x, y


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
            lane_sections = road.findall("./lanes/laneSection")
            for lane_section in lane_sections:
                left_lanes = lane_section.findall("./left/lane")
                right_lanes = lane_section.findall("./right/lane")
                total_lanes += len(left_lanes) + len(right_lanes)
        return total_roads, total_lanes
    except Exception as e:
        print("Error counting roads and lanes:", str(e))
        return 0, 0


def get_start_and_end_coordinates(road_element):
    geometry_points = road_element.findall('.//geometry/point')
    if not geometry_points:
        return None, None

    start_point = geometry_points[0]
    end_point = geometry_points[-1]

    start_x, start_y = float(start_point.get('x', 0)), float(start_point.get('y', 0))
    end_x, end_y = float(end_point.get('x', 0)), float(end_point.get('y', 0))

    return (start_x, start_y), (end_x, end_y)



class RoadCounter:
    def __init__(self):
        rospy.Subscriber("/carla/world_info", CarlaWorldInfo, self.world_info_callback)

    def world_info_callback(self, msg):
        world_info_callback(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('get_map_info_node', anonymous=True)
        road_counter = RoadCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
