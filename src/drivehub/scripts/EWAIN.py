import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
<<<<<<< Updated upstream
import threading
import math

# Global variables to store map name and opendrive XML
map_name = None
opendrive_xml = None
opendrive_data = None
total_roads = 0
total_lanes = 0
roads_data = {}  # Dictionary to store road data


def world_info_callback(world_info):
    global map_name, opendrive_xml, opendrive_data, total_roads, total_lanes
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive

    rospy.loginfo("Received map name: %s", map_name)
    # rospy.loginfo("Received OpenDRIVE XML:\n%s", opendrive_xml)

    # Parse the OpenDRIVE XML and store the data
    opendrive_data = parse_opendrive(opendrive_xml)
    total_roads, total_lanes = count_roads_and_lanes(opendrive_xml)


def parse_opendrive(opendrive_xml):
    try:
        root = ET.fromstring(opendrive_xml)

        # Count the number of road elements
        road_elements = root.findall('.//road')
        road_count = len(road_elements)

        rospy.loginfo("Total roads in OpenDRIVE map: %d" % road_count)

        # Extract start and end coordinates for each road and store in roads_data
        for road in road_elements:
            road_name = road.get('name')
            start_coord, end_coord = get_start_and_end_coordinates(road)
            if start_coord and end_coord:
                road_id = road.get('id')
                roads_data[road_id] = {'name': road_name, 'start_coord': start_coord, 'end_coord': end_coord}

                # print(f"Road ID: {road_id}")
                # print(f"Road Name: {road_name}")
                # print(f"Start Coordinates: {start_coord}")
                # print(f"End Coordinates: {end_coord}")
                # print("\n")

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
        # print("Total roads in OpenDRIVE map: %d" % self.total_roads)
        # print("Total lanes in OpenDRIVE map: %d" % self.total_lanes)

    def run(self):
        rospy.spin()

    def get_road_for_coordinates(self, x, y):
        min_distance = float('inf')
        closest_road = None
        for road_id, road_data in roads_data.items():
            start_x, start_y = road_data['start_coord']
            end_x, end_y = road_data['end_coord']
            distance = distance_to_line(x, y, start_x, start_y, end_x, end_y)
            if distance < min_distance:
                min_distance = distance
                closest_road = road_id
        return closest_road, roads_data.get(closest_road, {}).get('name', None)


def distance_to_line(x, y, x1, y1, x2, y2):
    # Calculate the distance from point (x, y) to the line segment defined by (x1, y1) and (x2, y2)
    dx = x2 - x1
    dy = y2 - y1
    if dx == dy == 0:  # The segment is just a point
        return math.hypot(x - x1, y - y1)

    # Calculate the t that minimizes the distance
    t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)

    # See if this represents one of the segment's end points or a point in the middle.
    if t < 0:
        closest_x, closest_y = x1, y1
    elif t > 1:
        closest_x, closest_y = x2, y2
    else:
        closest_x, closest_y = x1 + t * dx, y1 + t * dy

    # Calculate the distance from (x, y) to the closest point on the segment
    return math.hypot(x - closest_x, y - closest_y)


def input_thread():
    global road_counter
    while True:
        x = float(input("Enter x coordinate: "))
        y = float(input("Enter y coordinate: "))
        road_id, road_name = road_counter.get_road_for_coordinates(x, y)
        if road_id:
            print(f"The coordinates ({x}, {y}) are closest to Road ID: {road_id}, Name: {road_name}")
        else:
            print("No road found for the given coordinates.")


if __name__ == '__main__':
    try:
        rospy.init_node('get_map_info_node', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
        road_counter = RoadCounter()

        input_thread = threading.Thread(target=input_thread)
        input_thread.daemon = True
        input_thread.start()

        road_counter.run()  # Use the run method of RoadCounter instead of rospy.spin()

    except rospy.ROSInterruptException:
        pass
=======
import networkx as nx
import matplotlib.pyplot as plt
import scipy as sp

class RoadCounter:
    def __init__(self):
        self.map_graph = nx.Graph()

    def world_info_callback(self, world_info):
        self.parse_opendrive(world_info.opendrive)
        self.visualize_graph()

    def parse_opendrive(self, opendrive_xml):
        try:
            root = ET.fromstring(opendrive_xml)

            for road in root.findall('.//road'):
                self.check_driving_lanes(road)

        except Exception as e:
            rospy.logerr("Error parsing OpenDRIVE XML: %s", str(e))

    def check_driving_lanes(self, road_element):
        is_driving = False
        try:
            lane_sections = road_element.findall("./lanes/laneSection")
            for lane_section in lane_sections:
                left_lanes = lane_section.findall("./left/lane")
                center_lanes = lane_section.findall("./center/lane")
                right_lanes = lane_section.findall("./right/lane")
                
                for lane in left_lanes + center_lanes + right_lanes:
                    if lane.get('type') == 'driving':
                        is_driving = True
                        break

                if is_driving:
                    rospy.loginfo(f"Road '{road_element.get('name')}' has driving lane.")
                    
                    self.add_road_to_graph(road_element)
                    break

            if not is_driving:
                rospy.loginfo(f"The road '{road_element.get('name')}' has no driving lane.")
        
        except Exception as e:
            rospy.logerr("Error checking driving lanes: %s", str(e))

    def add_road_to_graph(self, road_element):
        try:
            road_id = road_element.get('id')
            length = 0.0
            for geometry in road_element.findall("./planView/geometry"):
                length += float(geometry.get('length'))
            self.map_graph.add_node(road_id, length=length)
            
            successor_id = road_element.find("./link/successor").get("elementId")
            predecessor_id = road_element.find("./link/predecessor").get("elementId")
            
            if successor_id:
                self.map_graph.add_edge(road_id, successor_id, weight=length)
            if predecessor_id:
                self.map_graph.add_edge(road_id, predecessor_id, weight=length)
                
        except Exception as e:
            rospy.logerr("Error adding road to graph: %s", str(e))

    def visualize_graph(self):
        pos = nx.spring_layout(self.map_graph)  # Use Spring layout
        plt.figure(figsize=(12, 8))  # Increase plot size
        nx.draw(self.map_graph, pos, with_labels=True, font_weight='bold', node_size=100, font_size=8)  # Increase node and label size
        labels = nx.get_edge_attributes(self.map_graph, 'weight')
        formatted_labels = {edge: "{:.2f}".format(weight) for edge, weight in labels.items()}
        nx.draw_networkx_edge_labels(self.map_graph, pos, edge_labels=formatted_labels)
        plt.title("Road Network Graph")
        plt.show()


def main():
    rospy.init_node('road_graph_builder')
    road_counter = RoadCounter()
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, road_counter.world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
>>>>>>> Stashed changes
