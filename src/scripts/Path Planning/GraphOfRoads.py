import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
import networkx as nx

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
                    self.add_road_to_graph(road_element)
                    break
        
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
        graph_dict = {}

        for node in self.map_graph.nodes():
            neighbors = {}
            for neighbor, attrs in self.map_graph[node].items():
                neighbors[neighbor] = attrs['weight']
            graph_dict[node] = neighbors

        print("graph = {")
        for node, neighbors in graph_dict.items():
            print(f"    '{node}': {neighbors},")
        print("}")


def main():
    rospy.init_node('road_graph_builder')
    road_counter = RoadCounter()
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, road_counter.world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
