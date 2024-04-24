import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
import networkx as nx
import matplotlib.pyplot as plt

def parse_xml(xml_string):
    root = ET.fromstring(xml_string)
    G = nx.DiGraph()
    for road in root.findall('.//road'):
        road_name = road.attrib['name']
        road_length = float(road.attrib['length'])
        for lane_section in road.findall('.//laneSection'):
            for lane in lane_section.findall('.//lane'):
                lane_id = f"{road_name}_Lane_{lane.attrib['id']}"
                lane_type = lane.attrib['type']
                if lane_type == 'driving':
                    predecessor = lane.find('.//predecessor')
                    successor = lane.find('.//successor')
                    predecessor_id = f"{road_name}_Lane_{predecessor.attrib['id']}" if predecessor is not None else None
                    successor_id = f"{road_name}_Lane_{successor.attrib['id']}" if successor is not None else None
                    if predecessor_id is not None:
                        G.add_edge(predecessor_id, lane_id, weight=road_length)
                    if successor_id is not None:
                        G.add_edge(lane_id, successor_id, weight=road_length)
    return G

def world_info_callback(msg):
    global opendrive_data
    if msg.opendrive:
        opendrive_data = msg.opendrive
        G = parse_xml(opendrive_data)
        # Create a dictionary of edge labels with weights
        edge_labels = {(u, v): w for u, v, w in G.edges(data='weight')}
        # Print the graph with weights
        print("graph = {")
        for node in G.nodes():
            neighbors = G[node]
            neighbor_weights = {neighbor: edge_labels[(node, neighbor)] for neighbor in neighbors}
            print(f"    '{node}': {neighbor_weights},")
        print("}")
        # Visualize the graph
        pos = nx.spring_layout(G)  # Define the layout for the graph
        nx.draw(G, pos, with_labels=True, node_size=1000, node_color="skyblue", font_size=10, font_weight="bold")  # Draw the graph
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')  # Display edge labels
        plt.show()

def main():
    rospy.init_node('lane_info_node', anonymous=True)
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
