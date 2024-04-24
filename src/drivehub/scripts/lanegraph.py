import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
import networkx as nx
import matplotlib.pyplot as plt

def parse_xml(xml_string):
    root = ET.fromstring(xml_string)
    G = nx.DiGraph()
    for road in root.findall('.//road'):
        for lane in road.findall('.//lane'):
            lane_type = lane.attrib.get('type', '')
            if lane_type == 'driving':
                lane_id = lane.attrib.get('id', '')
                G.add_node(f"{road.attrib['id']}_Lane_{lane_id}")
    for junction in root.findall('.//junction'):
        for connection in junction.findall('.//connection'):
            incoming_road = connection.attrib['incomingRoad']
            connecting_road = connection.attrib['connectingRoad']
            for lane_link in connection.findall('.//laneLink'):
                from_lane = lane_link.attrib['from']
                to_lane = lane_link.attrib['to']
                from_lane_id = f"{incoming_road}_Lane_{from_lane}"
                to_lane_id = f"{connecting_road}_Lane_{to_lane}"
                if G.has_node(from_lane_id) and G.has_node(to_lane_id):
                    G.add_edge(from_lane_id, to_lane_id)
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
