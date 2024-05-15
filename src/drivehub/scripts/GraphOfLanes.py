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

def lane_connections():
    global opendrive_data
    if opendrive_data:
        G = parse_xml(opendrive_data)
        lane_connections = {}
        for node in G.nodes():
            predecessors = [(predecessor, edge['weight']) for predecessor, edge in G[node].items() if 'weight' in edge]
            successors = [(successor, edge['weight']) for successor, edge in G.pred[node].items() if 'weight' in edge]
            lane_connections[node] = {'predecessors': predecessors, 'successors': successors}
        return lane_connections

def world_info_callback(msg):
    global opendrive_data
    if msg.opendrive:
        opendrive_data = msg.opendrive
        lane_connections_data = lane_connections()
        print("Lane Connections:")
        for lane, connections in lane_connections_data.items():
            predecessors = [f"{predecessor} ({distance}m)" for predecessor, distance in connections['predecessors']]
            successors = [f"{successor} ({distance}m)" for successor, distance in connections['successors']]
            print(f"{lane}: predecessors {predecessors}, successors {successors}")

def main():
    rospy.init_node('lane_info_node', anonymous=True)
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    opendrive_data = None
    main()
import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
import networkx as nx

def parse_xml(xml_data):
    graph = nx.DiGraph()
    root = ET.fromstring(xml_data)

    # Iterate through roads
    for road in root.findall('road'):
        road_id = road.attrib['id']
        
        # Iterate through lanes
        for lane_section in road.findall('./lanes/laneSection'):
            for lane in lane_section.findall('./left/lane') + lane_section.findall('./center/lane') + lane_section.findall('./right/lane'):
                lane_id = lane.attrib['id']
                graph.add_node(lane_id)
                
                # Add edges based on connections
                for link in lane.findall('./link'):
                    pred_id = link.find('./predecessor').attrib.get('id')
                    succ_id = link.find('./successor').attrib.get('id')
                    if pred_id:
                        graph.add_edge(pred_id, lane_id)
                    if succ_id:
                        graph.add_edge(lane_id, succ_id)
    
    return graph

def lane_connections():
    # Dummy implementation, replace with actual logic
    return {
        'Road 0_Lane_2': {'predecessors': [('Road 0_Lane_2', 13.310253693587601)], 'successors': [('Road 0_Lane_2', 13.310253693587601)]},
        'Road 0_Lane_1': {'predecessors': [('Road 0_Lane_1', 13.310253693587601)], 'successors': [('Road 0_Lane_1', 13.310253693587601)]},
        'Road 0_Lane_-1': {'predecessors': [('Road 0_Lane_-1', 13.310253693587601)], 'successors': [('Road 0_Lane_-1', 13.310253693587601)]}
    }

def world_info_callback(msg):
    global opendrive_data
    if msg.opendrive:
        opendrive_data = msg.opendrive
        lane_connections_data = lane_connections()
        print("Lane Connections:")
        for lane, connections in lane_connections_data.items():
            predecessors = [f"{predecessor} ({distance}m)" for predecessor, distance in connections['predecessors']]
            successors = [f"{successor} ({distance}m)" for successor, distance in connections['successors']]
            print(f"{lane}: predecessors {predecessors}, successors {successors}")

def main():
    rospy.init_node('lane_info_node', anonymous=True)
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    opendrive_data = None
    main()
