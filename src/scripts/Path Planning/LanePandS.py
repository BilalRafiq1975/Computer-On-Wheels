import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

def parse_xml(xml_string):
    root = ET.fromstring(xml_string)
    roads_info = {}
    for road in root.findall('.//road'):
        road_name = road.attrib['name']
        lanes_info = {}
        for lane_section in road.findall('.//laneSection'):
            for lane in lane_section.findall('.//lane'):
                lane_id = int(lane.attrib['id'])
                lane_type = lane.attrib['type']
                if lane_type == 'driving':
                    predecessor = lane.find('.//predecessor')
                    successor = lane.find('.//successor')
                    predecessor_id = int(predecessor.attrib['id']) if predecessor is not None else None
                    successor_id = int(successor.attrib['id']) if successor is not None else None
                    lanes_info[lane_id] = {'predecessor': predecessor_id, 'successor': successor_id}
        roads_info[road_name] = lanes_info
    return roads_info

def world_info_callback(msg):
    global opendrive_data
    if msg.opendrive:
        opendrive_data = msg.opendrive
        roads_info = parse_xml(opendrive_data)
        num_roads = len(roads_info)
        print(f"There are {num_roads} roads:")
        for road_name, lanes_info in roads_info.items():
            num_driving_lanes = len(lanes_info)
            print("--------------------------------------------------------------")
            print(f"  Road {road_name}:")
            print(f"    There are {num_driving_lanes} driving lanes in this road:")
            for lane_id, info in lanes_info.items():
                print(f"      Lane {lane_id}: Predecessor - {info['predecessor']}, Successor - {info['successor']}")

def main():
    rospy.init_node('lane_info_node', anonymous=True)
    rospy.Subscriber("/carla/world_info", CarlaWorldInfo, world_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

