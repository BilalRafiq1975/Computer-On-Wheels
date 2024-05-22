import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

def world_info_callback(world_info):
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive

    rospy.loginfo("Received map name: %s", map_name)

    # Parse the OpenDRIVE XML and extract road information
    road_info = extract_road_info(opendrive_xml)
    if road_info:
        for road_name, geometries in road_info.items():
            rospy.loginfo(f"Road Name: {road_name}")
            for idx, geometry in enumerate(geometries, start=1):
                rospy.loginfo(f"Segment {idx} Heading: {geometry['hdg']} radians")
            rospy.loginfo("\n")

def extract_road_info(opendrive_xml):
    road_info = {}
    try:
        root = ET.fromstring(opendrive_xml)

        # Extract road information
        for road in root.findall('.//road'):
            road_name = road.get('name')
            geometries = extract_geometries(road)
            road_info[road_name] = geometries

        return road_info
    except Exception as e:
        rospy.logerr("Error extracting road information: %s", str(e))
        return None

def extract_geometries(road):
    geometries = []
    plan_view = road.find('planView')
    for geometry in plan_view.findall('geometry'):
        hdg = float(geometry.get('hdg'))
        geometries.append({'hdg': hdg})
    return geometries

if __name__ == '__main__':
    try:
        rospy.init_node('road_info_node', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
