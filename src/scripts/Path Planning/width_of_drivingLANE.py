import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET

def world_info_callback(world_info):
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive

    rospy.loginfo("Received map name: %s", map_name)

    # Parse the OpenDRIVE XML and extract lane information
    lane_info = extract_lane_info(opendrive_xml)
    if lane_info:
        for road_name, road_data in lane_info.items():
            rospy.loginfo(f"Road Name: {road_name}")
            rospy.loginfo(f"Number of Driving Lanes: {road_data['num_driving_lanes']}")
            rospy.loginfo("Lane Widths:")
            for lane_id, lane_width in road_data['lane_widths'].items():
                rospy.loginfo(f"Lane {lane_id}: {lane_width} meters")
            rospy.loginfo("\n")


def extract_lane_info(opendrive_xml):
    lane_info = {}
    try:
        root = ET.fromstring(opendrive_xml)

        # Extract lane information for each road
        for road in root.findall('.//road'):
            road_name = road.get('name')
            lanes_data = {'lane_widths': {}, 'num_driving_lanes': 0}

            # Extract lane width for driving lanes
            for lane in road.findall("./lanes/laneSection/*/lane"):
                lane_id = lane.get('id')
                lane_type = lane.get('type')
                if lane_type == 'driving':
                    lane_width = float(lane.find('width').get('a', 0))
                    lanes_data['lane_widths'][lane_id] = lane_width
                    lanes_data['num_driving_lanes'] += 1
            
            lane_info[road_name] = lanes_data

        return lane_info
    except Exception as e:
        rospy.logerr("Error extracting lane information: %s", str(e))
        return None


if __name__ == '__main__':
    try:
        rospy.init_node('lane_info_node', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
