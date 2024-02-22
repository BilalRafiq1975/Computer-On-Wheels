import rospy
from carla_msgs.msg import CarlaWorldInfo
import xml.etree.ElementTree as ET
import math

# Define constants
ROAD_NAME = "Road 763"
LANE_TYPE = "driving"


def world_info_callback(world_info):
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive
    
    rospy.loginfo("Received map name: %s", map_name)

    # Parse the OpenDRIVE XML and extract lane information for the specified road
    segments = extract_segments(opendrive_xml, ROAD_NAME, LANE_TYPE)
    if segments:
        rospy.loginfo(f"Segments on {ROAD_NAME}:")
        for segment_id, segment_data in segments.items():
            rospy.loginfo(f"Segment {segment_id}:")
            rospy.loginfo(f"S: {segment_data['s']}")
            rospy.loginfo(f"Corners: {segment_data['corners']}")


def extract_segments(opendrive_xml, target_road_name, target_lane_type):
    segments = {}
    try:
        root = ET.fromstring(opendrive_xml)

        # Find the specified road
        target_road = None
        for road in root.findall('.//road'):
            if road.get('name') == target_road_name:
                target_road = road
                break

        if target_road is None:
            rospy.logwarn(f"Road '{target_road_name}' not found.")
            return None

        segment_id = 0
        # Extract lane information for all lanes of the specified type
        for lane in target_road.findall(f"./lanes/laneSection/*/lane"):
            lane_id = int(lane.get('id'))  # Convert lane id to integer
            lane_type = lane.get('type')
            if lane_type == target_lane_type:
                rospy.loginfo(f"Found lane {lane_id} of type {lane_type}")
                width = float(lane.find('width').get('a', 0))
                rospy.loginfo(f"Lane width: {width}")
                # Extract the geometry data
                for geometry in lane.findall('../../planView/geometry'):
                    s = float(geometry.get('s', 0))
                    x = float(geometry.get('x', 0))
                    y = float(geometry.get('y', 0))
                    hdg = math.radians(float(geometry.get('hdg', 0)))  # Convert hdg to radians
                    rospy.loginfo(f"Geometry data: s={s}, x={x}, y={y}, hdg={math.degrees(hdg)}")

                    # Calculate corners of the rectangle
                    corners = calculate_rectangle_corners(x, y, hdg, width)
                    rospy.loginfo(f"Corners: {corners}")

                    # Save segment data
                    segments[segment_id] = {'s': s, 'corners': corners}
                    segment_id += 1

        return segments
    except Exception as e:
        rospy.logerr("Error extracting segment information: %s", str(e))
        return None


def calculate_rectangle_corners(x, y, hdg, width):
    # Calculate the coordinates of the four corners of the rectangle
    half_width = width / 2
    cos_heading = math.cos(hdg)
    sin_heading = math.sin(hdg)

    # Define the corners in the local coordinate system
    local_corners = [
        (-half_width, -half_width),
        (half_width, -half_width),
        (half_width, half_width),
        (-half_width, half_width)
    ]

    # Rotate and translate the corners to the global coordinate system
    global_corners = []
    for corner in local_corners:
        rotated_x = corner[0] * cos_heading - corner[1] * sin_heading
        rotated_y = corner[0] * sin_heading + corner[1] * cos_heading
        global_corners.append((x + rotated_x, y + rotated_y))

    return global_corners


if __name__ == '__main__':
    try:
        rospy.init_node('road_representation_node', anonymous=True)
        rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
