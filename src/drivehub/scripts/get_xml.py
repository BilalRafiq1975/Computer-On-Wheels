#!/usr/bin/env python
import rospy
from carla_msgs.msg import CarlaWorldInfo
import os


def world_info_callback(world_info):
    map_name = world_info.map_name
    opendrive_xml = world_info.opendrive

    file_path = os.path.join("/home/maanz-ai/ahmed-workspace/src/drivehub/scripts", f"{map_name}_map.xml")

    
    with open(file_path, 'w') as xml_file:
        xml_file.write(opendrive_xml)

    rospy.loginfo(f'Successfully saved the XML file for map {map_name} at: {file_path}')


def get_map_xml():
    rospy.init_node('get_carla_map_xml', anonymous=True)
    rospy.Subscriber('/carla/world_info', CarlaWorldInfo, world_info_callback)
    
    rospy.spin()


if __name__ == '__main__':
    get_map_xml()
