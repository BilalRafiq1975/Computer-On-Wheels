#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    rospy.loginfo("GPS Data:\nLatitude: %f\nLongitude: %f\nAltitude: %f",
                  data.latitude,
                  data.longitude,
                  data.altitude)

def gps_listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_listener()
