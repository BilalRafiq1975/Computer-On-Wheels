
import rospy
from sensor_msgs.msg import Imu

def imu_callback(data):
    rospy.loginfo("IMU Data:\nOrientation: [%f, %f, %f, %f]\nLinear Acceleration: [%f, %f, %f]",
                  data.orientation.x,
                  data.orientation.y,
                  data.orientation.z,
                  data.orientation.w,
                  data.linear_acceleration.x,
                  data.linear_acceleration.y,
                  data.linear_acceleration.z)

def imu_listener():
    rospy.init_node('imu_listener', anonymous=True)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_listener()
