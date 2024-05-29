import rospy
from sensor_msgs.msg import Imu, NavSatFix
import carla
import tf

def carla_imu_to_ros_imu(carla_imu):
    ros_imu = Imu()
    ros_imu.header.stamp = rospy.Time.now()
    ros_imu.header.frame_id = "base_link"
    ros_imu.orientation.x = carla_imu.orientation.x
    ros_imu.orientation.y = carla_imu.orientation.y
    ros_imu.orientation.z = carla_imu.orientation.z
    ros_imu.orientation.w = carla_imu.orientation.w
    ros_imu.linear_acceleration.x = carla_imu.accelerometer.x
    ros_imu.linear_acceleration.y = carla_imu.accelerometer.y
    ros_imu.linear_acceleration.z = carla_imu.accelerometer.z
    return ros_imu

def carla_gnss_to_ros_navsatfix(carla_gnss):
    ros_navsatfix = NavSatFix()
    ros_navsatfix.header.stamp = rospy.Time.now()
    ros_navsatfix.header.frame_id = "base_link"
    ros_navsatfix.latitude = carla_gnss.latitude
    ros_navsatfix.longitude = carla_gnss.longitude
    ros_navsatfix.altitude = carla_gnss.altitude
    return ros_navsatfix

if __name__ == '__main__':
    rospy.init_node('carla_sensors', anonymous=True)
    
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    world = client.get_world()
    bp_library = world.get_blueprint_library()

    imu_bp = bp_library.find('sensor.other.imu')
    imu_bp.set_attribute('sensor_tick', '0.01')
    imu_transform = carla.Transform(carla.Location(x=0, y=0, z=0))
    imu_sensor = world.spawn_actor(imu_bp, imu_transform)
    
    gps_bp = bp_library.find('sensor.other.gnss')
    gps_bp.set_attribute('sensor_tick', '1.0')
    gps_transform = carla.Transform(carla.Location(x=0, y=0, z=0))
    gps_sensor = world.spawn_actor(gps_bp, gps_transform)

    def imu_callback(carla_imu):
        ros_imu = carla_imu_to_ros_imu(carla_imu)
        imu_pub.publish(ros_imu)

    def gps_callback(carla_gnss):
        ros_navsatfix = carla_gnss_to_ros_navsatfix(carla_gnss)
        gps_pub.publish(ros_navsatfix)

    imu_sensor.listen(imu_callback)
    gps_sensor.listen(gps_callback)
    
    rospy.spin()
