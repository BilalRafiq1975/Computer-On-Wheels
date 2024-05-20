
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class Localization:
    def __init__(self):
        rospy.init_node('localization', anonymous=True)
        self.pose = Pose()
        self.velocity = Twist()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        
    def odometry_callback(self, msg):
        self.pose = msg.pose.pose
        self.velocity = msg.twist.twist

    def get_position(self):
        return self.pose.position

    def get_orientation(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def get_velocity(self):
        return self.velocity.linear, self.velocity.angular

    def get_pose(self):
        return self.pose

    def get_linear_velocity(self):
        return self.velocity.linear

    def get_angular_velocity(self):
        return self.velocity.angular

    def get_roll_pitch_yaw(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return roll, pitch, yaw

    def get_pose_as_dict(self):
        pose_dict = {
            'position': {
                'x': self.pose.position.x,
                'y': self.pose.position.y,
                'z': self.pose.position.z
            },
            'orientation': {
                'roll': self.get_roll_pitch_yaw()[0],
                'pitch': self.get_roll_pitch_yaw()[1],
                'yaw': self.get_roll_pitch_yaw()[2]
            }
        }
        return pose_dict

    def get_odometry_as_dict(self):
        odom_dict = {
            'pose': self.get_pose_as_dict(),
            'linear_velocity': {
                'x': self.velocity.linear.x,
                'y': self.velocity.linear.y,
                'z': self.velocity.linear.z
            },
            'angular_velocity': {
                'x': self.velocity.angular.x,
                'y': self.velocity.angular.y,
                'z': self.velocity.angular.z
            }
        }
        return odom_dict

if __name__ == '__main__':
    try:
        localization = Localization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

