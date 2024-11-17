import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class AckermannSteeringNode:
    def __init__(self):
        """
        Initialize the Ackermann Steering ROS Node.
        """
        rospy.init_node("ackermann_steering_node", anonymous=True)

        # Vehicle parameters (from ROS parameter server or defaults)
        self.wheelbase = rospy.get_param("~wheelbase", 2.5)  # meters
        self.track_width = rospy.get_param("~track_width", 1.5)  # meters
        self.k_p = rospy.get_param("~proportional_gain", 0.5)  # Proportional control gain

        # Publishers for inner and outer steering angles
        self.inner_angle_pub = rospy.Publisher("/ackermann/inner_wheel_angle", Float64, queue_size=10)
        self.outer_angle_pub = rospy.Publisher("/ackermann/outer_wheel_angle", Float64, queue_size=10)

        # Subscriber for the desired turning radius
        self.turning_radius_sub = rospy.Subscriber("/ackermann/turning_radius", Float64, self.turning_radius_callback)

        # Subscriber for the current steering angle (optional feedback loop)
        self.current_angle_sub = rospy.Subscriber("/ackermann/current_angle", Float64, self.current_angle_callback)

        self.current_steering_angle = 0.0  # Initialize current angle

        rospy.loginfo("Ackermann Steering Node Initialized")

    def calculate_steering_angles(self, turning_radius: float) -> tuple:
        """
        Calculate inner and outer steering angles based on the Ackermann geometry.
        
        :param turning_radius: Desired turning radius (in meters).
        :return: Tuple of (inner_angle, outer_angle) in radians.
        """
        if turning_radius < self.track_width / 2:
            rospy.logwarn("Turning radius must be larger than half the track width.")
            return 0.0, 0.0

        # Inside and outside wheel radii
        R_inner = turning_radius - self.track_width / 2
        R_outer = turning_radius + self.track_width / 2

        # Calculate steering angles
        delta_inner = math.atan(self.wheelbase / R_inner)
        delta_outer = math.atan(self.wheelbase / R_outer)

        return delta_inner, delta_outer

    def proportional_control(self, desired_angle: float, current_angle: float) -> float:
        """
        Apply proportional control to adjust the steering angle.
        
        :param desired_angle: Desired steering angle (in radians).
        :param current_angle: Current steering angle (in radians).
        :return: Corrected steering angle (in radians).
        """
        error = desired_angle - current_angle
        return self.k_p * error

    def turning_radius_callback(self, msg: Float64):
        """
        Callback for the desired turning radius.
        
        :param msg: ROS message containing the turning radius (in meters).
        """
        turning_radius = msg.data
        inner_angle, outer_angle = self.calculate_steering_angles(turning_radius)

        # Apply proportional control for the inner wheel (optional)
        corrected_inner_angle = self.proportional_control(inner_angle, self.current_steering_angle)

        # Publish the calculated angles
        self.inner_angle_pub.publish(corrected_inner_angle)
        self.outer_angle_pub.publish(outer_angle)

        rospy.loginfo(f"Published Inner Angle: {math.degrees(corrected_inner_angle):.2f}°")
        rospy.loginfo(f"Published Outer Angle: {math.degrees(outer_angle):.2f}°")

    def current_angle_callback(self, msg: Float64):
        """
        Callback for the current steering angle.
        
        :param msg: ROS message containing the current steering angle (in radians).
        """
        self.current_steering_angle = msg.data

def main():
    try:
        ackermann_steering_node = AckermannSteeringNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Ackermann Steering Node Terminated")

if __name__ == "__main__":
    main()
