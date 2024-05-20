
import rospy
from geometry_msgs.msg import Twist

class VelocityController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('velocity_controller', anonymous=True)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Set control rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Target velocities
        self.target_linear_speed = 0.5  # meters per second
        self.target_angular_speed = 0.0  # radians per second
        
        # Twist message to publish
        self.twist_msg = Twist()

    def control_loop(self):
        # Main control loop
        while not rospy.is_shutdown():
            # Update twist message with target velocities
            self.twist_msg.linear.x = self.target_linear_speed
            self.twist_msg.angular.z = self.target_angular_speed
            
            # Publish twist message
            self.cmd_vel_pub.publish(self.twist_msg)
            
            # Sleep to maintain control rate
            self.rate.sleep()

    def shutdown(self):
        # Shutdown procedure to stop the robot safely
        rospy.loginfo("Stopping the velocity controller...")
        
        # Set velocities to zero
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        
        # Publish zero velocities
        self.cmd_vel_pub.publish(self.twist_msg)
        
        # Allow time for the robot to stop
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        # Create VelocityController instance
        controller = VelocityController()
        
        # Start control loop
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass

