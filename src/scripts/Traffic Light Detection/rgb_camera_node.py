import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import carla
import time

class RGBCameraNode:
    def __init__(self):
        # Initialize ROS Node
        rospy.init_node('rgb_camera_node', anonymous=True)

        # Define parameters
        self.camera_topic = '/carla/ego_vehicle/rgb_camera/image_raw'

        # Create ROS Publisher
        self.image_pub = rospy.Publisher(self.camera_topic, Image, queue_size=10)
        
        # Create a bridge to convert OpenCV images to ROS format
        self.bridge = CvBridge()

        # Connect to CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.camera = self.spawn_camera()

    def spawn_camera(self):
        # Find the vehicle and spawn the RGB camera
        vehicle = self.world.get_actors().filter('vehicle.*')[0]  # Assuming there is at least one vehicle
        transform = vehicle.get_transform()

        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=0.7),  # Camera position
            carla.Rotation(pitch=0, yaw=0, roll=0)
        )
        
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        
        # Set a callback for when the camera captures an image
        camera.listen(self.process_image)

        return camera

    def process_image(self, image):
        # Convert CARLA image to ROS Image and publish
        ros_image = self.bridge.cv2_to_imgmsg(image.raw_data, "bgr8")
        self.image_pub.publish(ros_image)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rgb_camera_node = RGBCameraNode()
        rgb_camera_node.run()
    except rospy.ROSInterruptException:
        pass
