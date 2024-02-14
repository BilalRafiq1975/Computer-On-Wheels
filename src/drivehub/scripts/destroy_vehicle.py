import rospy
from carla_msgs.srv import DestroyObject, DestroyObjectRequest


def get_object_id():
    while True:
        try:
            actor_id = int(input("Enter actor ID: "))
            return actor_id
        except ValueError:
            print("Invalid input. Please enter a valid integer.")


def destroy_vehicle(actor_id):
    rospy.wait_for_service('/carla/destroy_object')
    try:
        destroy_actor = rospy.ServiceProxy('/carla/destroy_object', DestroyObject)
        request = DestroyObjectRequest()
        request.id = actor_id
        response = destroy_actor(request.id)
        rospy.loginfo(f"Destroyed actor with ID: {actor_id}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    rospy.init_node('carla_destroy_vehicles', anonymous=True, log_level=rospy.DEBUG)

    actor_id = get_object_id()
    destroy_vehicle(actor_id)

    # Keep the script running
    rospy.spin()
