import carla


def get_s_and_t(waypoints, vehicle_position):
    closest_waypoint = min(waypoints, key=lambda waypoint: waypoint.transform.location.distance(vehicle_position))
    s = closest_waypoint.s  
    t = closest_waypoint.transform.location  
    return s, t


client = carla.Client('localhost', 2000)  
client.set_timeout(5.0)
world = client.get_world()
map = world.get_map()

waypoints = map.generate_waypoints(distance=1.0)
vehicle_position = carla.Location(x=-110, y=-91.44, z=10.0)
s, t = get_s_and_t(waypoints, vehicle_position)
print("Calculated s:", s)
print("Calculated t:", t)
