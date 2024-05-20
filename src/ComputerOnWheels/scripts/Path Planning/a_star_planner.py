import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaMapInfo

import heapq
import math

class AStarPlanner:
    def __init__(self):
        rospy.init_node('a_star_planner', anonymous=True)

        self.map_sub = rospy.Subscriber('/carla/occupancy_grid', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/a_star_path', Path, queue_size=10)
        
        self.map_info_sub = rospy.Subscriber('/carla/map_info', CarlaMapInfo, self.map_info_callback)
        self.map_data = None
        self.resolution = 0.05  # Example resolution, should be set to your map's resolution
        self.origin = None

    def map_info_callback(self, data):
        self.origin = data.origin

    def map_callback(self, data):
        self.map_data = data.data
        self.resolution = data.info.resolution

    def a_star(self, start, goal):
        start_node = (self.grid_index(start), 0)
        goal_node = self.grid_index(goal)

        open_set = []
        heapq.heappush(open_set, (0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self.heuristic(start_node, goal_node)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_node:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_node)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def grid_index(self, position):
        x, y = position
        grid_x = int((x - self.origin.position.x) / self.resolution)
        grid_y = int((y - self.origin.position.y) / self.resolution)
        return grid_x, grid_y

    def grid_position(self, index):
        grid_x, grid_y = index
        x = grid_x * self.resolution + self.origin.position.x
        y = grid_y * self.resolution + self.origin.position.y
        return x, y

    def get_neighbors(self, node):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (node[0] + dx, node[1] + dy)
            if self.is_valid(neighbor):
                neighbors.append(neighbor)
        return neighbors

    def is_valid(self, node):
        grid_x, grid_y = node
        if grid_x < 0 or grid_y < 0 or grid_x >= len(self.map_data[0]) or grid_y >= len(self.map_data):
            return False
        if self.map_data[grid_y * len(self.map_data[0]) + grid_x] != 0:  # Occupied cell
            return False
        return True

    def heuristic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(self.grid_position(current))
            current = came_from[current]
        path.reverse()
        return path

    def publish_path(self, path):
        if path is None:
            rospy.loginfo("No path found.")
            return

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for pos in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def run(self):
        rospy.sleep(1)  # Wait for map data to be received
        start = (10.0, 10.0)  # Example start position in meters
        goal = (50.0, 50.0)  # Example goal position in meters
        path = self.a_star(start, goal)
        self.publish_path(path)

if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass

