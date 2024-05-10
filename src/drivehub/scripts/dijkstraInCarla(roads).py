#!/usr/bin/env python

import heapq
import rospy
from std_msgs.msg import String
from GraphInfo import get_graph

class Dijkstra:
    def __init__(self):
        rospy.init_node('dijkstra_node', anonymous=True)
        self.graph = get_graph()  # Importing the graph from GraphInfo

    def run(self, start, end):
        # Dijkstra algorithm implementation
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0

        priority_queue = [(0, start)]
        previous_nodes = {}

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_node == end:
                path = [current_node]
                total_distance = distances[end]
                while current_node != start:
                    current_node = previous_nodes[current_node]
                    path.append(current_node)
                rospy.loginfo("Shortest path: {}".format(path[::-1]))
                rospy.loginfo("Total distance: {}".format(total_distance))
                return

            for neighbor, weight in self.graph[current_node].items():
                distance = current_distance + weight
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))
                    previous_nodes[neighbor] = current_node

        rospy.logerr("No path found from {} to {}.".format(start, end))

if __name__ == '__main__':
    try:
        dijkstra = Dijkstra()
       
        start = '5'  
        end = '933'   
        dijkstra.run(start, end)
    except rospy.ROSInterruptException:
        pass
