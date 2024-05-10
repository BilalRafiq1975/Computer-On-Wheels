import heapq

def dijkstra(graph, start, end):
    # Initialize distances with infinity for all nodes except the start node
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    
    # Priority queue to store nodes with their tentative distances
    priority_queue = [(0, start)]
    
    while priority_queue:
        # Pop the node with the smallest tentative distance
        current_distance, current_node = heapq.heappop(priority_queue)
        
        # If we've reached the end node, return the shortest path and its length
        if current_node == end:
            path = [current_node]
            while current_node != start:
                current_node = previous_nodes[current_node]
                path.append(current_node)
            return path[::-1], distances[end]
        
        # Explore neighbors of the current node
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            
            # If the new distance is shorter, update it and push to the priority queue
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                # Keep track of previous node to reconstruct the shortest path later
                previous_nodes[neighbor] = current_node
    
    # If no path from start to end is found
    return [], float('inf')

def update_graph(graph, obstacle_node):
    # Remove the obstacle node and its connections from the graph
    if obstacle_node in graph:
        del graph[obstacle_node]
    for node in graph:
        if obstacle_node in graph[node]:
            del graph[node][obstacle_node]

# Sample function to simulate obstacle detection and dynamic replanning
def simulate_obstacle_detection(graph):
    # Simulate obstacle detection, for example, at node '468'
    obstacle_node = '468'
    if obstacle_node in graph:
        update_graph(graph, obstacle_node)

# Sample usage
graph = { '0': {'10': 120.40156041807725, '3': 4.908388077782888},
    '10': {'0': 120.40156041807725, '17': 12.731014854747066},
    '3': {'0': 4.908388077782888, '532': 4.908388077782888, '565': 46.639999999999986, '566': 46.639999999999986, '630': 44.91265126400228, '637': 43.028126098808016, '655': 31.11973961824869},
    '1': {'664': 119.92999999999999, '8': 19.54185096138824, '675': 29.67, '676': 29.67, '712': 24.87238160049144},
    '664': {'1': 119.92999999999999, '2': 11.080000000000013, '11': 27.93},
    '8': {'1': 19.54185096138824, '4': 19.54185096138824},
    '2': {'532': 11.080000000000013, '664': 11.080000000000013, '565': 46.639999999999986, '566': 46.639999999999986, '579': 28.06750128613906, '584': 42.30119336443461, '597': 38.20871575233895, '608': 30.213976452589996, '675': 29.67, '676': 29.67, '703': 24.687772365962925},
    '532': {'2': 11.080000000000013, '3': 4.908388077782888, '21': 25.629999999999995},
    '4': {'468': 28.37, '8': 19.54185096138824, '476': 25.354372803315385, '486': 28.469289576609714, '515': 43.190000000000005, '516': 43.190000000000005},
    '468': {'4': 28.37, '5': 106.75, '13': 32.31999999999999},
    '5': {'719': 106.75, '468': 106.75, '469': 37.20111984503602, '472': 30.336091672116087, '515': 43.190000000000005, '516': 43.190000000000005, '735': 46.14999999999999, '736': 46.14999999999999, '763': 27.938169920412342, '780': 42.431871940709996, '787': 39.23823872064808, '795': 34.99843123270912},
    '719': {'5': 106.75, '6': 85.43999999999998, '18': 10.610000000000003},
    '6': {'23': 85.43999999999998, '719': 85.43999999999998, '89': 39.32000000000005, '90': 39.32000000000005, '100': 37.02883876824461, '105': 33.92809166669153, '125': 27.684039552021584, '735': 46.14999999999999, '736': 46.14999999999999, '801': 38.67889125395628, '811': 42.86266334953319, '823': 27.081664781675535, '832': 30.911681247445287},
    '23': {'6': 85.43999999999998, '7': 22.462667512143184, '22': 26.141432368214325},
    '7': {'17': 12.731014854747066, '23': 22.462667512143184, '24': 32.788210222483364, '33': 28.840194973862175, '41': 25.146166930182897, '89': 39.32000000000005, '90': 39.32000000000005},
    '17': {'7': 12.731014854747066, '10': 12.731014854747066},
    '9': {'841': 10.649999999999999, '134': 10.649999999999999, '152': 26.673648603056403, '160': 25.39950850883562, '172': 25.193795102122813, '179': 25.476961194474615, '843': 25.356216987815433, '848': 25.27359724806887},
    '841': {'9': 10.649999999999999, '20': 55.129999999999995, '21': 25.629999999999995},
    '134': {'9': 10.649999999999999, '11': 27.93, '12': 54.16},
    '11': {'134': 27.93, '664': 27.93, '150': 30.909999999999997, '151': 30.909999999999997, '152': 26.673648603056403, '160': 25.39950850883562, '703': 24.687772365962925, '712': 24.87238160049144},
    '12': {'895': 54.16, '134': 54.16, '150': 30.909999999999997, '151': 30.909999999999997, '172': 25.193795102122813, '179': 25.476961194474615, '900': 25.613483531034355, '939': 26.48813146006281},
    '895': {'12': 54.16, '13': 32.31999999999999, '14': 9.170000000000002},
    '13': {'895': 32.31999999999999, '468': 32.31999999999999, '469': 37.20111984503602, '472': 30.336091672116087, '476': 25.354372803315385, '486': 28.469289576609714, '900': 25.613483531034355, '933': 30.92000000000001, '934': 30.92000000000001},
    '14': {'189': 9.170000000000002, '895': 9.170000000000002, '256': 36.33742725166127, '268': 27.6737190082249, '338': 35.61415467524123, '344': 25.04039054998519, '466': 44.24000000000001, '467': 44.24000000000001, '933': 30.92000000000001, '934': 30.92000000000001, '939': 26.48813146006281},
    '189': {'14': 9.170000000000002, '15': 3.11054173814469, '19': 11.240000000000002, '20': 55.129999999999995},
    '15': {'16': 12.400103507574702, '189': 3.11054173814469, '286': 26.851082602322876, '296': 36.960224721088366, '315': 39.0901128244158, '375': 35.02138070035838, '382': 39.687255439833486, '395': 25.352083113844458, '466': 44.24000000000001, '467': 44.24000000000001},
    '16': {'15': 12.400103507574702, '22': 26.141432368214325},
    '22': {'16': 26.141432368214325, '23': 26.141432368214325, '24': 32.788210222483364, '33': 28.840194973862175, '41': 25.146166930182897, '100': 37.02883876824461, '105': 33.92809166669153, '125': 27.684039552021584},
    '18': {'19': 11.240000000000002, '719': 10.610000000000003, '763': 27.938169920412342, '780': 42.431871940709996, '787': 39.23823872064808, '795': 34.99843123270912, '801': 38.67889125395628, '811': 42.86266334953319, '823': 27.081664781675535, '832': 30.911681247445287},
    '19': {'18': 11.240000000000002, '189': 11.240000000000002, '254': 38.11, '255': 38.11, '256': 36.33742725166127, '268': 27.6737190082249, '286': 26.851082602322876, '296': 36.960224721088366, '315': 39.0901128244158},
    '20': {'841': 55.129999999999995, '189': 55.129999999999995, '254': 38.11, '255': 38.11, '338': 35.61415467524123, '344': 25.04039054998519, '375': 35.02138070035838, '382': 39.687255439833486, '395': 25.352083113844458, '848': 25.27359724806887, '875': 31.0, '876': 31.0},
    '21': {'532': 25.629999999999995, '841': 25.629999999999995, '579': 28.06750128613906, '584': 42.30119336443461, '597': 38.20871575233895, '608': 30.213976452589996, '630': 44.91265126400228, '637': 43.028126098808016, '655': 31.11973961824869, '843': 25.356216987815433, '875': 31.0, '876': 31.0},
    '24': {'7': 32.788210222483364, '22': 32.788210222483364},
    '33': {'22': 28.840194973862175, '7': 28.840194973862175},
    '41': {'22': 25.146166930182897, '7': 25.146166930182897},
    '89': {'7': 39.32000000000005, '6': 39.32000000000005},
    '90': {'7': 39.32000000000005, '6': 39.32000000000005},
    '100': {'22': 37.02883876824461, '6': 37.02883876824461},
    '105': {'22': 33.92809166669153, '6': 33.92809166669153},
    '125': {'22': 27.684039552021584, '6': 27.684039552021584},
    '150': {'12': 30.909999999999997, '11': 30.909999999999997},
    '151': {'12': 30.909999999999997, '11': 30.909999999999997},
    '152': {'9': 26.673648603056403, '11': 26.673648603056403},
    '160': {'9': 25.39950850883562, '11': 25.39950850883562},
    '172': {'12': 25.193795102122813, '9': 25.193795102122813},
    '179': {'9': 25.476961194474615, '12': 25.476961194474615},
    '254': {'20': 38.11, '19': 38.11},
    '255': {'20': 38.11, '19': 38.11},
    '256': {'19': 36.33742725166127, '14': 36.33742725166127},
    '268': {'14': 27.6737190082249, '19': 27.6737190082249},
    '286': {'15': 26.851082602322876, '19': 26.851082602322876},
    '296': {'15': 36.960224721088366, '19': 36.960224721088366},
    '315': {'15': 39.0901128244158, '19': 39.0901128244158},
    '338': {'20': 35.61415467524123, '14': 35.61415467524123},
    '344': {'20': 25.04039054998519, '14': 25.04039054998519},
    '375': {'15': 35.02138070035838, '20': 35.02138070035838},
    '382': {'15': 39.687255439833486, '20': 39.687255439833486},
    '395': {'20': 25.352083113844458, '15': 25.352083113844458},
    '466': {'15': 44.24000000000001, '14': 44.24000000000001},
    '467': {'15': 44.24000000000001, '14': 44.24000000000001},
    '469': {'13': 37.20111984503602, '5': 37.20111984503602},
    '472': {'13': 30.336091672116087, '5': 30.336091672116087},
    '476': {'13': 25.354372803315385, '4': 25.354372803315385},
    '486': {'13': 28.469289576609714, '4': 28.469289576609714},
    '515': {'5': 43.190000000000005, '4': 43.190000000000005},
    '516': {'5': 43.190000000000005, '4': 43.190000000000005},
    '565': {'3': 46.639999999999986, '2': 46.639999999999986},
    '566': {'3': 46.639999999999986, '2': 46.639999999999986},
    '579': {'2': 28.06750128613906, '21': 28.06750128613906},
    '584': {'21': 42.30119336443461, '2': 42.30119336443461},
    '597': {'21': 38.20871575233895, '2': 38.20871575233895},
    '608': {'2': 30.213976452589996, '21': 30.213976452589996},
    '630': {'3': 44.91265126400228, '21': 44.91265126400228},
    '637': {'3': 43.028126098808016, '21': 43.028126098808016},
    '655': {'3': 31.11973961824869, '21': 31.11973961824869},
    '675': {'2': 29.67, '1': 29.67},
    '676': {'2': 29.67, '1': 29.67},
    '703': {'2': 24.687772365962925, '11': 24.687772365962925},
    '712': {'11': 24.87238160049144, '1': 24.87238160049144},
    '735': {'6': 46.14999999999999, '5': 46.14999999999999},
    '736': {'6': 46.14999999999999, '5': 46.14999999999999},
    '763': {'18': 27.938169920412342, '5': 27.938169920412342},
    '780': {'18': 42.431871940709996, '5': 42.431871940709996},
    '787': {'18': 39.23823872064808, '5': 39.23823872064808},
    '795': {'18': 34.99843123270912, '5': 34.99843123270912},
    '801': {'6': 38.67889125395628, '18': 38.67889125395628},
    '811': {'6': 42.86266334953319, '18': 42.86266334953319},
    '823': {'18': 27.081664781675535, '6': 27.081664781675535},
    '832': {'18': 30.911681247445287, '6': 30.911681247445287},
    '843': {'21': 25.356216987815433, '9': 25.356216987815433},
    '848': {'9': 25.27359724806887, '20': 25.27359724806887},
    '875': {'21': 31.0, '20': 31.0},
    '876': {'21': 31.0, '20': 31.0},
    '900': {'12': 25.613483531034355, '13': 25.613483531034355},
    '933': {'14': 30.92000000000001, '13': 30.92000000000001},
    '934': {'14': 30.92000000000001, '13': 30.92000000000001},
    '939': {'14': 26.48813146006281, '12': 26.48813146006281},
      } # Your graph definition

start_node = '5'
end_node = '933'
previous_nodes = {}

# Initial path planning
shortest_path, shortest_distance = dijkstra(graph, start_node, end_node)
if shortest_path:
    print("Initial Shortest path from", start_node, "to", end_node, ":")
    print(shortest_path)
    print("Initial Shortest distance:", shortest_distance)
else:
    print("No path found from", start_node, "to", end_node)

# Simulate obstacle detection and dynamic replanning
simulate_obstacle_detection(graph)

# Replan the path after obstacle detection
#shortest_path, shortest_distance = dijkstra(graph, start_node, end_node)
#if shortest_path:
 #   print("\nReplanned Shortest path from", start_node, "to", end_node, ":")
  #  print(shortest_path)
   # print("Replanned Shortest distance:", shortest_distance)
#else:
 #   print("No path found from", start_node, "to", end_node)
