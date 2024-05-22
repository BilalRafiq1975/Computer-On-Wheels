import xml.etree.ElementTree as ET

# Parse the XML file
tree = ET.parse('Town10HD_Opt_map.xml')
root = tree.getroot()

# Dictionary to store lane connections
lane_connections = {}

# Iterate over junctions
for junction in root.findall('.//junction'):
    # Iterate over connections
    for connection in junction.findall('connection'):
        # Get IDs of incoming and connecting roads
        incoming_road_id = connection.get('incomingRoad')
        connecting_road_id = connection.get('connectingRoad')
        
        # Iterate over lane links
        for lane_link in connection.findall('laneLink'):
            # Get lane IDs
            from_lane = int(lane_link.get('from'))
            to_lane = int(lane_link.get('to'))
            
            # Construct key for current lane
            from_lane_key = f'road{incoming_road_id}_lane{abs(from_lane)}'
            to_lane_key = f'road{connecting_road_id}_lane{abs(to_lane)}'
            
            # Update lane connections dictionary
            if from_lane_key in lane_connections:
                lane_connections[from_lane_key].append(to_lane_key)
            else:
                lane_connections[from_lane_key] = [to_lane_key]

# Print lane connections
for from_lane, to_lanes in lane_connections.items():
    print(f"{from_lane} {{ {' , '.join(to_lanes)} }}")
