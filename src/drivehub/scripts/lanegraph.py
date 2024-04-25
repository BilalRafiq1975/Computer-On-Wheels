import xml.etree.ElementTree as ET

def get_driving_lane_ids(road_elem):
    lane_ids = []
    for lane_section in road_elem.findall('.//laneSection'):
        for lane in lane_section.findall('.//lane'):
            if lane.attrib.get('type') == 'driving':
                lane_ids.append(lane.attrib.get('id'))
    return lane_ids

def get_lane_predecessor_successor(lane_elem):
    predecessor = lane_elem.find('.//link/predecessor')
    successor = lane_elem.find('.//link/successor')
    
    if predecessor is not None:
        predecessor_id = predecessor.attrib.get('id')
        predecessor_type = predecessor.attrib.get('elementType')
    else:
        predecessor_id = "N/A"
        predecessor_type = "N/A"
        
    if successor is not None:
        successor_id = successor.attrib.get('id')
        successor_type = successor.attrib.get('elementType')
    else:
        successor_id = "N/A"
        successor_type = "N/A"
        
    return predecessor_type, predecessor_id, successor_type, successor_id

def main():
    # Parse the XML
    tree = ET.parse('Town10HD_Opt_map.xml')  
    root = tree.getroot()

    # Iterate through each road
    for road in root.findall('road'):
        road_name = road.attrib.get('name')
        driving_lane_ids = get_driving_lane_ids(road)
        if driving_lane_ids:
            print(f"Road '{road_name}' has driving lanes:")
            
            # Find information about road's predecessor and successor
            predecessor = road.find('.//predecessor')
            successor = road.find('.//successor')
            if predecessor is not None and successor is not None:
                Roadpredecessor_type = predecessor.attrib.get('elementType')
                Roadpredecessor_id = predecessor.attrib.get('elementId')
                Roadsuccessor_type = successor.attrib.get('elementType')
                Roadsuccessor_id = successor.attrib.get('elementId')
                
                print(f"  - Predecessor: Type - {Roadpredecessor_type}, ID - {Roadpredecessor_id}")
                print(f"  - Successor: Type - {Roadsuccessor_type}, ID - {Roadsuccessor_id}")
            else:
                print("  - No information about predecessor or successor.")
                
            # Iterate through each driving lane
            for lane_section in road.findall('.//laneSection'):
                for lane in lane_section.findall('.//lane'):
                    if lane.attrib.get('type') == 'driving':
                        # Get predecessor and successor for each lane
                        predecessor_type, predecessor_id, successor_type, successor_id = get_lane_predecessor_successor(lane)
                        print(f"    - Lane ID: {lane.attrib.get('id')}")
                        print(f"      - Predecessor: ID - {predecessor_id}, type: {Roadpredecessor_type}, Id: {Roadpredecessor_id}")
                        print(f"      - Successor: ID - {successor_id}, type: {Roadsuccessor_type}, Id: {Roadsuccessor_id}")
                
        else:
            print(f"Road '{road_name}' does not have driving lanes.")

if __name__ == "__main__":
    main()

