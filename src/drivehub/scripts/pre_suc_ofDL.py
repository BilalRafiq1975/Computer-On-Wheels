import xml.etree.ElementTree as ET

def has_driving_lane(road_elem):
    for lane_section in road_elem.findall('.//laneSection'):
        for lane in lane_section.findall('.//lane'):
            if lane.attrib.get('type') == 'driving':
                return True
    return False

def main():
    # Parse the XML
    tree = ET.parse('Town10HD_Opt_map.xml')  
    root = tree.getroot()

    # Iterate through each road
    for road in root.findall('road'):
        road_name = road.attrib.get('name')
        if has_driving_lane(road):
            print(f"'{road_name}' has driving lanes:")
            
            # Find predecessor and successor
            predecessor = road.find('.//predecessor')
            successor = road.find('.//successor')
            
            if predecessor is not None and successor is not None:
                predecessor_type = predecessor.attrib.get('elementType')
                predecessor_id = predecessor.attrib.get('elementId')
                successor_type = successor.attrib.get('elementType')
                successor_id = successor.attrib.get('elementId')
                
                print(f"  - Predecessor: Type - {predecessor_type}, ID - {predecessor_id}")
                print(f"  - Successor: Type - {successor_type}, ID - {successor_id}")
                print("----------------------------------------------------------------")
            else:
                print("  - No information about predecessor or successor.")
        else:
            print(f"Road '{road_name}' does not have driving lanes.")

if __name__ == "__main__":
    main()
