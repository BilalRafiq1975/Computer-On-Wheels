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
            print(f"Road '{road_name}' has driving lanes.")
        else:
            print(f"Road '{road_name}' does not have driving lanes.")

if __name__ == "__main__":
    main()
