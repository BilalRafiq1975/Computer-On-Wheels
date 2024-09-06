class ObstacleDetector:

    def check_obstacles_using_route_bb(self):
        if self.vehicle_pose is None or self.vehicle_list is None:
            return (False, None, -1)

        ego_location = self.vehicle_pose.position
        route_bb = self.get_route_bounding_box(ego_location)

        for target_vehicle in self.vehicle_list:
            target_location = target_vehicle.position
            if self.compute_distance(target_location, ego_location) > self.max_distance:
                continue

            target_polygon = self.get_vehicle_polygon(target_vehicle)
            if self.polygon_intersects(route_bb, target_polygon):
                return (True, target_vehicle, self.compute_distance(target_location, ego_location))

        return (False, None, -1)

    def get_route_bounding_box(self, ego_location):
        # Placeholder for actual bounding box computation
        return Polygon()

    def get_vehicle_polygon(self, vehicle):
        # Placeholder for converting vehicle data to a Polygon
        return Polygon()

    def polygon_intersects(self, poly1, poly2):
        return poly1.intersects(poly2)
