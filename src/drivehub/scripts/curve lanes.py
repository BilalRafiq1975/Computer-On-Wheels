import math
from math import atan, sin, cos
import numpy as np
import os

home_dir = os.path.expanduser("~")
downloads_folder = os.path.join(home_dir, "Downloads")
xml_filename = "Town10HD_Opt_map.xml"
xml_path = os.path.join(downloads_folder, xml_filename)

# Print the XML path to verify it's being read correctly
print("XML Path:", xml_path)


class AxisTransformation:
    def __init__(self, x, y, x_origin, y_origin, heading, curvature, s_value):
        self.x = x
        self.y = y
        self.x_origin = x_origin
        self.y_origin = y_origin
        self.heading = heading
        self.curvature = curvature
        self.s_value = s_value

        if curvature != 0:
            self.s, self.t = self.handle_curvature(x, y, x_origin, y_origin, heading, curvature, s_value)
        else:
            self.s, self.t = x, y

    @classmethod
    def handle_curvature(cls, x, y, x_origin, y_origin, heading, curvature, s_value):
        x_curvature, y_curvature = cls.forward_transformation(x, y, x_origin, y_origin, heading, curvature)
        adjacent, opposite, angle_in_radian = cls.get_triangle_data(x_curvature, y_curvature, curvature)
        radius_of_curvature = abs(1 / curvature)
        s = abs(radius_of_curvature * angle_in_radian) + s_value
        hypotenuse = math.sqrt(abs(pow(opposite, 2)) + abs(pow(adjacent, 2)))
        if curvature > 0:
            t = radius_of_curvature - hypotenuse
        else:
            t = hypotenuse - radius_of_curvature
        return s, t

    @classmethod
    def forward_transformation(cls, x, y, x_origin, y_origin, heading, curvature):
        if curvature != 0:
            radius_of_curvature = abs(1 / curvature)
            x_prime, y_prime = cls.__axis_translation(x, y, x_origin, y_origin)
            x_double_prime, y_double_prime = cls.__axis_rotation(x_prime, y_prime, heading)
            curvature_x_origin = 0
            if curvature > 0:
                curvature_y_origin = radius_of_curvature
            else:
                curvature_y_origin = -radius_of_curvature
            x_curvature, y_curvature = cls.__axis_translation(x_double_prime, y_double_prime,curvature_x_origin, curvature_y_origin)
            s, t = x_curvature, y_curvature
            return s, t
        else:
            x_prime, y_prime = cls.__axis_translation(x, y, x_origin, y_origin)
            s, t = cls.__axis_rotation(x_prime, y_prime, heading)
            return s, t

    @classmethod
    def reverse_transformation(cls, s, t, x_origin, y_origin, heading, curvature):
        if curvature != 0:
            radius = abs(1 / curvature)
            theta = np.rad2deg(s / radius)
            if curvature > 0:
                radius = radius - t
                adjacent, opposite = cls.get_angle_in_quadrant(curvature, theta, radius)
            else:
                radius = radius + t
                adjacent, opposite = cls.get_angle_in_quadrant(curvature, theta, radius)
            radius = abs(1 / curvature)
            if curvature > 0:
                x_prime, y_prime = cls.__axis_translation(opposite, adjacent, 0, -radius)
            else:
                x_prime, y_prime = cls.__axis_translation(opposite, adjacent, 0, radius)
            x_rotated, y_rotated = cls.__axis_rotation(x_prime, y_prime, -heading)
            x, y = cls.__axis_translation(x_rotated, y_rotated, -x_origin, -y_origin)
            return x, y
        else:
            x_prime, y_prime = cls.__axis_rotation(s, t, -heading)
            x, y = cls.__axis_translation(x_prime, y_prime, -x_origin, -y_origin)
            return x, y

    @classmethod
    def get_triangle_data(cls, x_curvature, y_curvature, curvature):
        x_prime_curvature, y_prime_curvature = cls.__axis_rotation(x_curvature, y_curvature, math.radians(-90))
        adjacent = x_prime_curvature
        opposite = y_prime_curvature
        if opposite == 0:
            angle_in_radian = 0
        else:
            angle_in_radian = atan(opposite / adjacent)
            angle_in_radian = cls.normalize_angle(adjacent, opposite, angle_in_radian, curvature)
        return adjacent, opposite, angle_in_radian

    @classmethod
    
    def normalize_angle(cls, adjacent, opposite, angle_in_radian, curvature):
    # Normalize the angle to be within the range of 0 to 2*pi
     if curvature > 0:
        if adjacent < 0:
            angle_in_radian += math.pi
     else:
        if adjacent > 0:
            angle_in_radian += math.pi
     return angle_in_radian


    @classmethod
    def get_angle_in_quadrant(cls, curvature, theta, radius):
        ad, op = 0,0
        if curvature > 0:
            if theta <= 90:
                ad = -1
                op = 1
            elif 90 < theta <= 180:
                theta = 180 - theta
                ad = 1
                op = 1
            elif 180 < theta <= 270:
                theta = theta - 180
                ad = 1
                op = -1
            elif 270 < theta <= 360:
                theta = 360 - theta
                ad = -1
                op = -1
            theta = np.deg2rad(theta)
            adjacent = radius * np.cos(theta) * ad
            opposite = radius * np.sin(theta) * op
            return adjacent, opposite

        if curvature < 0:
            if theta <= 90:
                ad = 1
                op = 1
            elif 90 < theta <= 180:
                theta = 180 - theta
                ad = -1
                op = 1
            elif 180 < theta <= 270:
                theta = theta - 180
                ad = -1
                op = -1
            elif 270 < theta <= 360:
                theta = 360 - theta
                ad = 1
                op = -1
            theta = np.deg2rad(theta)
            adjacent = radius * np.cos(theta) * ad
            opposite = radius * np.sin(theta) * op
            return adjacent, opposite

    def get_boundaries(self, max_t, min_t, geometry_length, curvature):
        if curvature != 0:
            curvature_x_origin, curvature_y_origin = self.forward_transformation(self.x, self.y, self.x_origin,self.y_origin, self.heading,self.curvature)
            adjacent, opposite, point_angle_in_radian = self.get_triangle_data(curvature_x_origin, curvature_y_origin,self.curvature)

            radius_of_curvature = abs(1 / curvature)
            if curvature > 0:
                min_radius = radius_of_curvature - max_t
                max_radius = radius_of_curvature - min_t
            else:
                min_radius = radius_of_curvature + min_t
                max_radius = radius_of_curvature + max_t

            point_radius = math.sqrt(pow(opposite, 2) + pow(adjacent, 2))

            if min_radius < point_radius < max_radius:
                is_vehicle_in_circle = True
            else:
                is_vehicle_in_circle = False

            radius_of_curvature = abs(1 / curvature)
            min_angle = 0
            max_angle = geometry_length / radius_of_curvature
            if min_angle <= point_angle_in_radian <= max_angle:
                is_point_in_sector = True
            else:
                is_point_in_sector = False
            x, y = self.reverse_transformation(curvature_x_origin, curvature_y_origin, self.x_origin, self.y_origin,self.heading, self.curvature)

            if is_point_in_sector and is_vehicle_in_circle:
                is_point_on_road = True
            else:
                is_point_on_road = False
            return x, y, abs(min_radius), abs(max_radius), is_point_on_road

        else:
            s, t = self.forward_transformation(self.x, self.y, self.x_origin, self.y_origin, self.heading,self.curvature)
            rect_side_a = s, t + max_t
            rect_side_b = s, t + min_t
            s = s + geometry_length
            rect_side_c = s, t + min_t
            rect_side_d = s, t + max_t
            rect_side_a = self.reverse_transformation(rect_side_a[0], rect_side_a[1], self.x_origin, self.y_origin,self.heading, self.curvature)
            rect_side_b = self.reverse_transformation(rect_side_b[0], rect_side_b[1], self.x_origin, self.y_origin,self.heading, self.curvature)
            rect_side_c = self.reverse_transformation(rect_side_c[0], rect_side_c[1], self.x_origin, self.y_origin,self.heading, self.curvature)
            rect_side_d = self.reverse_transformation(rect_side_d[0], rect_side_d[1], self.x_origin, self.y_origin,self.heading, self.curvature)
            return rect_side_a, rect_side_b, rect_side_c, rect_side_d

    def point_within_circle(self, point_x, point_y, tolerance=0.1):
        """
        Checks if a given point lies within the circle defined by the curvature.

        :param point_x: X-coordinate of the point to check
        :param point_y: Y-coordinate of the point to check
        :param tolerance: Tolerance for checking if the point is within the circle
        :return: True if the point is within the circle, False otherwise
        """
        # Transform the given point to the curvature coordinate system
        curvature_x, curvature_y = self.forward_transformation(point_x, point_y, self.x_origin, self.y_origin, self.heading, self.curvature)

        # Calculate the radius of the circle defined by the curvature
        radius_of_curvature = abs(1 / self.curvature)

        # Calculate the distance between the given point and the center of the circle
        distance_to_center = math.sqrt((curvature_x - 0) ** 2 + (curvature_y - radius_of_curvature) ** 2)

        # Check if the distance is within the tolerance range of the radius
        return abs(distance_to_center - radius_of_curvature) <= tolerance

    @property
    def s_t_axis(self):
        return self.s, self.t

    @classmethod
    def __axis_translation(cls, x, y, x_origin, y_origin):
        x_translation = (x - x_origin)
        y_translation = (y - y_origin)
        return x_translation, y_translation

    @classmethod
    def __axis_rotation(cls, x, y, heading):
        s = x * cos(heading) + y * sin(heading)
        t = y * cos(heading) - x * sin(heading)
        if t == -0.0:
            t = 0
        if s == -0.0:
            s = 0
        return s, t

# Example usage:
# Create an instance of AxisTransformation
axis_transform = AxisTransformation(x=0, y=0, x_origin=0, y_origin=0, heading=0, curvature=0.1, s_value=0)

# Check if a point lies within the circle defined by the curvature
# Given point
point_x = 1
point_y = 1

# Curvature value
curvature = 0.1  # assuming positive curvature

# Calculate the radius of the circle
radius_of_curvature = abs(1 / curvature)

# Calculate the distance between the given point and the center of the circle
distance_to_center = math.sqrt((point_x - 0) ** 2 + (point_y - radius_of_curvature) ** 2)

# Check if the distance is less than or equal to the radius of the circle
is_within_circle = distance_to_center <= radius_of_curvature

print(f"The point ({point_x}, {point_y}) is within the circle: {is_within_circle}")

