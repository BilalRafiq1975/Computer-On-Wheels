import math

def global_to_local(global_point, local_origin, local_heading):
    # Translate the global point to the local origin
    translated_point = (global_point[0] - local_origin[0], global_point[1] - local_origin[1])
    
    # Rotate the translated point to align with the local coordinate system
    rotated_x = translated_point[0] * math.cos(local_heading) + translated_point[1] * math.sin(local_heading)
    rotated_y = -translated_point[0] * math.sin(local_heading) + translated_point[1] * math.cos(local_heading)
    
    return (rotated_x, rotated_y)

def local_to_global(local_point, local_origin, local_heading):
    # Rotate the local point to align with the global coordinate system
    rotated_x = local_point[0] * math.cos(-local_heading) - local_point[1] * math.sin(-local_heading)
    rotated_y = local_point[0] * math.sin(-local_heading) + local_point[1] * math.cos(-local_heading)
    
    # Translate the rotated point to the global origin
    global_point = (rotated_x + local_origin[0], rotated_y + local_origin[1])
    
    return global_point

global_point = (10, 20)  #  global point
local_origin = (5, 10)   #  local origin
local_heading = math.radians(45)  #  local heading (in radians)

# Convert global point to local
local_point = global_to_local(global_point, local_origin, local_heading)
print("Local coordinates:", local_point)

# Convert local point back to global
converted_global_point = local_to_global(local_point, local_origin, local_heading)
print("Converted back to global coordinates:", converted_global_point)
