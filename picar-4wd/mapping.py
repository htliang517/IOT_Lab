import numpy as np
import math, yaml
import time
import picar_4wd as fc
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
import datetime


# Map Size
# [Option1] Read from config.yaml
with open('config.yaml', 'r') as file:
    try:
        config = yaml.safe_load(file)
    except Exception as e:
        print(f"Error with loading config.yaml: {e}")
MAP_SIZE = config['mapping']['mapSize']
REAL_WORLD_SIZE = config['mapping']['realSize']
# [Option2] Hardcoded
# MAP_SIZE = 100

# Map Initialization
map_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
prev_map_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)

# initialize car state, theta is the current angle of the car
# ~ vehicle_x, vehicle_y, vehicle_theta = 50, 0, 0
# vehicle_x, vehicle_y = 50, 0

# (-60 sensor is facing right)
SCAN_ANGLES = np.arange(-60, 61, 5)

"""sample 5 times, take median"""
def get_distance_median(angle, num_samples=5):
    distances = []
    for _ in range(num_samples):
        distance = fc.get_distance_at(angle)
        if 0 < distance < 100:  
            distances.append(distance)
        time.sleep(0.01)  
    
    return np.median(distances) if distances else None  

# Get the disatnce from obstacle to the car, angle is the sensor angle
def offsetXY(distance, vehicleX, vehicleY, vehicle_theta, angle):  
    total_angle_rad = math.radians(vehicle_theta + angle)
    GRID_SIZE = REAL_WORLD_SIZE / MAP_SIZE
    # 11.4 cm = 1 grid, convert cm to grid unit
    obs_x = int(vehicleX - distance/GRID_SIZE * math.sin(total_angle_rad))  
    obs_y = int(vehicleY - distance/GRID_SIZE * math.cos(total_angle_rad))  

    return obs_x, obs_y


# Get the obsatcle position relative to the car
def scan_obstacles(vehicle_x, vehicle_y, vehicle_theta):

    # global vehicle_x, vehicle_y, vehicle_theta
    obstacle_positions = []

    for angle in SCAN_ANGLES:
        distance = get_distance_median(angle, num_samples=5)

        if distance is not None:
            obs_x, obs_y = offsetXY(distance, vehicle_x, vehicle_y, vehicle_theta, angle)  
            print(f"Angle: {angle}, Distance: {distance}, Obstacle: ({obs_x}, {obs_y})")
        
            if 0 <= obs_x < MAP_SIZE and 0 <= obs_y < MAP_SIZE and distance < 50:  # < 100, region of the map
                map_grid[obs_x, obs_y] = 1  # mark as 1
                obstacle_positions.append((obs_y, obs_x))
        else:
            print(f"Angle: {angle}, Distance: None (Measurement Error)")
    return obstacle_positions

# Interpolation
# Two criteria to distinguish between 2 obstacles:
# ANGLES + DIATANCE
def connect_obstacles(map_grid, obstacle_positions, vehicle_x, vehicle_y, angle_threshold=10, max_distance=10):
    
    # Calculate the angle of each obstacle point relative to the car
    obstacle_positions.sort(key=lambda p: math.degrees(math.atan2(p[1] - vehicle_y, p[0] - vehicle_x)))

    for i in range(len(obstacle_positions) - 1):
        x1, y1 = obstacle_positions[i]
        x2, y2 = obstacle_positions[i + 1]

        angle1 = math.degrees(math.atan2(y1 - vehicle_y, x1 - vehicle_x))
        angle2 = math.degrees(math.atan2(y2 - vehicle_y, x2 - vehicle_x))

        # Euclidean distance between two points, if they are too far, not belong to the same obsatcle
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # angle < `angle_threshold` AND `distance < max_distance` 
        if abs(angle1 - angle2) < angle_threshold and distance < max_distance:
            for x, y in bresenham_line(x1, y1, x2, y2):
                if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                    map_grid[x, y] = 1  


# Bresenham, calculates all grid coordinates between two points
def bresenham_line(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points


# expand obstacle region
def expand_obstacles(map_grid, radius=2):
    struct = np.ones((radius, radius))  
    return binary_dilation(map_grid, structure=struct).astype(np.uint8)

# Updated Map
def update_map(curr_map_grid, vehicle_x, vehicle_y, vehicle_theta):
    global map_grid, prev_map_grid ###
    map_grid = curr_map_grid ###

    obstacle_positions = scan_obstacles(vehicle_x, vehicle_y, vehicle_theta)  # scan
    connect_obstacles(map_grid, obstacle_positions, vehicle_x, vehicle_y, angle_threshold=10, max_distance=10)  # connect
    map_grid = expand_obstacles(map_grid, radius=4)  # expand region
    
    #===== New Added =====
    # Combine two maps using element-wise OR
    tmp_combined_map = np.bitwise_or(map_grid, prev_map_grid)
    
    prev_map_grid = map_grid
    map_grid = tmp_combined_map
    # ====================

    # ~ np.save("obstacle_map.npy", map_grid)
    # 現在の日時を取得し、フォーマットを設定（例: obstacle_map_20250215_123456.npy）
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"obstacle_map_{timestamp}.csv"

    # np.save を使ってファイルに保存
    np.savetxt(filename, map_grid, delimiter=",", fmt="%d")

    print("Map saved as 'obstacle_map.npy' and 'obstacle_map.png'")
 


    return map_grid

# TEST
def test():
    vehicle_x, vehicle_y, vehicle_theta = 50, 0, 0
    obstacle_positions = scan_obstacles(vehicle_x, vehicle_y, vehicle_theta)  #first scan
    connect_obstacles(map_grid, obstacle_positions, vehicle_x, vehicle_y, angle_threshold=10, max_distance=10)  # interpolate
    map_grid = expand_obstacles(map_grid, radius=2)  # expand region
    print(map_grid)
    # # print_map(map_grid)  # terminal
    
    np.save("obstacle_map.npy", map_grid)

    # Flip upside down, then rotate 90°
    plt.imshow(np.rot90(np.fliplr(map_grid), k=1), origin='lower')
    # plt.imshow(map_grid, origin='lower')
    plt.savefig("obstacle_map_2.png")
    plt.close()
    print("Map saved as 'obstacle_map.npy' and 'obstacle_map.png'")


if __name__ == "__main__":
    test()
