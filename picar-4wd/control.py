import picar_4wd as fc
import time, math, yaml

import mapping
import planner

# Setup the configuration
with open('config.yaml', 'r') as file:
    try:
        config = yaml.safe_load(file)
    except Exception as e:
        print(f"Error with loading config.yaml: {e}")

pwm = config['control']['pwm']
step_time_turn45 = config['control']['stepTime_turn45']
step_time_forward = config['control']['stepTime_forward']

# Config Check (Check for None values)
if pwm is None:
    raise ValueError("Config Error: 'pwm' is None in code [control.py].")
if step_time_turn45 is None:
    raise ValueError("Config Error: 'step_time_turn45' is None in code [control.py].")
if step_time_forward is None:
    raise ValueError("Config Error: 'step_time_forward' is None in code [control.py].")


# ========== Obstacle Detection ==========
def encounter_obstacle():
    # Scan the obstacles
    obstacle_distance = mapping.get_distance_median(angle=0, num_samples=5)
    if obstacle_distance is not None and obstacle_distance < 40:
        return True
    return False

# ========== Moving Function ==========
def drive_forward(pwm , step_time):
    """
    Move the car forward.
    :param angle: Angle to turn the car (in degrees)
    :param pwm: PWM value for the motors (default: 5)
    :param step_time: One step of time to move forward 1 grid
    """
    fc.forward(pwm)
    time.sleep(step_time)
    # fc.stop()
    # time.sleep(0.1)

### Currently assuming the car would not move backward 
def drive_backward(pwm , step_time):
    """
    Move the car backward.
    :param angle: Angle to turn the car (in degrees)
    :param pwm: PWM value for the motors (default: 5)
    :param step_time: One step of time to move forward 1 grid
    """
    fc.backward(pwm)
    time.sleep(step_time)
    # fc.stop()
    # time.sleep(0.1)

def drive_right(angle, pwm , step_time):
    """
    Turn the car right by 45 degrees.
    :param angle: Angle to turn the car (in degrees)
    :param pwm: PWM value for the motors (default: 5)
    :param step_time: One step of time to turn 45 degrees
    """
    fc.stop()
    time.sleep(0.05)

    cycle = int(angle / 45)
    operating_time = cycle * step_time

    fc.turn_right(pwm)
    time.sleep(operating_time)
    fc.stop()
    time.sleep(0.5)

def drive_left(angle, pwm, step_time):
    """
    Turn the car left by 45 degrees.
    :param angle: Angle to turn the car (in degrees)
    :param pwm: PWM value for the motors (default: 5)
    :param step_time: One step of time to turn 45 degrees
    """
    fc.stop()
    time.sleep(0.05)

    cycle = int(angle / 45)
    operating_time = cycle * step_time + 0.1
    
    fc.turn_left(pwm)
    time.sleep(operating_time)
    fc.stop()
    time.sleep(0.5)
# ===================================

# ========== Control Logic ==========
def navigate_to_waypoint(state, next_position):
    """Navigate the car to the next waypoint.
    
    Args:
        state (list): Current state of the car.
        next_position (tuple): (x, y) position of the next waypoint.
    
    Returns:
        tuple: (steering_angle, speed) to move toward the waypoint.
    """
    # Extract coordinates
    x, y, curr_angle = state
    target_x, target_y = next_position

    # Compute the desired heading
    print("target, current", next_position, state)
    desired_heading = math.degrees(-math.atan2(target_x - x, y - target_y))
    print("Desired >>>> ",desired_heading)
    # Compute the steering angle (difference between desired and current heading)
    steering_angle = desired_heading - curr_angle
    
    # Normalize steering angle to be within [-pi, pi]
    steering_angle = (steering_angle + 180) % (2 * 180) - 180

    print("steering angle", steering_angle)

    if steering_angle > 0:
        drive_left(steering_angle, pwm, step_time_turn45)
        drive_forward(pwm, step_time_forward)
        print("turn left")
    elif steering_angle < 0:
        drive_right(abs(steering_angle), pwm, step_time_turn45)
        drive_forward(pwm, step_time_forward)
        print("turn right")
    else:
        drive_forward(pwm, step_time_forward)
        print("move forward")
    
    curr_angle += steering_angle
    curr_position = next_position

    return [curr_position[0], curr_position[1], curr_angle]
# ===================================

# ========== Main Function ==========
def run(stop_needed, start_state, goal_state):
    # Initialize the map
    curr_map_grid = mapping.map_grid
    
    vehicle_x, vehicle_y, vehicle_theta = start_state

    # Initialize the path
    path = planner.a_star(curr_map_grid, [vehicle_x, vehicle_y], goal_state)

    # Start driving the car
    while True:
        current_state = [vehicle_x, vehicle_y, vehicle_theta]
        # If path is empty, we have reached the goal
        if not path:
            print("Reached the goal!")
            fc.stop()
            break

        # Check Stop Sign First
        if stop_needed.value:
            print("@@@@@ STOP")
            fc.stop()
            time.sleep(5)
            # ~ drive_forward(pwm, step_time_forward)
            stop_needed.value = False
            

        # If encounter obstacles, remapping and replanning
        if encounter_obstacle():
            print("Obstacle detected!")
            
            # Stop the car
            fc.stop()
            time.sleep(0.1)

            # Mapping
            curr_map_grid = mapping.update_map(curr_map_grid, current_state[0], current_state[1], current_state[2])
            
            # ~ plt.imshow(np.rot90(np.fliplr(curr_map_grid), k=1), origin='lower')
            
            print("@@@@@ Map", curr_map_grid)
            # Plan a new path
            path = planner.a_star(curr_map_grid, [current_state[0], current_state[1]], goal_state)
            print("@@@@@ Path:", path)
            path.pop(0)
            fc.servo.set_angle(0)

        # Move the car to the next waypoint
        next_position = path.pop(0)
        new_state = navigate_to_waypoint(current_state, next_position)

        # Update the current state
        vehicle_x, vehicle_y, vehicle_theta = new_state
        print(f"Current State: ({vehicle_x}, {vehicle_y}, {vehicle_theta})")
        
# ===================================
