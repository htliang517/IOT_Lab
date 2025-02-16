import multiprocessing
import time
import argparse
import picar_4wd as fc

# ===== Import our modules =====
import detect
import mapping
import planner
import control

def parse_list(arg):
    return [int(x) for x in arg.strip('[]').split(',')]
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--start',
        help='Start position of the car. List of 3 integers: [x, y, theta]',
        required=False,
        default=[0,0,0],
        type=parse_list)
    parser.add_argument(
        '--goal1', 
        help='Goal position of the car. List of 2 integers: [x, y]',
        required=False,
        default=[100, 0],
        type=parse_list)
    parser.add_argument(
        '--goal2', 
        help='Goal position of the car. List of 2 integers: [x, y]',
        required=False,
        default=[100, 0],
        type=parse_list)
    args = parser.parse_args()

    start_state = args.start
    goal_state1 = args.goal1
    goal_state2 = args.goal2
    
    # ===== Setup and start processes - PLAN 1 =====
    manager = multiprocessing.Manager()
    stop_needed = manager.Value('b', False)     # Boolean
    p1 = multiprocessing.Process(target=detect.run, args=(stop_needed,), daemon=True)
    p2 = multiprocessing.Process(target=control.run, args=(stop_needed, start_state, goal_state1, goal_state2), daemon=True)
    p1.start()
    p2.start()

    # ===== Setup and start processes - PLAN 2 (Dismissed) =====
    # manager = multiprocessing.Manager()
    # stop_needed = manager.Value('b', False)     # Boolean
    # state = manager.list(start_position)        # List of [x, y, theta]
    # plan = manager.list([])                     # List of waypoints
    # map = multiprocessing.Array('i', 100*100)   # 100x100 map
    # for i in range(100*100):                    # Initialize map
    #     map[i] = 0
    # p1 = multiprocessing.Process(target=detect.run, args=(stop_needed,), daemon=True)
    # p2 = multiprocessing.Process(target=mapping.run, args=(map, state,), daemon=True)
    # p3 = multiprocessing.Process(target=planner.run, args=(map, state, goal_position, plan,), daemon=True)
    # p4 = multiprocessing.Process(target=control.run, args=(stop_needed, plan, state,), daemon=True)
    # p1.start()
    # p2.start()
    # p3.start()
    # p4.start()

    try:
        while True:
            time.sleep(1)  # Keep main.py running
    except KeyboardInterrupt:
        print("\nCtrl+C detected! Exiting...")
        fc.stop()
        p1.terminate()
        p2.terminate()
        p1.join()
        p2.join()
        print("Finish!!!")
