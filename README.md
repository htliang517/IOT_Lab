# lab1B

CS437 IOT - Lab1B

# Installation

```bash
pip install -r requirements.txt
```

# How to run the code

```bash
python3 main.py --start [0,0,0] --goal [100,0]
```

Args note:

- --start [start_x_position, start_y_position, start_car_heading]
- --goal [goal_x_position, goal_y_position]

# Code Structure

### `config.yaml`

- Basic unchanged configuration of the car.

### `efficientdet_lite0.tflite`

- Tensorflow detection model file

### `mapping.py`

- Map generation functions
- Act as a library
- Called in `control.py`

### `planner.py`

- Implementation of A* algorithm
- Act as a library
- Called in `control.py`

### `detect.py`

- Object detection code using openCV and tensorflow lite
- Run in one of the progress
- Generate `stop_needed` signal to `control.py`

### `object_detection_utils.py`

- Useful function for `detect.py`
- Act as a library

### `control.py`

- Main navigation code
- Run in one of the progress

### `main.py`

- Wrap-up python file to make `detect.py` and `control.py` run simultaneously
