# Copyright 2021 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Main script to run the object detection routine."""
import sys
import time
from multiprocessing import Queue

import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import object_detection_utils as utils

from picamera2 import Picamera2
import yaml

def detect(stop_needed: bool, model: str, camera_id: int, width: int, height: int, num_threads: int,
        enable_edgetpu: bool) -> None:
    """Continuously run inference on images acquired from the camera.

    Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
    """

    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

    # ~ # Start capturing video input from the camera
    # ~ cap = cv2.VideoCapture(camera_id)
    # ~ cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    # ~ cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    # Initialize the object detection model
    base_options = core.BaseOptions(
        file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
    detection_options = processor.DetectionOptions(
        max_results=3, score_threshold=0.3)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)


    # Picameraを起動
    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={
    "format": 'XRGB8888',
    "size": (640, 480)
    }))
    camera.start()

    # Continuously capture images from the camera and run inference
    while True:
        # カメラから画像を取得
        image = camera.capture_array()

        # 画像が3チャンネル以外の場合は3チャンネルに変換する
        channels = 1 if len(image.shape) == 2 else image.shape[2]
        if channels == 1:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if channels == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        counter += 1
        image = cv2.flip(image, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)

        # Draw keypoints and edges on input image
        image, result_dict = utils.visualize(image, detection_result, visualization=False)
        
        ########## This part is commented out since we don't need to visualize on Rpi ##########
        # # Calculate the FPS
        # if counter % fps_avg_frame_count == 0:
        #     end_time = time.time()
        #     fps = fps_avg_frame_count / (end_time - start_time)
        #     start_time = time.time()

        # # Show the FPS
        # fps_text = 'FPS = {:.1f}'.format(fps)
        # text_location = (left_margin, row_size)
        # cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
        #             font_size, text_color, font_thickness)
        ########################################################################################

        # Check if any object is detected and update the stop_needed signal
        all_false = all(value is False for value in result_dict.values())
        stop_needed.value = not all_false
        if stop_needed.value:
            print("Stop needed! Sleep for 10 seconds.")
            time.sleep(10)
            stop_needed.value = False
            time.sleep(10)      # Wait for the car to move so that obstacle disappears

        # Stop the program if the ESC key is pressed.
        if cv2.waitKey(1) == 27:
            break

        ########## This part is commented out since we don't need to visualize on Rpi ##########
        # cv2.imshow('object_detector', image)
        ########################################################################################
        
        # cap.release()
        cv2.destroyAllWindows()

  

def run(stop_needed: bool) -> None:
    with open('config.yaml', 'r') as file:
        try:
            config = yaml.safe_load(file)
        except Exception as e:
            print(f"Error with loading config.yaml: {e}")

    model = config['object_detection']['model']
    camera_id = config['object_detection']['cameraID']
    frame_width = config['object_detection']['frameWidth']
    frame_height = config['object_detection']['frameHeight']
    num_threads = config['object_detection']['numThreads']
    enable_edge_tpu = config['object_detection']['enableEdgeTPU']
    
    # Config Check (Check for None values)
    if model is None:
        raise ValueError("Config Error: 'model' is None in code [detect.py].")
    if camera_id is None:
        raise ValueError("Config Error: 'camera_id' is None in code [detect.py].")
    if frame_width is None:
        raise ValueError("Config Error: 'frame_width' is None in code [detect.py].")
    if frame_height is None:
        raise ValueError("Config Error: 'frame_height' is None in code [detect.py].")
    if num_threads is None:
        raise ValueError("Config Error: 'num_threads' is None in code [detect.py].")
    if enable_edge_tpu is None:
        raise ValueError("Config Error: 'enable_edge_tpu' is None in code [detect.py].")

    detect(stop_needed, model, camera_id, frame_width, frame_height, num_threads, enable_edge_tpu)


# if __name__ == '__main__':
#     run()