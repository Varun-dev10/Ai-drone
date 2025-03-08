import jetson_inference as neural_processor
import jetson_utils as camera_handler
import cv2 as vision_lib
import numpy as array_utils

recognition_engine = None
image_source = None

def prepare_detection_system():
    """
    Sets up the neural network and camera for object recognition.
    """
    global recognition_engine, image_source
    recognition_engine = neural_processor.detectNet("ssd-mobilenet-v2")
    image_source = camera_handler.videoSource("csi://0")
    print("Recognition system ready")

def get_image_resolution():
    """
    Obtains the resolution of the captured image.
    Returns:
        tuple: Width and height of the image.
    """
    return image_source.GetWidth(), image_source.GetHeight()

def terminate_image_source():
    """
    Shuts down the camera connection.
    """
    image_source.Close()

def retrieve_detected_entities():
    """
    Detects objects in the current image frame.
    Returns:
        tuple: List of detected humans, processing speed, and image data.
    """
    detected_humans = []
    captured_image = image_source.Capture()
    recognition_results = recognition_engine.Detect(captured_image)
    for result in recognition_results:
        if result.ClassID == 1:
            detected_humans.append(result)
    processing_speed = recognition_engine.GetNetworkFPS()

    return detected_humans, processing_speed, camera_handler.cudaToNumpy(captured_image)