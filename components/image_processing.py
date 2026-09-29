import cv2 
import argparse 
import sys 
import math 

def determine_midpoint(shape_data):
    # Determines the midpoint of a given shape.
    shape_moments = cv2.moments(shape_data)
    mid_x = int(shape_moments['m10'] / shape_moments['m00'])
    mid_y = int(shape_moments['m01'] / shape_moments['m00'])
    return (mid_x, mid_y)

def measure_distance(point_a, point_b):
    # Measures the Euclidean distance between two pointinates.
    return math.sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)

def measure_axis_deviation(value_a, value_b):
    # Measures the deviation between two values on a single axis.
    return value_b - value_a

def check_pointinate_in_region(point, boundary_left, boundary_right, boundary_top, boundary_bottom):
    # Checks if a pointinate lies within a specified region.
    if boundary_left < point[0] and point[0] < boundary_right and boundary_top < point[1] and point[1] < boundary_bottom:
        return True
    else:
        return False

def handle_frame_data(image_data):
    # Processes the image data to identify and annotate shapes.
  
    monochrome = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    shape_list, hierarchy_info = cv2.findContours(monochrome, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    image_midpoint = (round(image_data.shape[1] / 2), round(image_data.shape[0] / 2))

    selected_shapes = []
    for shape_item in shape_list:
        shape_size = cv2.contourArea(shape_item)
        print(shape_size)
        if shape_size > 100000.0:
            selected_shapes.append(shape_item)

    shape_counter = 0
    chosen_target = ((image_midpoint[0], image_midpoint[1]), 0)
    if len(selected_shapes) > 1:
        for shape_item in selected_shapes:
            shape_midpoint = determine_midpoint(shape_item)
            if shape_counter == 0:
                shape_midpoint = determine_midpoint(shape_item)
                distance_measure = measure_distance(shape_midpoint, image_midpoint)
                chosen_target = (shape_midpoint, distance_measure)
            else:
                current_measure = measure_distance(shape_midpoint, image_midpoint)
                if abs(chosen_target[1]) > abs(current_measure):
                    shape_midpoint = determine_midpoint(shape_item)
                    chosen_target = (shape_midpoint, current_measure)
            shape_counter += 1

    elif len(selected_shapes) == 1:
        shape_midpoint = determine_midpoint(selected_shapes[0])
        distance_measure = measure_distance(shape_midpoint, image_midpoint)
        chosen_target = (shape_midpoint, distance_measure)

    else:
        cv2.putText(image_data, "NO OBJECT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

    cv2.circle(image_data, chosen_target[0], 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)
    cv2.putText(image_data, str(chosen_target[1]), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.line(image_data, image_midpoint, chosen_target[0], (255, 0, 0), thickness=10, lineType=8, shift=0)

    cv2.circle(image_data, image_midpoint, 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    cv2.drawContours(image_data, selected_shapes, -1, (255, 255, 255), 3)

    return image_data