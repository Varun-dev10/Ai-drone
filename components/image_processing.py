import cv2 as image_proc
import argparse as opt_handler
import sys as sys_ops
import math as calc

def determine_midpoint(shape_data):
    """
    Determines the midpoint of a given shape.
    Args:
        shape_data: Shape data from image processing.
    Returns:
        tuple: Midpoint coordinates (x, y).
    """
    shape_moments = image_proc.moments(shape_data)
    mid_x = int(shape_moments['m10'] / shape_moments['m00'])
    mid_y = int(shape_moments['m01'] / shape_moments['m00'])
    return (mid_x, mid_y)

def measure_distance(coord_a, coord_b):
    """
    Measures the Euclidean distance between two coordinates.
    Args:
        coord_a (tuple): First coordinate (x, y).
        coord_b (tuple): Second coordinate (x, y).
    Returns:
        float: Distance value.
    """
    return calc.sqrt((coord_a[0] - coord_b[0]) ** 2 + (coord_a[1] - coord_b[1]) ** 2)

def measure_axis_deviation(value_a, value_b):
    """
    Measures the deviation between two values on a single axis.
    Args:
        value_a (float): First value.
        value_b (float): Second value.
    Returns:
        float: Deviation value.
    """
    return value_b - value_a

def check_coordinate_in_region(coord, boundary_left, boundary_right, boundary_top, boundary_bottom):
    """
    Checks if a coordinate lies within a specified region.
    Args:
        coord (tuple): Coordinate to check (x, y).
        boundary_left (float): Left boundary.
        boundary_right (float): Right boundary.
        boundary_top (float): Top boundary.
        boundary_bottom (float): Bottom boundary.
    Returns:
        bool: True if within region, False otherwise.
    """
    if boundary_left < coord[0] and coord[0] < boundary_right and boundary_top < coord[1] and coord[1] < boundary_bottom:
        return True
    else:
        return False

def handle_frame_data(image_data):
    """
    Processes the image data to identify and annotate shapes.
    Args:
        image_data: Input image data.
    Returns:
        Annotated image data.
    """
    monochrome = image_proc.cvtColor(image_data, image_proc.COLOR_BGR2GRAY)
    shape_list, hierarchy_info = image_proc.findContours(monochrome, image_proc.RETR_EXTERNAL, image_proc.CHAIN_APPROX_SIMPLE)

    image_midpoint = (round(image_data.shape[1] / 2), round(image_data.shape[0] / 2))

    selected_shapes = []
    for shape_item in shape_list:
        shape_size = image_proc.contourArea(shape_item)
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
        image_proc.putText(image_data, "NO OBJECT", (50, 50), image_proc.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, image_proc.LINE_AA)

    image_proc.circle(image_data, chosen_target[0], 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)
    image_proc.putText(image_data, str(chosen_target[1]), (50, 50), image_proc.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, image_proc.LINE_AA)
    image_proc.line(image_data, image_midpoint, chosen_target[0], (255, 0, 0), thickness=10, lineType=8, shift=0)

    image_proc.circle(image_data, image_midpoint, 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    image_proc.drawContours(image_data, selected_shapes, -1, (255, 255, 255), 3)

    return image_data