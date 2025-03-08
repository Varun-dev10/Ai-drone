import queue
import sys
import time
import argparse
sys.path.insert(1, 'components')

import cv2
import collections

import lidar_module as lidar_system
import detector_ssd as object_tracker
import uav_interface as uav
import image_processing as vision_util
import flight_controller as regulator
import keyboard as input_checker

# Command-line argument parser
options_parser = optparse.OptionParser(description='Autonomous navigation for UAV')
options_parser.add_option('--log_dir', type=str, default="logs/experiment1", help='Directory for log storage')
options_parser.add_option('--operation', type=str, default='active', help='Operation type: active, log, or display')
options_parser.add_option('--algorithm', type=str, default='PID', help='Control algorithm: PID or Simple')
parsed_options, remaining_args = options_parser.parse_args()

# System constants
THRESHOLD_RANGE = 1.5                          # meters
HEIGHT_CEILING = 1.5                           # meters
SPEED_CAP = 2                                  # meters per second
ANGLE_LIMIT = 20                               # degrees
BUFFER_SIZE_X = 5
BUFFER_SIZE_Y = 5
ROLLING_AVG_X = queue.deque(maxlen=BUFFER_SIZE_X)  # X-axis rolling average
ROLLING_AVG_Y = queue.deque(maxlen=BUFFER_SIZE_Y)  # Y-axis rolling average
SYSTEM_STATUS = "launch"                       # Initial phase: launch, descend, pursue, seek

def system_initialization():
    print("Starting LIDAR connection")
    lidar_system.start_lidar_connection("/dev/ttyTHS1")

    print("Configuring object detection module")
    object_tracker.setup_recognition()

    print("Linking with UAV")
    if parsed_options.operation == "active":
        print("Operation set to active")
        regulator.link_to_uav('/dev/ttyACM0')
    else:
        print("Operation set to simulation")
        regulator.link_to_uav('127.0.0.1:14550')

system_initialization()

display_width, display_height = object_tracker.retrieve_frame_dimensions()
display_center = (display_width / 2, display_height / 2)
log_video_recorder = cv2.VideoWriter(parsed_options.log_dir + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25.0, (display_width, display_height))

regulator.setup_control_mechanism(parsed_options.algorithm)
regulator.start_log_files(parsed_options.log_dir)

def execute_pursuit():
    print(f"Phase: PURSUIT -> {SYSTEM_STATUS}")
    while True:
        if input_checker.check_key_pressed('q'):
            print("User requested termination")
            perform_descent()

        tracked_objects, frame_speed, current_frame = object_tracker.fetch_recognized_objects()

        if len(tracked_objects) > 0:
            primary_target = tracked_objects[0]

            target_position = primary_target.Center

            horizontal_offset = vision_util.calculate_axis_difference(display_center[0], target_position[0])
            vertical_offset = vision_util.calculate_axis_difference(display_center[1], target_position[1])

            is_lidar_aimed = vision_util.is_within_bounds(display_center, primary_target.Left, primary_target.Right, primary_target.Top, primary_target.Bottom)

            lidar_measure = lidar_system.fetch_lidar_range()[0]

            ROLLING_AVG_Y.append(lidar_measure)
            ROLLING_AVG_X.append(horizontal_offset)

            forward_speed = 0
            if lidar_measure > 0 and is_lidar_aimed and len(ROLLING_AVG_Y) > 0:
                avg_distance_offset = compute_rolling_mean(ROLLING_AVG_Y)
                avg_distance_offset = avg_distance_offset - THRESHOLD_RANGE
                regulator.assign_distance_deviation(avg_distance_offset)
                forward_speed = regulator.obtain_forward_speed_command()

            orientation_adjust = 0
            if len(ROLLING_AVG_X) > 0:
                avg_horizontal_offset = compute_rolling_mean(ROLLING_AVG_X)
                regulator.assign_horizontal_deviation(avg_horizontal_offset)
                orientation_adjust = regulator.obtain_rotation_angle()

            regulator.apply_uav_commands()

            render_frame_data(lidar_measure, target_position, primary_target, current_frame, orientation_adjust, horizontal_offset, vertical_offset, frame_speed, forward_speed, is_lidar_aimed)
        else:
            return "seek"

def execute_searching():
    print(f"Phase: SEARCH -> {SYSTEM_STATUS}")
    initial_timestamp = datetime.datetime.now().timestamp()

    regulator.halt_uav_motion()
    while (datetime.datetime.now().timestamp() - initial_timestamp) < 40:
        if input_checker.check_key_pressed('q'):
            print("User requested termination")
            perform_descent()

        tracked_objects, frame_speed, current_frame = object_tracker.fetch_recognized_objects()
        print(f"Seeking targets: {len(tracked_objects)}")
        if len(tracked_objects) > 0:
            return "pursuit"
        if "test" == parsed_options.operation:
            cv2.putText(current_frame, f"Seeking object. Remaining time: {40 - (datetime.datetime.now().timestamp() - initial_timestamp)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
            show_frame(current_frame)

    return "descend"

def perform_launch():
    regulator.display_uav_status()
    print(f"Phase: LAUNCH -> {SYSTEM_STATUS}")
    regulator.activate_and_ascend(HEIGHT_CEILING)
    return "seek"

def perform_descent():
    print(f"Phase: DESCEND -> {SYSTEM_STATUS}")
    regulator.descend_uav()
    object_tracker.shutdown_camera()
    sys.exit(0)

def show_frame(frame_data):
    if "active" == parsed_options.operation:
        log_video_recorder.write(frame_data)
    else:
        cv2.imshow("display", frame_data)
        cv2.waitKey(1)
    return

def render_frame_data(lidar_measure, target_position, primary_target, current_frame, orientation_adjust, horizontal_offset, vertical_offset, frame_speed, forward_speed, is_lidar_aimed):
    lidar_x_pos = display_width - 50
    lidar_y_pos = display_height - 50
    lidar_y_pos_extended = int(display_height - lidar_measure * 200)
    cv2.line(current_frame, (lidar_x_pos, lidar_y_pos), (lidar_x_pos, lidar_y_pos_extended), (0, 255, 0), thickness=10, lineType=8, shift=0)
    cv2.putText(current_frame, f"Range: {round(lidar_measure, 2)}", (display_width - 300, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

    cv2.line(current_frame, (int(display_center[0]), int(display_center[1])), (int(target_position[0]), int(target_position[1])), (255, 0, 0), thickness=10, lineType=8, shift=0)

    cv2.rectangle(current_frame, (int(primary_target.Left), int(primary_target.Bottom)), (int(primary_target.Right), int(primary_target.Top)), (0, 0, 255), thickness=10)

    cv2.circle(current_frame, (int(display_center[0]), int(display_center[1])), 20, (0, 255, 0), thickness=-1, lineType=8, shift=0)

    cv2.circle(current_frame, (int(target_position[0]), int(target_position[1])), 20, (0, 0, 255), thickness=-1, lineType=8, shift=0)

    cv2.putText(current_frame, f"FPS: {round(frame_speed, 2)} Rotation: {round(orientation_adjust, 2)} Forward: {round(forward_speed, 2)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.putText(current_frame, f"LIDAR aligned: {is_lidar_aimed}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.putText(current_frame, f"X offset: {round(horizontal_offset, 2)} Y offset: {round(vertical_offset, 2)}", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3, cv2.LINE_AA)

    show_frame(current_frame)

def compute_rolling_mean(data_buffer):
    cumulative_sum = 0
    for entry in data_buffer:
        cumulative_sum += entry

    return cumulative_sum / len(data_buffer)

while True:
    # Core execution loop for managing system phases
    """ Controls whether PID or Simple algorithm is applied based on input """

    if SYSTEM_STATUS == "pursuit":
        regulator.update_phase("pursuit")
        SYSTEM_STATUS = execute_pursuit()

    elif SYSTEM_STATUS == "seek":
        regulator.update_phase("seek")
        SYSTEM_STATUS = execute_searching()

    elif SYSTEM_STATUS == "launch":
        SYSTEM_STATUS = perform_launch()

    elif SYSTEM_STATUS == "descend":
        SYSTEM_STATUS = perform_descent()