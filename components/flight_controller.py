from logging import record_log
from components import uav_interface as uav_system
from simple_pid import PIDRegulator
import time as clock

ACTIVATE_PID_ROTATION = True
ACTIVATE_PID_STEERING = False

VELOCITY_LIMIT = 3       # Maximum velocity in m/s
ROTATION_LIMIT = 20      # Maximum rotation rate in degrees/s

ROTATION_PROPORTIONAL = 0.6
ROTATION_INTEGRAL = 0
ROTATION_DERIVATIVE = 0

STEERING_PROPORTIONAL = 0.2
STEERING_INTEGRAL = 0
STEERING_DERIVATIVE = 0

is_regulation_enabled = True
rotation_regulator = None
steering_regulator = None
active_rotation_value = 0
active_steering_value = 0
rotation_input_data = 0
velocity_input_data = 0
is_regulation_enabled = True

rotation_log_file = None
velocity_log_file = None

def configure_regulation_system(method):
    """
    Sets up the regulation system for rotation and steering.
    Args:
        method (str): Regulation method ('PID' or 'Simple').
    """
    global steering_regulator, rotation_regulator

    print("Preparing regulation system")

    if method == 'PID':
        rotation_regulator = PIDRegulator(ROTATION_PROPORTIONAL, ROTATION_INTEGRAL, ROTATION_DERIVATIVE, setpoint=0)
        rotation_regulator.output_limits = (-ROTATION_LIMIT, ROTATION_LIMIT)
        steering_regulator = PIDRegulator(STEERING_PROPORTIONAL, STEERING_INTEGRAL, STEERING_DERIVATIVE, setpoint=0)
        steering_regulator.output_limits = (-VELOCITY_LIMIT, VELOCITY_LIMIT)
        print("PID regulation configured")
    else:
        rotation_regulator = PIDRegulator(ROTATION_PROPORTIONAL, 0, 0, setpoint=0)
        rotation_regulator.output_limits = (-ROTATION_LIMIT, ROTATION_LIMIT)
        steering_regulator = PIDRegulator(STEERING_PROPORTIONAL, 0, 0, setpoint=0)
        steering_regulator.output_limits = (-VELOCITY_LIMIT, VELOCITY_LIMIT)
        print("Simple regulation configured")

def activate_uav_connection(uav_endpoint):
    """
    Establishes a connection to the UAV.
    Args:
        uav_endpoint (str): Connection endpoint for the UAV.
    """
    uav_system.connect_uav(uav_endpoint)

def fetch_rotation_value():
    """
    Obtains the current rotation value.
    Returns:
        float: Rotation command value.
    """
    return active_rotation_value

def set_horizontal_input(new_horizontal_input):
    """
    Sets the horizontal input for rotation regulation.
    Args:
        new_horizontal_input (float): Horizontal deviation value.
    """
    global rotation_input_data
    rotation_input_data = new_horizontal_input

def fetch_velocity_command():
    """
    Obtains the current velocity command.
    Returns:
        float: Velocity command value.
    """
    return active_steering_value

def set_distance_input(new_distance_input):
    """
    Sets the distance input for velocity regulation.
    Args:
        new_distance_input (float): Distance deviation value.
    """
    global velocity_input_data
    velocity_input_data = new_distance_input

def set_operation_phase(new_phase):
    """
    Updates the operation phase.
    Args:
        new_phase (str): New phase identifier.
    """
    global phase
    phase = new_phase

def trigger_ascension(maximum_height):
    """
    Initiates UAV ascension to the specified height.
    Args:
        maximum_height (float): Target height in meters.
    """
    uav_system.ascend_and_activate(maximum_height)

def trigger_descent():
    """
    Commands the UAV to descend.
    """
    uav_system.descend()

def show_uav_information():
    """
    Displays the current status of the UAV.
    """
    print(uav_system.get_navigation_status())
    print(uav_system.get_power_status())
    print(uav_system.get_firmware_version())

def prepare_log_files(base_filepath):
    """
    Sets up log files for rotation and velocity data.
    Args:
        base_filepath (str): Base path for log files.
    """
    global rotation_log_file, velocity_log_file
    rotation_log_file = open(base_filepath + "_rotation.txt", "a")
    rotation_log_file.write("P: I: D: Error: Output:\n")

    velocity_log_file = open(base_filepath + "_velocity.txt", "a")
    velocity_log_file.write("P: I: D: Error: Output:\n")

def record_rotation_log(output_value):
    """
    Records rotation regulation data to the log file.
    Args:
        output_value (float): Rotation output value.
    """
    global rotation_log_file
    rotation_log_file.write(f"0,0,0,{rotation_input_data},{output_value}\n")

def record_velocity_log(output_value):
    """
    Records velocity regulation data to the log file.
    Args:
        output_value (float): Velocity output value.
    """
    global velocity_log_file
    velocity_log_file.write(f"0,0,0,{rotation_input_data},{output_value}\n")

def regulate_uav_motion():
    """
    Applies regulation commands to the UAV.
    """
    global active_rotation_value, active_steering_value
    if rotation_input_data == 0:
        uav_system.send_rotation_command(0)
    else:
        active_rotation_value = (rotation_regulator(rotation_input_data) * -1)
        uav_system.send_rotation_command(active_rotation_value)
        record_rotation_log(active_rotation_value)

    if velocity_input_data == 0:
        uav_system.send_motion_command(0, 0, 0)
    else:
        active_steering_value = (steering_regulator(velocity_input_data) * -1)
        uav_system.send_motion_command(active_steering_value, 0, 0)
        record_velocity_log(active_steering_value)

def cease_uav_motion():
    """
    Halts all UAV motion.
    """
    uav_system.send_rotation_command(0)
    uav_system.send_motion_command(0, 0, 0)