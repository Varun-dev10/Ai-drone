from logging import record_log
from components import drone_interface as drone
from simple_pid import PID
import time as clock

Pid_yaw = True
Pid_roll = False

max_speed = 3       # Maximum velocity in m/s
max_yaw = 20      # Maximum YAW rate in degrees/s

YAW_PROPORTIONAL = 0.6
YAW_INTEGRAL = 0
YAW_DERIVATIVE = 0

Roll_PROPORTIONAL = 0.2
Roll_INTEGRAL = 0
Roll_DERIVATIVE = 0

is_regulation_enabled = True
YAW_pid = None
Roll_pid = None
active_YAW_value = 0
active_Roll_value = 0
YAW_input_data = 0
velocity_input_data = 0
is_regulation_enabled = True

YAW_log_file = None
velocity_log_file = None

def configure_regulation_system(method):
    
    # Sets up the regulation system for YAW and Roll.
 
    global Roll_pid, YAW_pid

    print("Preparing regulation system")

    if method == 'PID':
        YAW_pid = PID(YAW_PROPORTIONAL, YAW_INTEGRAL, YAW_DERIVATIVE, setpoint=0)
        YAW_pid.output_limits = (-max_yaw, max_yaw)
        Roll_pid = PID(Roll_PROPORTIONAL, Roll_INTEGRAL, Roll_DERIVATIVE, setpoint=0)
        Roll_pid.output_limits = (-max_speed, max_speed)
        print("PID  configured")
    else:
        YAW_pid = PID(YAW_PROPORTIONAL, 0, 0, setpoint=0)
        YAW_pid.output_limits = (-max_yaw, max_yaw)
        Roll_pid = PID(Roll_PROPORTIONAL, 0, 0, setpoint=0)
        Roll_pid.output_limits = (-max_speed, max_speed)
        print("Simple regulation configured")

def activate_drone_connection(drone_location): 
    # Establishes a connection to the drone.
    drone.connect_drone(drone_location)

def fetch_YAW_value():
    # Obtains the current YAW value.
    return active_YAW_value

def set_horizontal_input(new_horizontal_input):
    # Sets the horizontal input for YAW regulation.
    global YAW_input_data
    YAW_input_data = new_horizontal_input

def fetch_velocity_command():
    # Obtains the current velocity command.
    return active_Roll_value

def set_distance_input(new_distance_input):
    # Sets the distance input for velocity regulation.
    global velocity_input_data
    velocity_input_data = new_distance_input

def set_operation_phase(new_phase):
    # Updates the operation phase.
    global phase
    phase = new_phase

def arm_takeoff(maximum_height):
    # Initiates drone ascension to the specified height.
    drone.ascend_and_activate(maximum_height)
def land():
    drone.land()

def show_drone_information():
    # Displays the current status of the drone.
    print(drone.get_EKF_status())
    print(drone.get_battery_info())
    print(drone.get_version())




def prepare_log_files(base_filepath):
    # Sets up log files for YAW and velocity data.
    global YAW_log_file, velocity_log_file
    YAW_log_file = open(base_filepath + "_YAW.txt", "a")
    YAW_log_file.write("P: I: D: Error: Output:\n")

    velocity_log_file = open(base_filepath + "_velocity.txt", "a")
    velocity_log_file.write("P: I: D: Error: Output:\n")

def record_YAW_log(output_value):
    # Records YAW regulation data to the log file.
    global YAW_log_file
    YAW_log_file.write(f"0,0,0,{YAW_input_data},{output_value}\n")

def record_velocity_log(output_value):
    # Records velocity regulation data to the log file.
    global velocity_log_file
    velocity_log_file.write(f"0,0,0,{YAW_input_data},{output_value}\n")

def regulate_drone_motion():
    # Applies regulation commands to the drone.
    global active_YAW_value, active_Roll_value
    if YAW_input_data == 0:
        drone.send_YAW_command(0)
    else:
        active_YAW_value = (YAW_pid(YAW_input_data) * -1)
        drone.send_YAW_command(active_YAW_value)
        record_YAW_log(active_YAW_value)

    if velocity_input_data == 0:
        drone.send_motion_command(0, 0, 0)
    else:
        active_Roll_value = (Roll_pid(velocity_input_data) * -1)
        drone.send_motion_command(active_Roll_value, 0, 0)
        record_velocity_log(active_Roll_value)

def stop_drone():
    drone.send_YAW_command(0)
    drone.send_motion_command(0, 0, 0)