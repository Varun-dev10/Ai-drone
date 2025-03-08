from dronekit import *

autonomous_unit = None

def establish_uav_connection(access_point):
    """
    Creates a connection to the UAV using the provided access point.
    Args:
        access_point (str): Connection endpoint (e.g., '/dev/ttyACM0').
    """
    global autonomous_unit
    if autonomous_unit == None:
        autonomous_unit = connect(access_point, wait_ready=True, baud=57600)
    print("UAV connection activated")

def sever_uav_connection():
    """
    Disconnects from the UAV.
    """
    autonomous_unit.close()

def query_firmware_details():
    """
    Fetches the firmware version of the UAV.
    Returns:
        str: Firmware version.
    """
    global autonomous_unit
    return autonomous_unit.version

def query_position_data():
    """
    Fetches the current GPS coordinates of the UAV.
    Returns:
        Location: GPS coordinates.
    """
    global autonomous_unit
    return autonomous_unit.location.global_frame

def query_altitude_data():
    """
    Fetches the current orientation and altitude of the UAV.
    Returns:
        Attitude: Orientation data.
    """
    global autonomous_unit
    return autonomous_unit.attitude

def query_speed_data():
    """
    Fetches the current velocity of the UAV.
    Returns:
        list: Velocity vector [x, y, z].
    """
    global autonomous_unit
    return autonomous_unit.velocity

def query_power_status():
    """
    Fetches the battery status of the UAV.
    Returns:
        Battery: Battery information.
    """
    global autonomous_unit
    return autonomous_unit.battery

def query_operation_mode():
    """
    Fetches the current operation mode of the UAV.
    Returns:
        str: Current mode name.
    """
    global autonomous_unit
    return autonomous_unit.mode.name

def query_base_position():
    """
    Fetches the designated home position of the UAV.
    Returns:
        Location: Home coordinates.
    """
    global autonomous_unit
    return autonomous_unit.home_location

def query_navigation_health():
    """
    Verifies the health of the navigation system (EKF).
    Returns:
        bool: True if healthy, False otherwise.
    """
    return autonomous_unit.ekf_ok

def adjust_camera_angle(new_angle):
    """
    Adjusts the camera gimbal to the specified angle.
    Args:
        new_angle (float): Target angle in degrees.
    """
    global autonomous_unit
    print(f"Setting camera angle to: {new_angle}")
    return autonomous_unit.gimbal.rotate(0, new_angle, 0)

def set_movement_speed(new_speed):
    """
    Sets the UAV's movement speed.
    Args:
        new_speed (float): Target speed in m/s.
    """
    global autonomous_unit
    print(f"Adjusting speed to: {new_speed}")
    autonomous_unit.groundspeed = new_speed

def initiate_ascension(target_elevation):
    """
    Prepares and launches the UAV to the specified elevation.
    Args:
        target_elevation (float): Target elevation in meters.
    """
    global autonomous_unit

    print("Configuring default speed to 3 m/s for safety")
    autonomous_unit.groundspeed = 3

    print("Performing pre-launch checks")
    while not autonomous_unit.is_armable:
        print("Awaiting UAV readiness...")
        timing.sleep(1)

    print("Activating propulsion systems")
    autonomous_unit.mode = VehicleMode("GUIDED")
    autonomous_unit.armed = True

    while not autonomous_unit.armed:
        print("Waiting for propulsion activation...")
        timing.sleep(1)

    print("Commencing ascent!")
    autonomous_unit.simple_takeoff(target_elevation)

    while True:
        print(f"Elevation: {autonomous_unit.location.global_relative_frame.alt}")
        if autonomous_unit.location.global_relative_frame.alt >= target_elevation * 0.95:
            print("Target elevation achieved")
            break
        timing.sleep(1)

def commence_landing():
    """
    Commands the UAV to enter landing mode.
    """
    global autonomous_unit
    print("Entering DESCEND mode...")
    autonomous_unit.mode = VehicleMode("LAND")

def return_to_origin():
    """
    Commands the UAV to return to its starting position.
    Note: No obstacle avoidance!
    """
    autonomous_unit.mode = VehicleMode("RTL")

def issue_rotation_command(target_direction):
    """
    Sends a rotation command to the UAV.
    Args:
        target_direction (float): Desired direction in degrees (0-360).
    """
    global autonomous_unit
    rotation_rate = 0
    turn_direction = 1

    print(f"Issuing rotation command with direction: {target_direction}")

    if target_direction < 0:
        target_direction = target_direction * -1
        turn_direction = -1

    instruction_packet = autonomous_unit.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_direction,
        rotation_rate,
        turn_direction,
        1,
        0, 0, 0)

    autonomous_unit.send_mavlink(instruction_packet)

def issue_motion_command(speed_x, speed_y, speed_z):
    """
    Sends a motion command to the UAV in X, Y, Z directions.
    Args:
        speed_x (float): Forward/backward speed.
        speed_y (float): Left/right speed.
        speed_z (float): Up/down speed.
    """
    global autonomous_unit

    print(f"Issuing motion command: X={speed_x} Y={speed_y} Z={speed_z}")

    instruction_packet = autonomous_unit.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        speed_x, speed_y, speed_z,
        0, 0, 0,
        0, 0)

    autonomous_unit.send_mavlink(instruction_packet)