from dronekit import *

copter = None

def establish_uav_connection(access_point):
    global copter
    if copter == None:
        copter = connect(access_point, wait_ready=True, baud=57600)
    print("UAV connection activated")

def sever_uav_connection():

    copter.close()

def query_firmware_details():
    global copter
    return copter.version

def query_position_data():  
    global copter
    return copter.location.global_frame

def query_altitude_data(): 
    global copter
    return copter.attitude

def query_speed_data():
    global copter
    return copter.velocity

def query_power_status():
    global copter
    return copter.battery

def query_operation_mode():
    global copter
    return copter.mode.name

def query_base_position():
    global copter
    return copter.home_location

def query_navigation_health():
    return copter.ekf_ok

def adjust_camera_angle(new_angle):
    global copter
    print(f"Setting camera angle to: {new_angle}")
    return copter.gimbal.rotate(0, new_angle, 0)

def set_movement_speed(new_speed):
    global copter
    print(f"Adjusting speed to: {new_speed}")
    copter.groundspeed = new_speed

def initiate_ascension(target_elevation):
    global copter

    print("Configuring default speed to 3 m/s for safety")
    copter.groundspeed = 3

    print("Performing pre-launch checks")
    while not copter.is_armable:
        print("Awaiting UAV readiness...")
        time.sleep(1)

    print("Activating propulsion systems")
    copter.mode = VehicleMode("GUIDED")
    copter.armed = True

    while not copter.armed:
        print("Waiting for propulsion activation...")
        time.sleep(1)

    print("Commencing ascent!")
    copter.simple_takeoff(target_elevation)

    while True:
        print(f"Elevation: {copter.location.global_relative_frame.alt}")
        if copter.location.global_relative_frame.alt >= target_elevation * 0.95:
            print("Target elevation achieved")
            break
        time.sleep(1)

def commence_landing():
    """
    Commands the UAV to enter landing mode.
    """
    global copter
    print("Entering DESCEND mode...")
    copter.mode = VehicleMode("LAND")

def return_to_origin():
    """
    Commands the UAV to return to its starting position.
    Note: No obstacle avoidance!
    """
    copter.mode = VehicleMode("RTL")

def issue_rotation_command(target_direction):
   
    global copter
    rotation_rate = 0
    turn_direction = 1  #direction -1 ccw, 1 cw
    
    #heading 0 to 360 degree. if negative then ccw 

    print(f"Issuing rotation command with direction: {target_direction}")

    if target_direction < 0:
        target_direction = target_direction * -1
        turn_direction = -1
    #point drone into correct heading 
    instruction_packet = copter.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_direction,
        rotation_rate,    #speed deg/s
        turn_direction,
        1,                #relative offset 1
        0, 0, 0)

    copter.send_mavlink(instruction_packet)

def issue_motion_command(speed_x, speed_y, speed_z):
    """
    Sends a motion command to the UAV in X, Y, Z directions.
    Args:
        speed_x (float): Forward/backward speed.
        speed_y (float): Left/right speed.
        speed_z (float): Up/down speed.
    """
    global copter

    print(f"Issuing motion command: X={speed_x} Y={speed_y} Z={speed_z}")

    instruction_packet = copter.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        speed_x, speed_y, speed_z,
        0, 0, 0,
        0, 0)

    copter.send_mavlink(instruction_packet)