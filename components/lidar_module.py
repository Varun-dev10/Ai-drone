import serial as serial_comm
import time as timing
import numpy as num_array

# Serial port handler for LIDAR interaction
port_handler = None

def activate_lidar_link(port_identifier):
    """
    Initiates a serial connection to the LIDAR device.
    Args:
        port_identifier (str): Serial port identifier (e.g., '/dev/ttyTHS1').
    Returns:
        str: Connection status ('successful' or 'already_active').
    """
    global port_handler
    port_handler = serial_comm.Serial(port_identifier, 115200, timeout=0)
    if port_handler.isOpen() == False:
        port_handler.open()
        return "successful"
    else:
        return "already_active"

def deactivate_lidar_link():
    """
    Terminates the serial connection to the LIDAR device.
    """
    global port_handler
    port_handler.close()

def verify_lidar_link():
    """
    Confirms if the LIDAR serial connection is active.
    Returns:
        bool: True if active, False otherwise.
    """
    global port_handler
    return port_handler.isOpen()

def obtain_lidar_measurements():
    """
    Retrieves distance and signal strength from the LIDAR device.
    Returns:
        tuple: (distance in meters, signal strength value).
    """
    global port_handler
    while True:
        pending_bytes = port_handler.in_waiting
        if pending_bytes > 6:
            incoming_data = port_handler.read(7)
            port_handler.reset_input_buffer()
            if incoming_data[0] == 0x59 and incoming_data[1] == 0x59:
                range_value = incoming_data[2] + incoming_data[3] * 256
                signal_value = incoming_data[4] + incoming_data[5] * 256
                return range_value / 100.0, signal_value

def obtain_lidar_thermal_reading():
    """
    Retrieves the temperature reading from the LIDAR device.
    Returns:
        float: Temperature in Celsius.
    """
    global port_handler
    while True:
        pending_bytes = port_handler.in_waiting
        if pending_bytes > 8:
            incoming_data = port_handler.read(9)
            port_handler.reset_input_buffer()
            if incoming_data[0] == 0x59 and incoming_data[1] == 0x59:
                thermal_value = incoming_data[6] + incoming_data[7] * 256
                thermal_value = (thermal_value / 8.0) - 256.0
                return thermal_value