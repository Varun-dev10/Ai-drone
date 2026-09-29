import serial 
import time 
import numpy as np

# Serial port handler for LIDAR interaction
port_handler = None

def activate_lidar_link(port_identifier):
    # Initiates a serial connection to the LIDAR device.
    global port_handler
    port_handler = serial.Serial(port_identifier, 115200, timeout=0)
    if port_handler.isOpen() == False:
        port_handler.open()
        return "successful"
    else:
        return "already_active"

def deactivate_lidar_link():
    # Terminates the serial connection to the LIDAR device.
    global port_handler
    port_handler.close()

def verify_lidar_link():
    # Confirms if the LIDAR serial connection is active.
    global port_handler
    return port_handler.isOpen()

def obtain_lidar_measurements():
    # Retrieves distance and signal strength from the LIDAR device.
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

def read_lidar_temp():
    # Retrieves the temperature reading from the LIDAR device.

    global port_handler
    while True:
        pending_bytes = port_handler.in_waiting
        if pending_bytes > 8:
            incoming_data = port_handler.read(9)
            port_handler.reset_input_buffer()
            if incoming_data[0] == 0x59 and incoming_data[1] == 0x59:
                temp = incoming_data[6] + incoming_data[7] * 256
                temp = (temp / 8.0) - 256.0
                return temp