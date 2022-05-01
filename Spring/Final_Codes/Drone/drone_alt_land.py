import sys, time, math, argparse, os.path, cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
from signal import pause

from bluedot.btcomm import BluetoothClient
from signal import pause
import time
import sys

global s
global basestatus
global dronestatus
basestatus = "1"

def data_received(data):
    global s
    global dronestatus
    global basestatus
    print(data)
    received = data
    basestatus = received[0]
    
s = BluetoothClient('raspberrypi-talon-base', data_received)

#Establishes connection to the drone.
def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
        print("CONNECTING...")
        return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)

#CONSTANTS_DRONE
CONNECTION_BAUDRATE = 57600
CONNECTION_STRING = '/dev/ttyAMA1'

vehicle = connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE)
print("We R Connected")

@vehicle.on_attribute('armed')
def armed_listener(self, name, msg):
    global s
    global dronestatus
    if msg:
        dronestatus = "1"
        s.send(basestatus+dronestatus)
    else:
        dronestatus = "0"
        s.send(basestatus+dronestatus)

def main():
    try:
        pause()
    except KeyboardInterrupt:
        quit()
if __name__ == "__main__":
    main()
        