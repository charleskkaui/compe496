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

def data_received(data):
    global s
    global dronestatus
    global basestatus
    print(data)
    received = data
    basestatus = received[0]
    #dronestatus = received[1]
    s.send(basestatus+dronestatus)
    

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
    print(name," attribute is: ", msg)
    if msg == True:
        dronestatus = 1
        s.send(basestatus+dronestatus)
    else:
        dronestatus = 0
        s.send(basestatus+dronestatus)

def main():
    pause()

if __name__ == "__main__":
    main()
        