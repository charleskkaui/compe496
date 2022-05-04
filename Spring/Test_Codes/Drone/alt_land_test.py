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
global takeoff
takeoff = "0"
basestatus = "1"
dronestatus = "1"

def data_received(data):
    global s
    global dronestatus
    global basestatus
    global takeoff
    
    print(data)
    received = data
    basestatus = received[0]
    takeoff = received[2]
    
s = BluetoothClient('raspberrypi-talon-base', data_received)


def main():
    try:
        while True:
            s.send(basestatus+input()+takeoff)
        pause()
    except KeyboardInterrupt:
        quit()
if __name__ == "__main__":
    main()
        