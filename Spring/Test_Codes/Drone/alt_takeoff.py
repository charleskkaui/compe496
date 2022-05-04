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
global takeoff
global dronestatus
basestatus = "1"
takeoff = "0"
dronestatus = "1"

def data_received(data):
    global s
    global basestatus
    global dronestatus
    global takeoff
    print(data)
    received = data
    basestatus = received[0]
    
s = BluetoothClient('raspberrypi-talon-base', data_received)

def main():
    global s
    global basestatus
    global dronestatus
    global takeoff
    try:
        takeoff = "1"
        s.send(basestatus+dronestatus+takeoff)        
        while True:
            print("Waiting for flight Clearence...")
            time.sleep(1)
            if basestatus == "0":
                print("TAKING OFF")
                quit()
    except KeyboardInterrupt:
        quit()
if __name__ == "__main__":
    main()
        