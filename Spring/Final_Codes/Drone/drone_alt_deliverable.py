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
    s.send(basestatus+dronestatus+takeoff)
    

s = BluetoothClient('raspberrypi-talon-base', data_received)

def base_takeoff(vehicle):
    global takeoff
    global basestatus
    ALTITUDE_TAKEOFF = 2
    takeoff = "1"
    s.send(basestatus+dronestatus+takeoff)        
    while True:
        print("Waiting for flight Clearence...")
        time.sleep(1)
        if basestatus == "0":
            arm(vehicle)
            take_off_now(vehicle,ALTITUDE_TAKEOFF)
            fly_go(vehicle,2,0,0,2)
            fly_go(vehicle,0,0,0,1)
            land_now(vehicle)
            print("Take off succseful")
            s.disconnect()
            quit()

    
#Establishes connection to the drone.
def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
        print("CONNECTING...")
        return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)

#CONSTANTS_DRONE
CONNECTION_BAUDRATE = 57600
CONNECTION_STRING = '/dev/ttyAMA1'
global vehicle
vehicle = connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE)
print("We R Connected")

@vehicle.on_attribute('armed')
def armed_listener(self, name, msg):
    global s
    global dronestatus
    if msg:
        dronestatus = "1"
        print("Sending: ",basestatus+dronestatus+takeoff)
        s.send(basestatus+dronestatus+takeoff)
    else:
        dronestatus = "0"
        print("Sending: ",basestatus+dronestatus+takeoff)
        s.send(basestatus+dronestatus+takeoff)
        #time.sleep(5)
        #s.disconnect()
        #quit()

#Establishes connection to the drone.
def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
        print("CONNECTING...")
        return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)

def arm(vehicle):
    vehicle.mode = "GUIDED"
    #vehicle.is_armable  ##try or while or if
    vehicle.armed = True
    while(vehicle.armed == False):
        print("ARMING DRONE...")
        time.sleep(1)
    print("DRONE ARMED")

#Referenced from Dronekit-Python
def fly_go(vehicle,velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0,
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)

    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#Referenced from Dronekit-Python
def fly_spin(vehicle,heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def take_off_now(vehicle,TargetAltitude):
    print("Ascending...")
    vehicle.simple_takeoff(TargetAltitude)
    while vehicle.location.global_relative_frame.alt<=TargetAltitude*0.90:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    #By defualt the drone wants to yaw into the direction of travel.
    #Adjusting the yaw manually allows drone to sideward hover.
    fly_go(vehicle,0,0,0,1)
    fly_spin(vehicle,0,True)
    time.sleep(1)
    print("ALTITUDE REACHED: ",vehicle.location.global_relative_frame.alt)

def land_now(vehicle):
    print("LANDING...")
    vehicle.mode = "LAND"

def main():
    global s
    global basestatus
    global dronestatus
    global takeoff
    global vehicle
    
    try:
        while True:
            if dronestatus == "0" and basestatus == "1":
                time.sleep(5)
                base_takeoff(vehicle)
                print("Initiaite Take off!!!!!")
                quit()
    except KeyboardInterrupt:
        s.disconnect()
        quit()
if __name__ == "__main__":
    main()
        