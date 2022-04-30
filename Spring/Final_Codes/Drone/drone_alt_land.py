import sys, time, math, argparse, os.path, cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
from signal import pause

#Establishes connection to the drone.

def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
        print("CONNECTING...")
        return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)

#Callback function for the THR_MIN parameter
def armed_callback(self, attr_name, value):
    print(" PARAMETER CALLBACK:",attr_name,"changed to: " ,self.armed)




def main():
    #CONSTANTS_DRONE
    CONNECTION_BAUDRATE = 57600
    CONNECTION_STRING = '/dev/ttyAMA1'

    vehicle = connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE)
    
    #Add observer for the vehicle's armed parameter
    vehicle.parameters.on_attribute('armed')
    vehicle.parameters.add_attribute_listener('armed', armed_callback)


    pause()



if __name__ == "__main__":
    main()
        