from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time,sys,argparse,math
#ADD BLUETOOTH CLIENT

CONNECTION_BAUDRATE = 57600
CONNECTION_STRING = '127.0301:14550' #COnnection String for the Drone

def connect_drone():
    print("CONNECTING...")
    return connect(CONNECTION_STRING_DRONEKITSITL, wait_ready=True)#, baud=BAUDRATE)
    #connect() utilizes another function vehicle.wait_ready() which prevents the return of connect() until the connection is confirmed 

def arm(vehicle):
    print("ARMING DRONE")
    vehicle.mode = "GUIDED"
    time.sleep(1)
    vehicle.armed = True
    time.sleep(1)
    print("DRONE ARMED")

def disarm(vehicle):
    print("DISARMING DRONE")
    time.sleep(1)
    vehicle.armed = False
    time.sleep(1)
    print("DRONE DISARMED")

def take_off_now(vehicle,TargetAltitude):
    print("ARMING DRONE")
    print("DRONE ARMED")
    arm(vehicle)
    time.sleep(1)
    vehicle.simple_takeoff(TargetAltitude)
    while vehicle.location.global_relative_frame.alt<=TargetAltitude*0.95:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(5)
    print("ALTITUDE REACHED")

def main():
    #DEFINE GLOBAL VARIABLES
    global s 
    global received 

    #INSTANTIATE BLUETOOTH
    s = BluetoothServer(data_received)
    
    received = "0"

    vehicle = connect_drone()


    take_off_now(vehicle,ALTITUDE)


if __name__ == "__main__":
    main()