from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time,sys,argparse,math
#ADD BLUETOOTH CLIENT

def data_received(data):
    global received
    received = data
    time.sleep(1)
    s.send("0")

def main():
    #DEFINE GLOBAL VARIABLES
    global s 
    global received 

    #INSTANTIATE BLUETOOTH
    s = BluetoothServer(data_received)
    
    received = "0"

    vehicle = connect_drone()

    while True:
        if vehicle.armed == False:
            s.send("A")
        
if __name__ == "__main__":
    main()
