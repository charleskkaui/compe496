from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time,sys,argparse,math

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

    while True:
        if vehicle.armed:
            s.send("B")
        elif not vehicle.armed:
            s.send("A")

if __name__ == "__main__":
    main()
