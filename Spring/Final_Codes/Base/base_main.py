import sys
from bluedot.btcomm import BluetoothServer
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#PIN_DECLERATIONS
SENSOR_01 = 20
SENSOR_02 = 21
RELAY_01 = 14 #Latch 1
RELAY_02 = 25  #Latch 2
RELAY_03 = 12 #Latch 3
RELAY_04 = 15 #Latch 4

#GPIO Initializations
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SENSOR_01, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(SENSOR_02, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(RELAY_01,GPIO.OUT)
GPIO.setup(RELAY_02,GPIO.OUT)
GPIO.setup(RELAY_03,GPIO.OUT)
GPIO.setup(RELAY_04,GPIO.OUT)

#DEFINE GLOBAL VARIABLES
global s
global dronestatus
global basestatus
global takeoff
dronestatus = "1"
takeoff = "0"

def data_received(data):
    global s
    global dronestatus
    global basestatus
    global takeoff
    print(data)
    received = data
    #basestatus = received[0]
    dronestatus = received[1]
    takeoff = received[2]
    #s.send(basestatus+dronestatus+takeoff)
    

def latch():
    global s
    global dronestatus
    global basestatus
    global takeoff
    print("I am Latching")
    basestatus = "1"
    GPIO.output(RELAY_01,0)
    GPIO.output(RELAY_02,0)
    time.sleep(1)
    GPIO.output(RELAY_03,1)
    GPIO.output(RELAY_04,1)
    time.sleep(15)
    s.send(basestatus+dronestatus+takeoff)

def unlatch():
    global s
    global dronestatus
    global basestatus
    global takeoff
    print("I am Unlatching")
    basestatus = "0"
    GPIO.output(RELAY_04,0)            
    GPIO.output(RELAY_03,0)            
    time.sleep(1)
    GPIO.output(RELAY_02,1)             
    GPIO.output(RELAY_01,1)
    time.sleep(15)
    s.send(basestatus+dronestatus+takeoff)

def main():
    
    #DEFINE GLOBAL VARIABLES
    global s
    global dronestatus
    global basestatus
    global takeoff

    #INSTANTIATE BLUETOOTH
    s = BluetoothServer(data_received)

    unlatch()
    
    while True:
        if(GPIO.input(SENSOR_01) == GPIO.LOW and GPIO.input(SENSOR_02) == GPIO.LOW ):
                sensors = 1
        else:
                sensors = 0
          
        if (dronestatus == "0" and sensors):
            latch()
        elif (takeoff == "1" or dronestatus == "1"):
            unlatch()
        else:
            time.sleep(1)

if __name__ == "__main__":
    main()
