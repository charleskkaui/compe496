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

def data_received(data):
    global received
    print(data)
    received = data
    time.sleep(1)
    s.send("TX Received")

def latch():
    print("I am Latching")
    GPIO.output(RELAY_01,0)
    GPIO.output(RELAY_02,0)
    time.sleep(1)
    GPIO.output(RELAY_03,1)
    GPIO.output(RELAY_04,1)

def unlatch():
    print("I am Unlatching")                    
    GPIO.output(RELAY_04,0)            
    GPIO.output(RELAY_03,0)            
    time.sleep(1)
    GPIO.output(RELAY_02,1)             
    GPIO.output(RELAY_01,1)

def main():
    
    #DEFINE GLOBAL VARIABLES
    global s 
    global received 

    #INSTANTIATE BLUETOOTH
    s = BluetoothServer(data_received)
    
    received = "0"
    
    while True:
        #COPY CODE TO CHECK SENSORS HERE
        ################################
        ################################
        if(SENSOR_01 == GPIO.LOW and Sensor_02 == GPIO.LOW ):
                sensors = 1
        else:
                sensors = 0
            
        if (received == "A" and sensors): #WE ARE LATCHING
            latch()
            received = "0"
        elif (received == "B"):
            unlatch()
            received = "0"
        else:
            print("Idle")
            time.sleep(1)

if __name__ == "__main__":
    main()