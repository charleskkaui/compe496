import sys
from bluedot.btcomm import BluetoothServer
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#GLOBAL_VARIABLES
global received

#INSTANTIATE BLUETOOTH
s = BluetoothServer(data_received)

##PIN_DECLERATIONS
#SENSOR_01 = 20
#SENSOR_02 = 21
#RELAY_01 = 16 #Latch 1
#RELAY_02 = 4  #Latch 2
#RELAY_03 = 12 #Latch 3
#RELAY_04 = 13 #Latch 4

##GPIO Initializations
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#GPIO.setup(SENSOR_01, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
#GPIO.setup(SENSOR_02, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
#GPIO.setup(RELAY_01,GPIO.OUT)
#GPIO.setup(RELAY_02,GPIO.OUT)
#GPIO.setup(RELAY_03,GPIO.OUT)
#GPIO.setup(RELAY_04,GPIO.OUT)

def data_received(data):
    print(data)
    received = data
    time.sleep(1)
    s.send("TX Received")

def latch():
    print("I am Latching")
    #GPIO.output(4,0)
    #GPIO.output(12,0)
    #time.sleep(1)
    #GPIO.output(16,1)
    #GPIO.output(13,1)

def unlatch():
    print("I am Unlatching")                    
    #GPIO.output(16,0)            
    #GPIO.output(13,0)            
    #time.sleep(1)
    #GPIO.output(4,1)             
    #GPIO.output(12,1)


def main():
    received = "0"
    while True:
        #COPY CODE TO CHECK SENSORS HERE
        ################################
        ################################

        if "A" in received: #WE ARE LATCHING
            latch()
            received = "0"
        elif "B" in received:
            unlatch()
            received = "0"
        else:
            print("Idle")
            
        time.sleep(5)

if __name__ == "__main__":
    main()