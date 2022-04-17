import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time

#GPIO Initializations
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(4,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)

                             #limit switch is value 0 stating the switch is not pressed
print("Switch off")          #print statement saying the state it is in
GPIO.output(16,0)            #is relay 1 that makes the base arms retract for arm 1
GPIO.output(13,0)            #is relay 2 that makes the base arm retract for arm 2
time.sleep(.1)
GPIO.output(4,1)             #is relay 
GPIO.output(12,1)