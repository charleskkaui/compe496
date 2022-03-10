import RPi.GPIO as GPIO
import time


DIR = 20                        #Direction of GPIO Pin
STEP = 21                       #Step GPIO Pin


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DIR, GPIO.OUT)       #Direction pin to output
GPIO.setup(STEP, GPIO.OUT)      #Step pin as output
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off) 

delay = 0.001

def steps(n):
    count = 0
    while count<n:
        GPIO.output(STEP,GPIO.HIGH)   #setting the code to HIGH makes it go CW
        time.sleep(delay)
        GPIO.output(STEP,GPIO.LOW)    #setting the code to LOW makes it go CCW
        time.sleep(delay)
        count +=1


count = 0
done = 0
while True:
    
    if GPIO.input(17)==GPIO.LOW:     #if the state for switch pin is pressed or GPIO.LOW then limit switch (ls)=1 
        if count >= 10:
            ls=1                     #declared value of limit switch
            print("Switch pushed")       #print statement saying which state it is in
        else:
            time.sleep(1)
            count++
    else:
        ls=0                         #limit switch is value 0 stating the switch is not pressed
        count = 0
        print("Switch off")          #print statement saying the state it is in
        time.sleep(.1)

    if ls and not done:      #drone and ls:                  #ls = limit switch
        GPIO.output(DIR,GPIO.HIGH)   #this direction goes clockwise 
        steps(100)                   # steps (200)=360 degree turn, steps (50) is 90 degree turn with [no load]
        done = 1