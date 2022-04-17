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

while True:
    
    if GPIO.input(17)==GPIO.HIGH:     #if the state for switch pin is pressed or GPIO.LOW then limit switch (ls)=1 
        ls=1                         #declared value of limit switch
        print("Switch pushed")       #print statement saying which state it is in
    else:
        ls=0                         #limit switch is value 0 stating the switch is not pressed
        print("Switch off")          #print statement saying the state it is in
        time.sleep(.1)

    if ls:      #drone and ls:                  #ls = limit switch
        time.sleep(0)
        GPIO.output(DIR,GPIO.LOW)   #this direction goes clockwise 
        steps(800)                   #3200 pulse/rev 800 per 45 degree turns 
      
        while GPIO.input(17)==GPIO.HIGH: #while loop that checks if the switch is still pressed. 
            time.sleep(.1)
            steps(0)                    #if the switch is still pressed then state is low and steps are reduced to (0)
            if GPIO.input(17)==GPIO.LOW:
                break

        
#B2=yellow
#B1=blue
#A1=green
#A2=red