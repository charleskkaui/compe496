import time
import RPi.GPIO as GPIO

DIR =  20                     #Direction of GPIO Pin
STEP = 21                      #Step GPIO Pin

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(DIR, GPIO.OUT)       #Direction pin to output
GPIO.setup(STEP, GPIO.OUT)      #Step pin as output

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
        GPIO.output(DIR,GPIO.HIGH)   #this direction goes clockwise 
        steps(1200)                   # steps (200)=360 degree turn, steps (50) is 90 degree turn with [no load]
        time.sleep(1)
