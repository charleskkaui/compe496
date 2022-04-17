from time import sleep
import RPi.GPIO as GPIO

DIR=10
STEP=8
SPR=400
CW=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR,CW)

step_count=SPR
delay=0.010

for x in range(step_count):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    
GPIO.cleanup()