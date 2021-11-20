import numpy
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import picamera

id_to_find = 75
aruco_marker_size = 10.3 #(in cm)

def oldCode():
    while True:
    
        # read camera frame
        ret, rawfootage = cap.read()
        mycolor = cv2.cvtColor(rawfootage, cv2.COLOR_BGR2RGB)
        mycolor2 = cv2.cvtColor(rawfootage, cv2.COLOR_BGR2GRAY)
        mycolor3 = cv2.cvtColor(rawfootage, cv2.COLOR_BGR2HSV)
        mycolor4 = cv2.cvtColor(rawfootage, cv2.COLOR_BGRA2RGBA)
    
        
    
        cv2.imshow('Raw',rawfootage)
        cv2.imshow('BGR to RGB',mycolor)
        cv2.imshow('BGR to Gray',mycolor2)
        cv2.imshow('BGR to HSV',mycolor3)
        cv2.imshow('BGRA to RGBA',mycolor4)
        key = cv2.waitKey(1) & 0xFF
        if(key == ord('q')):
            cap.release()
            cv2.destroyAllWindows()
            break

def main():

    #CAPTURE THE CAMERA?
    #cap = cv2.VideoCapture(0)
    camera = PiCamera()
    camera.resolution(640,480)
    camera.framerate = 30
    camera.brightness = 65

    raw_footage = PiRGBArray(camera, size(640,480))


    #SET THE CAMERA SIZE 1080P OR 720P OR 640X480
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    #cap.set(cv2.CAP_PROP_FPS,60)
    #cap.set(cv2.CAP_PROP_BRIGHTNESS,100)
    #cap.set(cv2.CAP_PROP_EXPOSURE,95)
    #cap.set(cv2.CAP_PROP_CONVERT_RGB,1)


    for frame in camera.capture_continuous(raw_footage, format="rgb", use_video_port=True):
        img = frame.array
        cv2.imshow("Original Image", img)
        raw_capture.truncate(0)
        if cv2.waitKey(1) & 0xFF==ord('q'):
            break
    

    #oldCode()


if __name__ == "__main__":
    main()