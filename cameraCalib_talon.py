import numpy
import cv2
import cv2.aruco as aruco
import sys
import time
import math

id_to_find = 75
aruco_marker_size = 10.3 #(in cm)

def main():

    #CAPTURE THE CAMERA?
    cap = cv2.VideoCapture(0)

    #SET THE CAMERA SIZE 1080P OR 720P OR 640X480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    #cap.set(cv2.CAP_PROP_FPS,95)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,95)
    #cap.set(cv2.CAP_PROP_EXPOSURE,95)



    

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


if __name__ == "__main__":
    main()