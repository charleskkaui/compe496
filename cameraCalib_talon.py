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
    libcamera-hello
    cap = cv2.VideoCapture(0)

    #SET THE CAMERA SIZE 1080P OR 720P OR 640X480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
    #cap.set(cv2.CAP_PROP_FPS,30)
    #cap.set(cv2.CAP_PROP_HUE,5)
    cap.set(cv2.CAP_PROP_EXPOSURE,5)


    

    while True:

        # read camera frame
        ret,frame = cap.read()

        cv2.imshow('frame',frame)
        key = cv2.waitKey(1) & 0xFF
        if(key == ord('q')):
            cap.release()
            cv2.destroyAllWindows()
            break






if __name__ == "__main__":
    main()
