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
    ircam = cv2.VideoCapture(0)

    #SET THE CAMERA SIZE 1080P OR 720P OR 640X480
    ircam.set(cv2.CAP_PROP_FRAME_WIDTH,640)
    ircam.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

    

    while True:

        # read camera frame
        ret,frame = ircam.read()

        cv2.imshow('frame',frame)
        key = cv2.waitKey(1) & 0xFF
        if(key == ord('q')):
            ircam.release()
            cv2.destroyAllWindows()
            break






if __name__ == "__main__":
    main()
