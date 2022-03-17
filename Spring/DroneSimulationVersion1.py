import rospy
from sensor_msgs.msg import Image
import cv2
import cvs.arcuo as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp #github repsoitry to numpify stuff butim not using it im using the one beow instead
import rospy.numpy_msg import numpy_msg

#--- Define Tag
id_to_find  = 7
marker_size  = 10 #- [cm]

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters  = aruco.DetectorParameters_create()

#---Create Publisher
newimg_pub = rospy.Publisher('/camera/color/image_new', numpy_msg(Image), queue_size = 10)

#-- Set the camera size as the one it was calibrated with !!!!!!!!!!!!!!!!!!!
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#--- Get the camera calibration path !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

time_last = 0
time_to_wait = 0.1 #100ms



def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/camera/color/image_raw', numpy_msg(Image), callback)
    rospy.spin()

def callback(data):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find

    if time.time() - time_last > time_to_wait:
        #-- Convert in gray scale
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
        
        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix, distCoeff=camera_distortion)
    
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            #-- Get the Velocity Vectors for X and Y assuming DURATION for t. v=d/t
            distance_vector_x[0] = abs(pos_camera[0]-tvec[0])
            distance_vector_y[1] = abs(pos_camera[1]-tvec[1])
            distance_vector_z[2] = abs(pos_camera[2]-tvec[2])
            distance_vector = math.sqrt(math.pow(distance_vector_x,2) + math.pow(distance_vector_y,2))
            #if distance_vector 
            distance_vector_disp = "Drone must travel: x=%4.0f  y=%4.0f  z=%4.0f"%(distance_vector[0], distance_vector[1], distance_vector[2])
            cv2.putText(frame, distance_vector_disp, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        newimg_pub.publish(message)




if __name__ == "__main__":
    main()