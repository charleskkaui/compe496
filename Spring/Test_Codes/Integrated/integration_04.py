#The following code uses portions of a modified OpenCV 4.5.5 version of Tiziano's OpenCV Drone Landing script.
#Alterations were made to make the OpenCV visual data collection compatible with OpenCV 4.5.5 as the source code was outdated and from a previous version.
#The slightly altered compoenents from Tizianos OpenCV is used to obtain the camera position relative to an Aruco Marker.
#From the location data collected, Charles Kaui extracts the XYZ distances of the camera relative to the marker.
#The camera running at 30fps supplies 30 distance vectors every second.
#From the OpenCV data, a sample is taken every 15 frames or 0.5s.
#The X and Y components of the sample is then fed into their own PID controllers.
#Each PID recieves a distance vector component and outputs a calculated speed.
#The speed components are then passed to the fly_go method.
#The fly_go method accepts 3 velocity vector compoenents and a duration in seconds to perform the movement.
#The fly_go method translates the input arguments into a mavlink message then sends the message to the flight controler of the drone.
#The drone then performs the desired movment.
#This process repeats until the drone is centered over the Aruco Marker.
#When the X and Y errors caluclated by the PID is below their error_thresholds, the Z component is set to lower the drone
#This process is repeated until the Z reaches z_minimum.
#At z_minimum the drone uses the land_now definition to land the drone.
#Upon landing the Drone Disarms and the script ends.

#Resources
#PID https://renegaderobotics.org/pid-beginners-guide/#basicCode
#OpenCV https://github.com/tizianofiorenzani/how_do_drones_work
#Tiziano's GitHub https://opencv.org/opencv-4-5-5/
#Dronekit-Python https://dronekit-python.readthedocs.io/en/latest/

import sys, time, math, argparse, os.path, cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil

#Referenced from PID resource
def myPID(pid_in, target, last_error, integral, kP, kI, kD, min_pid_out, max_pid_out):
    #speed_xyz, last_error_xy, integral_xy, derivative_xy myPID(pid_in, target, last_error, integral, kP, kI, kD, min_pid_out, max_pid_out)
    error = target - pid_in;
    integral = integral + error;
    derivative = error - lastError;

    proportional_adjustment = error * kP 
    integral_adjustment = integral * kI
    derivative_adjustment = derivative * kD

    pid_out = proportional_adjustment + integral_adjustment + derivative_adjustment
    return max(min(max_pid_out, pid_out), min_pid_out) , error , integral, derivative

#Sourced from Tiziano
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

#Sourced from Tiziano
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

#Referenced from Dronekit-Python
def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
    try:
        print("CONNECTING...")
        return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)
    except:
        print("Conection Failure")

#Referenced from Dronekit-Python
def arm(vehicle):
    vehicle.mode = "GUIDED"
    #vehicle.is_armable  ##try or while or if
    vehicle.armed = True
    while(vehicle.armed == False):
        print("ARMING DRONE...")
        time.sleep(1)
    print("DRONE ARMED")

#Referenced from Dronekit-Python
def disarm(vehicle):
    vehicle.mode = "GUIDED"
    vehicle.armed = False
    while(vehicle.armed == True):
        print("DISARMING DRONE...")
        time.sleep(1)
    print("DRONE DISARMED")

#Referenced from Dronekit-Python
def take_off_now(vehicle,TargetAltitude):
    print("Ascending...")
    vehicle.simple_takeoff(TargetAltitude)
    while vehicle.location.global_relative_frame.alt<=TargetAltitude*0.90:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    #By defualt the drone wants to yaw into the direction of travel.
    #Adjusting the yaw manually allows drone to sideward hover.
    fly_go(vehicle,0,0,0,1)
    fly_spin(vehicle,0,True)
    time.sleep(1)
    print("ALTITUDE REACHED: ",vehicle.location.global_relative_frame.alt)

#Referenced from Dronekit-Python
def land_now(vehicle):
    print("LANDING...")
    vehicle.mode = "LAND"

#Referenced from Dronekit-Python
def fly_go(vehicle,velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0,
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)

    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#Referenced from Dronekit-Python
def fly_spin(vehicle,heading, relative=False):
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def main():
    #Charles Kaui Code
    #CONSTANTS_DRONE
    CONNECTION_BAUDRATE = 57600
    CONNECTION_STRING = '/dev/ttyAMA1' #COnnection String for the Drone
    SPEED_Z = 0.1
    SPEED_XY_MAX = 1
    SPEED_XY_MIN = 0

    #CONSTANTS_ARUCO_TARGET_POSITION
    TARGET_X = 8
    TARGET_Y = 15
    TARGET_Z = -150

    #CONSTANTS PID
    KP_X = 0
    KP_Y = 0
    KI_X = 0
    KI_Y = 0
    KD_X = 0
    KD_Y = 0

    #Charles Kaui Code
    distance_vector = [0,0,0]
    last_error_x = 0
    integral_x = 0
    last_error_y = 0
    integral_y = 0

    try:
        vehicle = connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE)
    
        #creating file for data
        now = datetime.now()
        save_path = "/home/pi/compe496/Spring/Test_Codes/OpenCV/TestData"
        file_name = "PID_"+now.strftime("%Y")+"_"+now.strftime("%m")+"_"+now.strftime("%d")+"_"+now.strftime("%H")+"_"+now.strftime("%M")+".txt"
        complete_filename = os.path.join(save_path,file_name)
    
        try:
            myfile = open(complete_filename,'w')
        except:
            exit("The File cannot be created")
            quit()

        #Following Section is Sourced from Tiziano
        #--- Define Tag
        id_to_find  = 325
        marker_size  = 8 #- [cm]

        #--- Get the camera calibration path
        calib_path  = "~/home/pi/"
        cameraMatrixfile = "/home/pi/cameraMatrix_webcam.txt"
        cameraDistortionfile = "/home/pi/cameraDistortion_webcam.txt"
        camera_matrix   = np.loadtxt(cameraMatrixfile, delimiter=',')#(calib_path+"cameraMatrix_webcam.txt", delimiter=',')
        camera_distortion   = np.loadtxt(cameraDistortionfile, delimiter=',')

        #--- 180 deg rotation matrix around the x axis
        R_flip  = np.zeros((3,3), dtype=np.float32)
        R_flip[0,0] = 1.0
        R_flip[1,1] =-1.0
        R_flip[2,2] =-1.0

        #--- Define the aruco dictionary
        aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        parameters  = aruco.DetectorParameters_create()

        #--- Capture the videocamera (this may also be a video or a picture)
        cap = cv2.VideoCapture(0)

        #-- Set the camera size as the one it was calibrated with
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        #-- Font for the text in the image
        font = cv2.FONT_HERSHEY_PLAIN

        while True:
            #-- Read the camera frame
            ret, frame = cap.read()

            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    
            if ids is not None and ids[0] == id_to_find:

                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)

                #-- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

                #-- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc*np.matrix(tvec).T

                #-- Get the attitude of the camera respect to the frame
                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

                #End of Tiziano Sourcing

                #Charles Kaui Code
                #-- Get the Distance Vector for X and Y and Z.
                distance_vecotr[0] = (pos_camera[0]-tvec[0])
                distance_vector[1] = (pos_camera[1]-tvec[1])
                distance_vector[2] = (pos_camera[2]-tvec[2])
            
                if samplerate_counter > 30:
            
                    speed_x, last_error_x, integral_x, derivative_x  = myPID(distance_vector[0], TARGET_X, last_error_x, integral_x, KP_X, KI_X, KD_X, SPEED_XY_MIN, SPEED_XY_MAX)
                    speed_y, last_error_y, integral_y, derivative_y  = myPID(distance_vector[1], TARGET_Y, last_error_y, integral_y, KP_Y, KI_Y, KD_Y, SPEED_XY_MIN, SPEED_XY_MAX)
            
                    if abs(last_error_x) < error_threshold and abs(last_error_y) < error_threshold:
                        speed_z = SPEED_Z_MAX
                    else:
                        speed_z = 0

                    fly_go(vehicle,speed_x,speed_y,speed_z,0)
                    samplerate_counter = 0
                else:
                    samplerate_counter += 1      
                
            else:
                print("LOST ARUCO")

    except KeyboardInterrupt:
        cap.release()
        cv2.destroyAllWindows()
        myfile.close()
        quit()

if __name__ == "__main__":
    main()
