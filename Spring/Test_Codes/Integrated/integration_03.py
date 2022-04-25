import sys, time, math, argparse, os.path, cv2
import numpy as np
import cv2.aruco as aruco
from datetime import datetime
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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

# Establishes connection to the drone.
def connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE):
    print("CONNECTING...")
    return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)

def arm(vehicle):
    print("ARMING DRONE...")
    vehicle.mode = "GUIDED"
    time.sleep(1)
    vehicle.armed = True
    time.sleep(1)
    print("DRONE ARMED")

def disarm(vehicle):
    print("DISARMING DRONE...")
    time.sleep(1)
    vehicle.armed = False
    time.sleep(1)
    print("DRONE DISARMED")

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
    
def land_now(vehicle):
    print("LANDING...")
    vehicle.mode = "LAND"

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
    #CONSTANTS_DRONE
    CONNECTION_BAUDRATE = 57600
    CONNECTION_STRING = '/dev/ttyAMA1' #COnnection String for the Drone
    VELOCITY = 0.5
    ALTITUDE_TAKEOFF = 2
    #CONSTANTS_ARUCO_POSITION
    TARGET_X = 8
    TARGET_Y = 15
    TARGET_Z = -150
    #CONSTANTS_PRECISSION_LANDING
    TARGET_PRECISSION = 7
    precission = 20
    corrector = 0

    vehicle = connect_drone(CONNECTION_STRING,CONNECTION_BAUDRATE)
    arm(vehicle)
    take_off_now(vehicle,ALTITUDE_TAKEOFF)

    #creating file for data
    now = datetime.now()
    save_path = "/home/pi/compe496/Spring/Test_Codes/OpenCV/TestData"
    file_name = "DistanceVector_"+now.strftime("%Y")+"_"+now.strftime("%m")+"_"+now.strftime("%d")+"_"+now.strftime("%H")+"_"+now.strftime("%M")+".txt"
    complete_filename = os.path.join(save_path,file_name)
    
    try:
        myfile = open(complete_filename,'w')
    except:
        exit("The File cannot be created")

    velocityx = 0
    velocityy = 0
    velocityz = 0
    distance_vector_x = np.zeros(1)
    distance_vector_y = np.zeros(1)
    distance_vector_z = np.zeros(1)
    distance_x_arr = np.zeros(10)
    distance_y_arr = np.zeros(10)
    distance_z_arr = np.zeros(10)
    count=0
    average_vector = (0,0,0)
    error_level = 0
    error_level_0_cnt = 2
    zmin = -5000



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
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            #-- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Get the Velocity Vectors for X and Y assuming DURATION for t. v=d/t
            distance_vector_x = (pos_camera[0]-tvec[0])
            distance_vector_y = (pos_camera[1]-tvec[1])
            distance_vector_z = (pos_camera[2]-tvec[2])
            #myfile.write(np.array2string(distance_vector_x)+","+np.array2string(distance_vector_y)+","+np.array2string(distance_vector_z)+"\n")
            
            #distance_vector_disp = "Raw Drone Travel Values: x=%4.0f  y=%4.0f  z=%4.0f"%(distance_vector_x, distance_vector_y, distance_vector_z)
            #cv2.putText(frame, distance_vector_disp, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            if count >= 9:
                arr = np.sort(distance_x_arr)
                average_x = np.average(arr[2:8])
                arr = np.sort(distance_y_arr)
                average_y = np.average(arr[2:8])
                arr = np.sort(distance_z_arr)
                average_z = np.average(arr[2:8])
                average_vector = (average_x,average_y,average_z*-1)
                count = 0
                distance_x_arr[count] = distance_vector_x
                distance_y_arr[count] = distance_vector_y
                distance_z_arr[count] = distance_vector_z
            else:
                count+=1
                distance_x_arr[count] = distance_vector_x
                distance_y_arr[count] = distance_vector_y
                distance_z_arr[count] = abs(distance_vector_z)

            average_vector[0] = distance_vector_x
            average_vector[1] = distance_vector_y
            average_vector[2] = distance_vector_z

            print(average_vector[0] ,"is of type ",type(average_vector[0]))

        
            #distance_vector_disp = "Drone must travel: x=%4.0f  y=%4.0f  z=%4.0f"%(average_vector[0],average_vector[1],average_vector[2])
            #cv2.putText(frame, distance_vector_disp, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            if average_vector[2] != 0:
                print("Precission: ", precission, end=" :")
                if average_vector[2] > TARGET_Z or vehicle.mode.name == "LAND":
                    print("TIME TO LAND")
                    land_now(vehicle)
                    cap.release()
                    cv2.destroyAllWindows()
                    myfile.close()
                else:
                    if average_vector[0] > TARGET_X + precission:
                        velocityy = VELOCITY
                        print("GO RIGHT", end=" :")
                    elif average_vector[0] < TARGET_X - precission:
                        velocityy = -VELOCITY
                        print("GO LEFT", end=" :")
                    else:
                        velocityy = 0
                        print("L/R OK", end=" :")

                    if average_vector[1] > TARGET_Y + precission:
                        velocityx = -VELOCITY
                        print("GO BACK", end=" :")
                    elif average_vector[1] < TARGET_Y - precission:
                        velocityx = VELOCITY
                        print("GO FORWARD", end=" :")
                    else:
                        velocityx = 0
                        print("F/B OK", end=" :")

                    if(velocityx == 0 and velocityy == 0):
                        velocityz = VELOCITY
                        print("GO DOWN")
                    elif average_vector[2] > zmin-precission-5:
                        velocityz = VELOCITY
                        print("GO DOWN CUZ UP")
                    else:
                        velocityz = 0

                    fly_go(vehicle,velocityx,velocityy,velocityz,1)
                    fly_go(vehicle,0,0,0,1)
                    time.sleep(1)
                    
                    
                
                if  average_vector[2] < -500:
                    precission = 25
                elif average_vector[2] < -400:
                    precission = 20
                elif average_vector[2] < -300:
                    precission = 15
                elif average_vector[2] < -200:
                    precission = 10
                else:
                    precission = 5

                if zmin > average_vector[2] and average_vector[2] > -150:
                    zmin = average_vector[2]

                error_level = 0
                error_level_0_cnt = 2

        else:
            if average_vector[2] != 0:
                ###IF WE CANNOT FIND THE ARUCO WE SHOULD GO BACK THE LAST DIRECTION FOR 2s
                if error_level == 0:
                    print("LOST ARUCO ERROR LEVEL: ",error_level," cnt: ", error_level_0_cnt,end=" ")
                    fly_go(vehicle,velocityx*-1,velocityy*-1,0,1)
                    fly_go(vehicle,0,0,0,1)
                    time.sleep(1)
                    #if error_level_0_cnt >= 2:
                    #    fly_go(vehicle,velocityx*-1,velocityy*-1,0,1)
                    #    error_level_0_cnt = 1
                    #elif error_level_0_cnt == 1:
                    #    fly_go(vehicle,velocityx*-1,velocityy*-1,0,1)
                    #    error_level_0_cnt = 0
                    #else:
                    #    fly_go(vehicle,velocityx,velocityy,velocityz,2)
                    #    error_level = 1
                    #    print("LOST FOREVER")
                    #    land_now(vehicle)
                #elif error_level == 1:
                #    fly_go(vehicle,0,0,0,1)
                #    if error_level_0_cnt < 2:
                #        error_cnt += 1
                #    else: 
                #        error_level = 2
                #elif error_level == 2:
                #    print("REALLY LOST...")
                #    fly_go(vehicle,0,0,VELOCITY,1)
                #    time.sleep(2)
                #elif error_level == 3:
                #    print("SEARCHING FOR MARKER")
                #    fly_go(vehicle,0,VELOCITY,0,1)


        #--- Display the frame
        #cv2.imshow('frame', frame)
        #print(np.array2string(distance_vector_x)+","+np.array2string(distance_vector_y)+","+np.array2string(distance_vector_z)+"\n")
        print("\n",average_vector)

        #--- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            myfile.close()
            break

if __name__ == "__main__":
    main()
