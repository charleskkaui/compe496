from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time,sys,argparse,math

#sim_vehicle.py -v ArduCopter -f quad --map --console -L SDSU
CONNECTION_BAUDRATE = 57600
CONNECTION_STRING = '/dev/ttyAMA1' #COnnection String for the Drone
CONNECTION_STRING_DRONEKITSITL = 'udp:127.0.0.1:14551' #Connection String for the Simulation
VELOCITY = 1
DURATION = 1
HEADING = 180
ALTITUDE = 1

def connect_drone():
    print("CONNECTING...")
    return connect(CONNECTION_STRING, wait_ready=True, baud=CONNECTION_BAUDRATE)
    #connect() utilizes another function vehicle.wait_ready() which prevents the return of connect() until the connection is confirmed 

def arm(vehicle):
    print("ARMING DRONE")
    vehicle.mode = "GUIDED"
    time.sleep(1)
    vehicle.armed = True
    time.sleep(1)
    print("DRONE ARMED")

def disarm(vehicle):
    print("DISARMING DRONE")
    time.sleep(1)
    vehicle.armed = False
    time.sleep(1)
    print("DRONE DISARMED")

def take_off_now(vehicle,TargetAltitude):
    vehicle.simple_takeoff(TargetAltitude)
    while vehicle.location.global_relative_frame.alt<=TargetAltitude*0.70:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(5)
    print("ALTITUDE REACHED")

def land_now(vehicle):
    print("Im supposed to land")
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
    vehicle = connect_drone()
    arm(vehicle)
    take_off_now(vehicle,2)   
    #vehicle.mode = "LOITER"
    time.sleep(2)
    fly_go(vehicle,0,0,0,1)
    fly_spin(vehicle,0,False)
    time.sleep(2)
    fly_go(vehicle,1,0,0,1)
    time.sleep(2)
    fly_go(vehicle,-1,0,0,1)
    time.sleep(2)
    fly_go(vehicle,0,1,0,1)
    time.sleep(2)
    fly_go(vehicle,0,-1,0,1)
    time.sleep(2)
    #fly_go(vehicle,-VELOCITY,0,0,DURATION)
    #fly_spin(vehicle,HEADING)
    
    #fly_go(vehicle,0,VELOCITY,0,DURATION)
    #fly_go(vehicle,0,-VELOCITY,0,DURATION)
    fly_spin(vehicle,0)
    #time.sleep(15)
    #send_global_velocity(0, VELOCITY, 0, DURATION)
    #send_global_velocity(0, VELOCITY, 0, DURATION)
    land_now(vehicle)
    disarm(vehicle)
    print("End of Script")



if __name__ == "__main__":
    main()
