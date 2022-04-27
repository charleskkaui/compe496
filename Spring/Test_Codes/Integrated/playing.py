if distance_vector_x > TARGET_X + precission:
    VELOCITYY = VELOCITY
    print("GO RIGHT")
elif distance_vector_x < TARGET_X - precission:
    VELOCITYY = -VELOCITY
    print("GO LEFT")
else:
    VELOCITYY = 0
    print("L/R OK")

if distance_vector_y > TARGET_Y + precission:
    VELOCITYX = -VELOCITY
    print("GO BACK")
elif distance_vector_y < TARGET_Y - precission:
    VELOCITYX = VELOCITY
    print("GO FORWARD")
else:
    VELOCITYX = 0
    print("F/B OK")

if(VELOCITYX == 0 and VELOCITYY == 0):
    VELOCITYZ = VELOCITY
else:
    VELOCITYZ = 0

fly_go(vehicle,VELOCITYX,VELOCITYY,VELOCITYZ,1) #ROLL_FORWARD

