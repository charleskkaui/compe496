durationtime = 1
search_direction = 1
while ids[0] != id_to_find:
    if search_direction == 1:
        currenttime = 0
        while ids[0] != id_to_find and durationtime >= currenttime:
            fly_go(vehicle,0.5,0,0,1)
            currenttime+=1
        if ids[0] != id_to_find:
            search_direction = 2
    elif search_direction == 2:
        currenttime = 0
        while ids[0] != id_to_find and durationtime >= currenttime:
            fly_go(vehicle,0,0.5,0,1)
            currenttime+=1
        search_direction = 2
        durationtime += 1
        search_direction = 3
    elif search_direction == 3:
        fly_go(vehicle,-0.5,0,0,1)
        search_direction == 0
    else:
        fly_go(vehicle,0,-0.5,0,1)
        durationtime += 1
        search_direction = 1