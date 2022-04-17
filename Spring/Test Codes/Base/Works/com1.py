import sys
from bluedot.btcomm import BluetoothServer
import time

###
#Start com 1 on one device and then start com4 on another
###
print("Enter Com1")
print("Start Listen")


#listening = False
global received

def data_received(data):
    #print(data)
    received=data
    print("this is received: " + received)
    s.send("3")
    time.sleep(1)
        

s = BluetoothServer(data_received)
    
while True:
    global received
    data_received
    time.sleep(4)
    if "ready to land" in received:
        s.send("Landing, waiting for landing signal.")
        print("I sent: 'Landing, waiting for landing signal.'")
        import landing
        continue
    elif "landed" in received:
        import takeoff
        s.send("retracting pins")
        print("I sent: 'retracting pins'")
        continue
    else:
        pass
    print("listening")

#import com2
#print("Terminating Listening")
#sys.exit(0)

        
#