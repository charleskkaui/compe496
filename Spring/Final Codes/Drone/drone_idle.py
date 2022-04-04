import sys
from bluedot.btcomm import BluetoothServer

def data_received(data):
    print(data)
    received = data
    s.send(data) #MOVE TO THE IF?

print("DRONE_IDLE_START")
received = "0"
s = BluetoothServer(data_received)

while True:
    if(received == "U"):
        print("IDLE2TAKEOFF")
        import drone_takeoff
        print("TERMINATING_IDLE")
        sys.exit(0)
    else:
        print("DRONE_IDLE")
        
