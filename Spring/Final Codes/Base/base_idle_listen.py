import sys
from bluedot.btcomm import BluetoothServer #speaking = client server = listen

def data_received(data):
    print(data)
    received = data
    s.send(data) #MOVE TO THE IF?

print("BASE_IDLE__LISTEN_START")
received = "0"
s = BluetoothServer(data_received)

while True:
    if(received == "T"):
        print("IDLE2LATCH")
        import base_latch
        print("TERMINATING_IDLE")
        sys.exit(0)
    else:
        print("BASE_IDLE_LISTENING")