from bluedot.btcomm import BluetoothClient
from signal import pause
import time

SLEEP_TIME = 30 #time is in seconds

def data_received(data):
    print(data)

c = BluetoothClient("raspberrypi", data_received)
time.sleep(SLEEP_TIME)
c.send("U")
print("TERMINATING_IDLE")
sys.exit(0)