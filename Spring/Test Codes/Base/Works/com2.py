from bluedot.btcomm import BluetoothClient
from signal import pause
import time
import sys

sleep_time = 10
def data_received(data):
    print(data)
    
print("Enter Com2")
c = BluetoothClient("raspberrypiServer", data_received)
time.sleep(sleep_time)
c.send("3")
import com1
print("terminating idle")
sys.exit(0)