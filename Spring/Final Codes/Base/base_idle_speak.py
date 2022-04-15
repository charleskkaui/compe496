from bluedot.btcomm import BluetoothClient
from signal import pause
import time
import sys

def data_received(data):
    print(data)
    c.send("TX Received")

c = BluetoothClient("raspberry", data_received)

while True:
    time.sleep(10)
    c.send("A")
    time.sleep(10)
    c.send("B")