from bluedot.btcomm import BluetoothClient
from signal import pause
import time
# import serial
# from serial.serialutil import SerialException
import sys

# def initiate_ble_serial_port():
#      try:
#          port_ble_tablet = serial.Serial('/dev/rfcomm0')
#          print('Serial port for the device initialized' + str(port_ble_tablet))
#          return port_ble_tablet
#      except SerialException as e:
#          print(e)
#          return None
    
# port_ble_tablet = initiate_ble_serial_port()
received = "Base Received"

def data_received(data):
    global received
    received = data
    print(data)

c = BluetoothClient('raspberrypi-talon-base', data_received)

while True:
    message = input()
    c.send(message)
    if received == "Base Received":
        pass
    else:
        c.send(message)


    #if BluetoothClient.connected:
    #    print('I am connected')
    #elif not BluetoothClient.connected:
    #    c.connect('raspberrypi-talon-base')
#         BluetoothClient.connect('raspberrypi-talon-base')
# ble_msg = str(rcv_cnt/10) + '...'
# if port_ble_tablet:
#     port_ble_tablet.write(ble_msg.encode())
#     c.send("B")
# if data != '0' or != 'Base Received: B' or != 'Base Received: A':
#     import0
