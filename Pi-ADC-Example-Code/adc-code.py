import time
import spidev
import os
import statistics

#Define Variables
delay = 0.1
row, col = (8,11)
lightVal = [[0]*col]*row
count = 0

#Create SPI
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 7629

def getADCData(channel):
    # read SPI data from the MCP3008, 8 channels in total
    #check to make sure we are using a valid channel
    if channel > 7 or channel < 0:
        return -1
    #pull raw data from the chip
    #transfer what we get out of the serial peripheral interface()
    #(send in 3 bytes get out 3 bytes)
    rawData  = spi.xfer([1, 8 + channel << 4, 0])
    #processs the raw data into something we understand
    #bit manipulation
    #get the last 2 positions of the 2nd byte and shift it by 8 then  append the 3rd byte to that
    processedData = ((rawData[1] & 3) << 8) + rawData[2]
    return processedData

def collectData():
   for i in range(row):
   #     for j in range(col):
   #         lightVal[i][j] = getADCData(i)
        #lightVal[i][0] = round(statistics.mean(lightVal[i][1:col]))
        lightVal[i] = getADCData(i)

def printAvg():
    #os.system('cls||clear')
    print("===========================")
    print ("%f %f %f" % (lightVal[0], lightVal[1], lightVal[2]))
    print ("%f %f %f" % (lightVal[3],  lightVal[4], lightVal[5]))
    print ("%f %f %f" % (lightVal[6],  lightVal[7], 0))


while True:
    collectData()
    printAvg()
    time.sleep(0.5)