# Example to send a packet periodically between addressed nodes
# Author: Jerry Needell
# modified by Scott Kirkpatrick
#this now receives a series of floats as binary in the rf packet and converts for tracking
#this app now drives the display and reports gps position and altitude on the small display
#this app includes button press capability
#this app includes sending a record command to the sensor rpi
import time
import struct
import board
import busio
import digitalio
from digitalio import DigitalInOut, Direction, Pull
from time import sleep
import adafruit_rfm9x
import adafruit_ssd1306
import csv

#setup csv file with headers for logging data, and adding headers
with open('logremote.csv','w', newline='') as csvfile:
    fieldnames=['latitude','longitude','pressure','temperature',
    'xaccel','yaccel','zaccel','gpsalt','gpsheight','gpsspeed','gpstrack','xmag','ymag','zmag']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

# Button A
btnA = DigitalInOut(board.D5)
btnA.direction = Direction.INPUT
btnA.pull = Pull.UP

# Button B
btnB = DigitalInOut(board.D6)
btnB.direction = Direction.INPUT
btnB.pull = Pull.UP

# Button C
btnC = DigitalInOut(board.D12)
btnC.direction = Direction.INPUT
btnC.pull = Pull.UP

# Create the I2C interface.
i2c = busio.I2C(board.SCL, board.SDA)

# 128x32 OLED Display
reset_pin = DigitalInOut(board.D4)
display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)
# Clear the display.
display.fill(0)
display.show()
width = display.width
height = display.height

# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip.
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# enable CRC checking
rfm9x.enable_crc = True
# set node addresses
rfm9x.node = 2
rfm9x.destination = 1
rfm9x.tx_power=23
# initialize counter
counter = 0
latnum=0
lonnum=0 
press=0 
temp=0
recording=False 
last_record=False
start_record=False
# send a broadcast message from my_node with ID = counter
rfm9x.send(bytes("startup message from node {} ".format(rfm9x.node), "UTF-8"))

# Wait to receive packets.
print("Waiting for packets...")
#sending info to minidisplay
display.text("Waiting for packets...",  width-110, height-20, 1)
display.show()
# initialize flag and timer
time_now = time.monotonic()
while True:
    #check buttonpress
    #button check to start recording
    if not btnA.value:
        # Button A Pressed toggle recording request to other pi
        start_record= not start_record
         #send request
        rfm9x.send(struct.pack('?',start_record))
        sleep(0.1)
    # Look for a new packet: only accept if addresses to my_node
    packet = rfm9x.receive(with_header=True)
    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        # lets not print("Received (raw header):", [hex(x) for x in packet[0:124]])
        #convert rawpackets to floats from the other device
        latnum=struct.unpack('d',packet[4:12])
        lonnum=struct.unpack('d',packet[12:20])
        press=struct.unpack('d',packet[20:28])
        temp=struct.unpack('d',packet[28:36])
        xaccel=struct.unpack('d',packet[36:44])
        yaccel=struct.unpack('d',packet[44:52])
        zaccel=struct.unpack('d',packet[52:60])
        gpsalt=struct.unpack('d',packet[60:68])
        gpsheight=struct.unpack('d',packet[68:76])
        gpsspeed=struct.unpack('d',packet[76:84])
        gpstrack=struct.unpack('d',packet[84:92])
        xmag=struct.unpack('d',packet[92:100])
        ymag=struct.unpack('d',packet[100:108])
        zmag=struct.unpack('d',packet[108:116])
        palt=struct.unpack('d',packet[116:124])
        recordin=struct.unpack('?',packet[124:125])
        recording=recordin[0] #apparently the packet is pulled in as an f-ing tuple this untuples it
        #convert floats to strings
        lat=str('%.7g' % latnum) 
        lon=str('%.7g' % lonnum) 
        hgt=str('%.7g' % gpsalt)
        #output to telnet
        print("located other pi at following coordinates")
        print("latitude "+lat+" longitude "+lon)
        print("altitude " +hgt)
        # debugging (this is how I found it was an f-ing tuple) print(type(recordin))
        #print(type(recording))
        #print(recordin)
        #print(recording)
        #output to lcd screen (first clear screen)
        display.fill(0)
        display.show()
        #writing altitude longitude and latitude to small screen
        display.text('altitude '+hgt, width-125, height-8, 1)
        display.text("latitude "+lat,  width-125, height-20, 1)
        display.text("longitude "+lon,  width-125, height-30, 1)
        #recording notification if video is recording
        if recording is True:
            print("other pi has started recording video")
            display.text("rec", width-20, height-8,1)
            pass
        display.show()
        #now saving a log of the data on the remote pi
        with open('logremote.csv','a', newline='') as csvfile:
            fieldnames=['latitude','longitude','pressure','temperature',
            'xaccel','yaccel','zaccel','gpsalt','gpsheight','gpsspeed','gpstrack','xmag','ymag','zmag']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow({'latitude': latnum,'longitude': lonnum,'pressure': press,'temperature': temp,
            'xaccel': xaccel,'yaccel': yaccel,'zaccel': zaccel,'gpsalt': gpsalt,'gpsheight': gpsheight,'gpsspeed': gpsspeed,
            'gpstrack': gpstrack,'xmag': xmag,'ymag': ymag,'zmag': zmag})
        #print("altitude "+ gpsalt)   
        #rfm9x.send(struct.pack('d',latnum)+struct.pack('d',lonnum))      
       # print("Received (raw payload): {0}".format(packet[4:]))
        print("Received RSSI: {0}".format(rfm9x.last_rssi))
        # send reading after any packet received
        counter = counter + 1
        #removing generic send response
        # after 10 messages send a response to destination_node from my_node with ID = counter&0xff
       # if counter % 10 == 0:
        #    time.sleep(0.5)  # brief delay before responding
         #   rfm9x.identifier = counter & 0xFF
         #   rfm9x.send(
          #      bytes(
           #         "message number {} from node {} ".format(counter, rfm9x.node),
            #        "UTF-8",
             #   ),
         #       keep_listening=True,
          #  )