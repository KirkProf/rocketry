# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
#
# Testing how many add-ons the raspi zero can handle
#started from a gps python from ladayada
# Will wait for a fix and print a message every second with the current location
# and other details.
#added camera capabilities
#added saving data to a file
#added remote pi control of camera video start and stop
#first the basic imports
import time
import board
import busio
import struct
import adafruit_bmp3xx
import digitalio
from time import sleep
#imports for specific add-ons
from picamera import PiCamera
import adafruit_gps
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import adafruit_rfm9x
import csv

#setup csv file with headers for logging data, and adding headers
with open('log.csv','w', newline='') as csvfile:
    fieldnames=['latitude','longitude','pressure','altitude','temperature',
    'xaccel','yaccel','zaccel','gpsalt','gpsheight','gpsspeed','gpstrack','xmag','ymag','zmag']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

# SPI setup for pressure
from digitalio import DigitalInOut, Direction
spi = busio.SPI(board.D21, board.D20, board.D19)
cs = DigitalInOut(board.D17)
bmp = adafruit_bmp3xx.BMP3XX_SPI(spi, cs)
bmp.pressure_oversampling = 8
bmp.temperature_oversampling = 2


#setup for accelerometer and magnetic field
i2c = busio.I2C(board.SCL, board.SDA)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

#setup for radio check for overlap with SPI pressure sensor
transmit_interval = 10
# Define radio parameters.
RADIO_FREQ_MHZ = 915.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.
# Define pins connected to the chip.
CSFM = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
# Initialize SPI bus. Modified name to not clash with pressure
spiFM = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spiFM, CSFM, RESET, RADIO_FREQ_MHZ)
# enable CRC checking
rfm9x.enable_crc = True
# set node addresses
rfm9x.node = 1
rfm9x.destination = 2
rfm9x.tx_power=23
# initialize counter
counter = 0

#adding camera commands to gps program creating instance
camera = PiCamera()

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)
# for a computer, use the pyserial library for uart access
import serial
uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
# If using I2C, we'll create an I2C interface to talk to using default pins
# i2c = board.I2C()

#creating/declaring variables for pressure temperature and accel
press= float(0)
temp=float(0)
xaccel=float(0)
yaccel=float(0)
zaccel=float(0)
xmag=float(0)
ymag=float(0)
zmag=float(0)
palt=float(0)
last_camstart=0.0
recordin=False
last_recording=False
recnumber=1
gpsalt=float(0)
gpsspeed=float(0)
gpsheight=float(0)
gpstrack=float(0)

#creating variables for gps latitude and longitude
latnum=float(0)
lonnum=float(0)
lat=''
lon=''
#setting up binary array for sending data
#First declaring the type of data being sent double floats and boolean
s = struct.Struct('d d d d d d d d d d d d d d d ?')
values=(latnum,lonnum,press,temp,xaccel,yaccel,zaccel,gpsalt,gpsheight,gpsspeed,gpstrack,xmag,ymag,zmag,palt,recordin)
packed_data = s.pack(*values)
status=False #this is the video recording status
# Create a GPS module instance.
gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
# gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
# Turn on just minimum info (RMC only, location):
# gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
# gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on everything (not all of it is parsed!)
# gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
#or.. take a zero away and go at 10hz..?
gps.send_command(b"PMTK220,110")
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')

#setting camera text size
camera.annotate_text_size = 50  #apparently you can write text over the video, this makes it a bit big but easily viewable
#status=camera._check_recording_stopped()
#print(status)

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()
while True:
    # Make sure to call gps.update() every loop iteration and at least twice
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    gps.update()
    #start camera view
    #camera.start_preview()
   
    #checking for data getting sent and sending to a packet
    packet = rfm9x.receive(with_header=True)
    current = time.monotonic()
    if packet is not None:
        # Received a packet!
        askrecordin=struct.unpack('?',packet[4:5])
        recordin=askrecordin[0] #packet boolean is placed in a f-ing tuple soo you have to save the "list" to a boolean variable..grr.
        if (recordin==True): #is it start or stop record
             
            if (current-last_camstart>=0.5) and (status is False):#trying to avoid instant switchoff and double starts
                #start recording
                status=True #tag that gives state of recording
                last_camstart=time.monotonic() #tag the start time of the camera
                camera.start_recording('launch%d.h264' %recnumber) #incrementing the recording
                recnumber+=1
        else:
            #stop recording
            if status is True: #if statement for popping in with an undefined value
                camera.stop_recording()
                recordin=False
                status=False
    
    if (current-last_camstart >= 30.0):
        if (recordin) and (status):  #keep videos to 30 seconds checking for recording
            camera.stop_recording()
            recordin=False
            status=False
     # Every second print out current location details if there's a fix.
    #modifying the loop time to 0.1 from 1.0 second
    if current - last_print >= 0.10:
        last_print = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.
        print("=" * 40)  # Print a separator line.
        print(
            "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                gps.timestamp_utc.tm_mday,  # struct_time object that holds
                gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                gps.timestamp_utc.tm_min,  # month!
                gps.timestamp_utc.tm_sec,
            )
        )
        #putting data into variables
        latnum=gps.latitude
        lonnum=gps.longitude
        if gps.altitude_m is not None:
            gpsalt=gps.altitude_m
        if gps.height_geoid is not None:
            gpsheight=gps.height_geoid
        if gps.speed_knots is not None:
            gpsspeed=gps.speed_knots
        if gps.track_angle_deg is not None:
            gpstrack=gps.track_angle_deg
        press=bmp.pressure
        temp=bmp.temperature
        palt=bmp.altitude
        xaccel,yaccel,zaccel=accel.acceleration
        xmag,ymag,zmag=mag.magnetic
        lat=str('%.5g' % gps.latitude )
        lon=str('%.5g' % gps.longitude)
        spalt=str('%.5g' % palt)
        #printing collected data to comm screen
        print("Latitude: {} degrees".format(lat))
        print("Longitude: {} degrees".format(lon))
        print("Pressure: {:6.4f}  Temperature: {:5.2f}".format(bmp.pressure, bmp.temperature))
        print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
        print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
        #print(palt)
        #print(recordin)
        #sending data over radio by packing floats into binary should use 8 bytes per float, 4 byte header on packet
        s = struct.Struct('d d d d d d d d d d d d d d d ?') #this is the structure of the data sent the ? means bool d is double float
        #print()
        values=(latnum,lonnum,press,temp,xaccel,yaccel,zaccel,gpsalt,gpsheight,gpsspeed,gpstrack,xmag,ymag,zmag,palt,recordin)
        #print(values )
        packed_data = s.pack(*values) #data packed into binary using the structure above
        rfm9x.send(packed_data) #sending the data to the radio
        #status=camera._check_recording_stopped() 
        print('camera:')
        print(status) #checking the functionality of the video recording software
        #camera gps attempt at printing--this works for images
        camera.annotate_text = "Lat:{}deg  Lon:{}deg  alt{}m".format(lat, lon, spalt)
        #output currently disabled print("Latitude: {0:.6f} degrees Longitude: {0:.6f} degrees?".format(lat, lon))
        #enable for camera capture camera.capture('/home/pi/image1.jpg')
        #writing to a log file, appending new data
        with open('log.csv','a', newline='') as csvfile:
            fieldnames=['latitude','longitude','pressure','altitude','temperature',
            'xaccel','yaccel','zaccel','gpsalt','gpsheight','gpsspeed','gpstrack','xmag','ymag','zmag']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow({'latitude': latnum,'longitude': lonnum,'pressure': press,'altitude':palt,'temperature': temp,
            'xaccel': xaccel,'yaccel': yaccel,'zaccel': zaccel,'gpsalt': gpsalt,'gpsheight': gpsheight,'gpsspeed': gpsspeed,
            'gpstrack': gpstrack,'xmag': xmag,'ymag': ymag,'zmag': zmag})

       # with open('logfile') as log:
        #    log.append(latnum)
        sleep(0.05)
        #camera.stop_preview()
        print("Fix quality: {}".format(gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        if gps.satellites is not None:
            print("# satellites: {}".format(gps.satellites))
        if gps.altitude_m is not None:
            print("Altitude: {} meters".format(gps.altitude_m))
        if gps.speed_knots is not None:
            print("Speed: {} knots".format(gps.speed_knots))
        if gps.track_angle_deg is not None:
            print("Track angle: {} degrees".format(gps.track_angle_deg))
        if gps.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(gps.horizontal_dilution))
        if gps.height_geoid is not None:
            print("Height geoid: {} meters".format(gps.height_geoid))
