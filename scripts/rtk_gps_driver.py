#!/usr/bin/env python3 
# -*- coding: utf-8 -*-

import rospy
import serial #to access serial port data
#import serial
from math import sin, pi
from std_msgs.msg import Float64
import time
import utm
from gps_driver.msg import gns_msg #import custom message

if __name__ == '__main__':
    SENSOR_NAME = "gps_lab2"
    rospy.init_node('ini_gps')
    serial_port = rospy.get_param('~port','/dev/ttyACM0')   #define the port name
    serial_baud = rospy.get_param('~baudrate',57600)        #define the baud rate
    sampling_rate = rospy.get_param('~sampling_rate',5.0)   #define the sampling rate
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    rospy.logdebug("Using gps sensor on port "+serial_port+" at "+str(serial_baud))
    rospy.logdebug("Initializing sensor with *0100P4\\r\\n ...")
    
    sampling_count = int(round(1/(sampling_rate*0.007913))) #calculating the sampling count
    i = 1                                                   #counter variable for sequence number
    rospy.sleep(0.2)        
    
    pub = rospy.Publisher('full_gps', gns_msg, queue_size=10)   #definging publisher with topic name and custom message
    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing longitude and latitude.")
    msg = gns_msg()                                             #declaring a message object
   
    try:
        while not rospy.is_shutdown():
            msg.header.seq=i                                    #updating the message header sequence number
            line = port.readline()                              #reading the serial port data
            if line == '':                                      #if there is no data, display "gngga: no data"
                rospy.logwarn("gngga: No data")
            else:
                if line.startswith(b'$GNGGA') :                 #parse the data from serial port only when it starts with $GNGGA
                    #print(line)
                    s =line.split(b",")                         #split the string with delimiter ","
                    lat = s[2].decode('utf-8')                  #getting required values of latitude, longitude,latitude direction,longitude direction, Gns fix quality etc. and decode them into UTF-8 format
                    lon = s[4].decode('utf-8')
                    lat_dir = s[3].decode('utf-8')
                    lon_dir = s[5].decode('utf-8')
                    gns_fix = int(s[6].decode('utf-8'))
                    utc_time = s[1].decode('utf-8')
                    alt = float(s[9].decode('utf-8'))
                                    
                    degrees_lat=int(float(lat)/100)
                    #print("\nDegree lat:"+str(degrees_lat))
                    minutes_lat=float(lat)-(degrees_lat*100)
                    #print("\tminutes lat:"+str(minutes_lat))
                    degrees_lon=int(float(lon)/100)
                    #print("\nDegree lon:"+str(degrees_lon))
                    minutes_lon=float(lon)-(degrees_lon*100)
                    #print("\tminutes lon:"+str(minutes_lon))
                    dd_lat= float(degrees_lat) + float(minutes_lat)/60  #final latitude and longitude conversion to decimal degrees
                    dd_lon= float(degrees_lon) + float(minutes_lon)/60 
                    if lon_dir == 'W':                                  #sign inverting decimal degree values of latitude and longitude based on direction
                        dd_lon *= -1
                    if lat_dir == 'S':
                        dd_lat *= -1
                    print("\n"+str(dd_lat)+" "+str(dd_lon))
    
                    utm_data3=utm.from_latlon(dd_lat,dd_lon)            #getting UTM Data using from_latlon function taking decimal degree values of longitude and latitude
                    print(utm_data3)
                    #(easting,northing,zone_float,zone_letter) = utm_data3.split(",")[0:4]
                    easting = utm_data3[0]                              #assigning utm data to variables to write to message
                    northing = utm_data3[1]
                    zone_float = utm_data3[2]
                    zone_letter = utm_data3[3]

                    #publish data to the message
                    msg.header.stamp=rospy.get_rostime()
                    msg.header.frame_id="GPS_Data"
                    msg.latitude = dd_lat
                    msg.longitude = dd_lon
                    msg.altitude =  alt
                    msg.gps_quality = gns_fix
                    msg.utm_easting = easting
                    msg.utm_northing = northing
                    msg.zone_number = zone_float
                    msg.zone_letter = zone_letter
                    pub.publish(msg)
                    i=i+1                                       #incrementing counter value for message header sequence
                
    except rospy.ROSInterruptException:
        port.close()
    
