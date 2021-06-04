#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import json
import serial
from serial import SerialException
from pprint import pprint
import random

# ROS node and topic handles
NODE_NAME = 'ESPcomms_ROS'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
serialPort = "/dev/ttyUSB0"
serialBaud = 115200
# serialBaud = 9600

# Serial settings
ser  = serial.Serial(serialPort, baudrate= serialBaud, 
        timeout=0.5, 
        parity=serial.PARITY_NONE, 
        bytesize=serial.EIGHTBITS, 
        stopbits=serial.STOPBITS_ONE
    )

# Global storage for steering and throttle values
normalized_steering = 0.0
normalized_throttle = 0.0

# triggered periodically, or with new throttle/steering, sends JSON overserial
def sendJSON():
    if ser.isOpen():
        try:
            dt = ser.readline()
            # print("debug", dt)
            incoming = dt.decode("ascii")
            print ("decodedMsg = ",incoming)    #what supposed to happen
        except Exception as e:
            print ('err', e)
            pass
            
    else:
        print ("opening error")



def main():
    # send the heartbeat at 20Hz
    rospy.init_node(NODE_NAME + "bis", anonymous=False)
    rate = rospy.Rate(10) # 20Hz
    print("entering while loop")
    
    while not rospy.is_shutdown():
        rate.sleep()
        sendJSON()

    ser.close()
    print("killing serial")

if __name__ == '__main__':
    main()