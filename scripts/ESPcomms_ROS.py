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




# steering topic callback
def callbackSteering(steerData):
    global normalized_steering
    normalized_steering = round(steerData.data,3)
    # print("normalized_steering", normalized_steering, "steerdata",steerData.data)
    # sendJSON()
    
# throttle topic callback
def callbackThrottle(throttleData):
    global normalized_throttle
    normalized_throttle = round(throttleData.data,3)
    # sendJSON()

# creates and returns a JSON filled with throttle, steering, and heartbeat data
def fillJSON():
    # print("normalized_throttle", normalized_throttle, "normalized_steering", normalized_steering)
    data = {}
    data["throttle"] = normalized_throttle
    data["steering"] = normalized_steering
    # data["heartbeat"] = round(time.time(),0)
    data2=json.dumps(data)
    # print("data2",data2)
    return data2 

    # return json.dumps({'throttle': normalized_throttle, 'steering': normalized_steering,'heartbeat':round(time.time(),0)})



# triggered periodically, or with new throttle/steering, sends JSON overserial
def sendJSON():
    data = fillJSON()

    
    if ser.isOpen():
            print("sending JSON!")
            data = data + " \n"
            ser.write(data.encode("ascii"))
            
            # data = "testMsg\n"
            print("encodedJSON = ", data.encode("ascii"))
            # print(fillJSON().encode('ascii'))
            # print(fillJSON().encode('utf-8'))
            # ser.write(fillJSON().encode('utf-8'))
            # print("writtn", ser.write(data.encode('ascii')))
            print('done sending')
            # print("debug sent ", ser.read(8))
            # print('done reqding')
            # print(ser.readline().decode("ascii"))
            ser.flush()
            try:
                print("entered the try incoming")
                
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
    rospy.init_node(NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callbackSteering)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callbackThrottle)
    
      
    # send the heartbeat at 20Hz
    rate = rospy.Rate(10) # 20Hz
    print("entering while loop")
    
    while not rospy.is_shutdown():
        rate.sleep()
        sendJSON()

    ser.close()
    print("killing serial")

if __name__ == '__main__':
    main()
    

    
