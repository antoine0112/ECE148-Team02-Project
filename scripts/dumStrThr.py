#!/usr/bin/env python


import rospy
from std_msgs.msg import Float32
import time
import numpy as np


# ROS node and topic handles
NODE_NAME = 'dummySteeringThrottle'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'


rospy.init_node(NODE_NAME, anonymous=False)
throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=10)
steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=10)

# send the heartbeat at 20Hz
rate = rospy.Rate(20) # 20Hz
while not rospy.is_shutdown():
    throttle_msg = Float32()
    throttle_msg.data = np.sin(time.time()/20)
    
    steering_msg = Float32()
    steering_msg.data = np.cos(time.time()/20)
    
    throttle_pub.publish(throttle_msg)
    steering_pub.publish(steering_msg)
