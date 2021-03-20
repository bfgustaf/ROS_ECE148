#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
import atexit
import signal

THROTTLE_NODE_NAME = 'throttle_client'
THROTTLE_TOPIC_NAME = 'throttle_adjusted'

'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 0
'''

# for our car throttle, 0.14721435 is lowest value to turn wheels, set 90% weight to 0.17, 0% to 0.1472142
#this makes the throttle equation (linear) to y = 0.030873111*x + 0.1472142

#if you run the calibration script before starting the ESC, the ESC will calibrate as 0v to 0% throttle

kit = ServoKit(channels=16)

# scale down sensitive throttle
#throttle_scale = 0.1472142  #original value determined
throttle_scale = 0.0
throttle_slope = 0.0

#set the normalize throttle scale to a linear equation
def callback(data):
    rospy.loginfo(data.data)
    normalized_throttle = data.data
    kit.continuous_servo[2].throttle = (normalized_throttle * throttle_slope) + throttle_scale 
    #kit.continuous_servo[2].throttle = normalized_throttle


def listener():
    rospy.init_node(THROTTLE_NODE_NAME, anonymous=False)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callback)
    rospy.spin()

def throttle_exit():
    kit.continuous_servo[2].throttle = 0.0

atexit.register(throttle_exit)

signal.signal(signal.SIGTERM, throttle_exit)
signal.signal(signal.SIGINT, throttle_exit)

if __name__ == '__main__':
    listener()

