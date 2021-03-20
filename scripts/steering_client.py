#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit
import atexit
import signal

STEERING_NODE_NAME = 'steering_client'
STEERING_TOPIC_NAME = 'steering_adjusted'

'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 1
'''
#Steering value is not -1 to 1, we will get better range with the corrected values; 
#fully left =  - 0.5 and fully right = 0.6; wheels straight is 0.05
slope = 0.75
offset = -0.05
kit = ServoKit(channels=16)

def callback(data):
    normalized_steering = data.data  # this is a value between -1 and 1, with -1 being fully left and 1 being fully right
    rospy.loginfo(data.data)
    offset_steering = (normalized_steering * slope) + offset
    angle_delta = offset_steering * 90  # difference in degrees from the center 90 degrees
    kit.servo[3].angle = 90 + angle_delta


def listener():
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rospy.spin()

def steering_exit():
    kit.servo[3].angle = 90

atexit.register(steering_exit)

signal.signal(signal.SIGTERM, steering_exit)
signal.signal(signal.SIGINT, steering_exit)

if __name__ == '__main__':
    listener()
