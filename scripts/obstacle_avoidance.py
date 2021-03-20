#! /usr/bin/env python

import numpy as np
import rospy
import math
from std_msgs.msg import Float32, String
from ucsd_robo_car_simple_ros.msg import distance

distance_topic_name = 'distance'
laneguidance_steering_topic_name = 'steering'
laneguidance_throttle_topic_name = 'throttle'
steering_topic_name = 'steering_adjusted'
throttle_topic_name = 'throttle_adjusted'

#for testing
#to manually publish: rostopic pub -r 30 /distance ucsd_robo_car_simple_ros/distance '{range: 1.0, direction: "front_leftside"}'


global steering_float, throttle_float, steering_output, throttle_output
steering_float = Float32()
throttle_float = Float32()
steering_output = Float32()
throttle_output = Float32()


def steering_input(data):
    global steering_float
    steering_float = data.data #initial steering from lane_guidance


def throttle_input(data):
    global throttle_float
    throttle_float = data.data #initial throttle from lane_guidance


def guidance(data):
    global steering_float, throttle_float, steering_output, throttle_output

    weight_throttle = 1.0 #initial weight for throttle - if none of the conditions play out, 
                          #will not change throttle value from lane guidance
    weight_steer = 0.0 #initial weight for steering - if none of the conditions play out, 
                       #will not change steering value from lane guidance
    steer_direction = 0.0 #initial value

    steer_right = 1.0 #sets direction
 
    steer_left = -1.0 #sets direction

    np.nan_to_num(steering_float) #if the float variables are empty, this sets them to 0

    np.nan_to_num(throttle_float)

    if data.direction == 'front_rightside':
        steer_direction = steer_left
        weight_steer = 0.25 #scaled steering weight
        if data.range > 0.05: #if object is within range
            weight_throttle = (1-math.exp(-(data.range - 0.05))) #weight for how much throttle changes
        if math.isnan(data.range): #no throttle change for undetermined range
            weight_throttle = 1.0
        if data.range <= 0.05:  #for less than specified distance, throttle goes to 0
            weight_throttle = 0.0

    if data.direction == 'front_leftside':
        steer_direction = steer_right
        weight_steer = 0.25
        if data.range > 0.05:
            weight_throttle = (1-math.exp(-(data.range - 0.05))) 
        if math.isnan(data.range):
            weight_throttle = 1.0
        if data.range <= 0.05: 
            weight_throttle = 0.0  

    if data.direction == 'right':
        weight_steer = 0.0
        weight_throttle = 1.0

    if data.direction == 'left':
        weight_steer = 0.0        
        weight_throttle = 1.0
    
    throttle_output = weight_throttle * throttle_float
    steering_output = (weight_steer * steer_direction) + steering_float


def main():
    rospy.init_node('obstacle_avoidance_node')
    distance_sub = rospy.Subscriber(distance_topic_name, distance, guidance) #subscribing from laserdistance range and direction
    steering_sub = rospy.Subscriber(laneguidance_steering_topic_name, Float32, steering_input) #subscribing from laneguidance steering
    throttle_sub = rospy.Subscriber(laneguidance_throttle_topic_name, Float32, throttle_input) #subscribing from laneguidance throttle
    steering_pub = rospy.Publisher(steering_topic_name, Float32, queue_size=1) #publishing to steering_client
    throttle_pub = rospy.Publisher(throttle_topic_name, Float32, queue_size=1) #publishing to throttle_client
    rate = rospy.Rate(30) #rate in which throttle and steering is published
    while not rospy.is_shutdown(): # publish

        steering_pub.publish(steering_output)
        throttle_pub.publish(throttle_output)
        rate.sleep()


if __name__ == '__main__':
    main()
