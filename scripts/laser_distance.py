#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, String
from ucsd_robo_car_simple_ros.msg import distance
from sensor_msgs.msg import LaserScan

distance_topic_name = 'distance'
laserscan_topic_name = 'scan'

laser_detect = distance() #setting variable laser_detect as custom message type distance

def callback(msg):
    #seperating the laserscan to different sections
    front_rightside_dist = min(msg.ranges[283:423]) #right-side of the front section
    front_leftside_dist = min(msg.ranges[424:564]) #left-side of the front section
    frontright_dist = min(msg.ranges[1:282]) #frontright of the camera
    frontleft_dist = min(msg.ranges[565:847]) #frontleft of the camera

    #displaying the measured values
    #rospy.loginfo('[Front]:'+str(front_dist))
    #rospy.loginfo('[FrontRight]:'+str(frontright_dist))
    #rospy.loginfo('[FrontLeft]:'+str(frontleft_dist))

    #classifying the data in a custom message type distance()
    #.range refers to the actual distance to the obstacle
    #.direction refers to which direction is the obstacle located
    if  front_leftside_dist <=2 or front_rightside_dist <= 2: #if detected in front section, decides if it's 
                                                              #on the right or left side of the front section
        if front_leftside_dist >= front_rightside_dist: 
            laser_detect.direction = 'front_rightside'
            laser_detect.range = front_rightside_dist 
        else: 
            laser_detect.direction = 'front_leftside'
            laser_detect.range = front_leftside_dist

    if frontright_dist <= 2:
        laser_detect.direction = 'right'
        laser_detect.range = frontright_dist
    
    if frontleft_dist <= 2:
        laser_detect.direction = 'left'
        laser_detect.range = frontleft_dist

    
def main():
    rospy.init_node('laser_distance_node')
    sub = rospy.Subscriber(laserscan_topic_name, LaserScan, callback) #subscribed to the laserscan topic
    pub = rospy.Publisher(distance_topic_name, distance, queue_size=1) #publishing to the distance topic, passing to obstacleavoidance.py
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        pub.publish(laser_detect)
        rate.sleep()

if __name__ == '__main__':
    main()
