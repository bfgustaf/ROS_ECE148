#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from decoder import decodeImage

global mid_x
mid_x = Int32()
global mid_y
mid_y = Int32()

pub = rospy.Publisher('/centroid', Int32, queue_size=1)


def video_detection(data):
    # decode image
    frame = decodeImage(data.data, data.height, data.width)

    #convert image from RGB to BGR (for Realsense only) #img = cv2.cvtColor(frame[240:290, 100:750], cv2.COLOR_RGB2BGR)

    # getting and setting image properties
    height, width, channels = frame.shape
    img = cv2.cvtColor(frame[250:440, 200:648], cv2.COLOR_BGR2RGB)
    orig = img.copy()
    #print(width)

    # get rid of white noise from grass
    kernel = np.ones((5, 5), np.float32)/15
    blurred = cv2.filter2D(img, -1, kernel)
    dilation = cv2.dilate(blurred, kernel, iterations=5)

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # setting threshold limits for yellow color filter
    lower = np.array([10, 10, 130])
    upper = np.array([30, 255, 255])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)
    
    #use contours to find the outline of the objects in the mask
    #ret,thresh = cv2.threshold(mask, 40, 255, 0) #threshold mask not needed-using color mask image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #find the largest contour by area, but if the 'max' comes back as NaN (no contours detected), 
    #then move to calculating the moment of the color mask
    try:
        img_cont = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(img_cont)
        img = cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2) #and plot a green box around the object

        #calculating centroid
        m = cv2.moments(img_cont, False) #this finds the centroid at the center of the largest mask

    except ValueError:
        m = cv2.moments(mask, False) #calculating the centroid of the color mask

    try:
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00']) #extract centroid coordinates
        
        #then save the current centroid coordinates, if available
        cy_prev = cy
        cx_prev = cx

    #if the new centroid cannot be found, the script will first attempt to use the previous centroid
    #coordinates to keep to the same course; 
    #otherwise it will set the centroid at the center of the frame
    except ZeroDivisionError:
            try:
                cy = cy_prev
                cx = cx_prev

            except UnboundLocalError:
                cy, cx = int(height / 2), int(width / 2)

    # Publish centroid
    mid_x.data = cx
    mid_y.data = cy
    cv2.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
    pub.publish(mid_x)

    # plotting results
    try:
        cv2.imshow("original", orig)
        cv2.moveWindow("original", 100,100)
        cv2.imshow("yellow mask", mask)
        cv2.moveWindow("yellow mask", 100,330);
        cv2.imshow("plotting centroid", img)
        cv2.moveWindow("plotting centroid", 100,560);
        cv2.waitKey(1)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()


def main():

    rospy.init_node('lane_detection_node', anonymous=True)
    camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, video_detection)
    rate = rospy.Rate(30)
    rospy.spin()


if __name__ == '__main__':
    main()
