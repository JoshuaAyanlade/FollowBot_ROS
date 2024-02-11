#!/usr/bin/env python3

import rospy
import cv2,cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.img_sub = rospy.Subscriber("/robot_a/front_rgbd_camera/rgb/image_raw",Image,self.img_callback)
        self.cmd_vel_pub = rospy.Publisher("/robot_a/cmd_vel",Twist,queue_size=1)
        self.twist = Twist()
        cv2.namedWindow("Frame",1)

    def img_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #bgr_img = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20,20,50])
        upper_yellow = np.array([255,255,255])
        mask_img = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
        #masked = cv2.bitwise_and(image,image,mask=mask_img)

        h,w,d=image.shape
        h_top = int(3*h/4)
        h_bot = int(h_top + 20)
        mask_img[0:h_top,0:w] = 0
        mask_img[h_bot:h,0:w] = 0

        #finds the centroid coordinate of the image
        M = cv2.moments(mask_img)
        
        if M['m00']>0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image,(cx,cy),20,(0,0,255),-1)

            dev = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(dev) / 100
            self.cmd_vel_pub.publish(self.twist)
        #cv2.imshow("Frame",mask_img)
        cv2.imshow("Frame",image)
        cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node("follower")
    follower = Follower()
    rospy.spin()