#!/usr/bin/env python3

import cv2
import numpy as np
import os
import yaml
import rospy

from cv_bridge import CvBridge, CvBridgeError

from duckietown.dtros import DTROS,NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped


class LineFollowerNode(DTROS):

    def __init__(self, node_name):
        super(LineFollowerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        self.bridge = CvBridge()

        self.sub_image_topic = '/'+self.veh_name+'/camera_node/image/compressed'
        self.sub_image = rospy.Subscriber(self.sub_image_topic, CompressedImage, self.camera_callback)
       
        # publisher for wheels command message (WheelsCmdStamped)
        self.pub_img_res_topic = '/'+self.veh_name+'/line_following_node/image'
        self.pub_img_res = rospy.Publisher(self.pub_img_res_topic, Image, queue_size=10)

    def camera_callback(self,img_msg):
        rospy.loginfo("I received an image message")
        try:
             cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV).astype(np.float)

        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments 
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        
        pub_img=self.bridge.cv2_to_imgmsg(res,"bgr8")
        pub_img.header = img_msg.header
        self.pub_img_res.publish(pub_img)
        
        error_x = cx - width / 2
        angular_z = -error_x / 100
        rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))


    def clean_up(self):
       cv2.destroyAllWindows()


def main():
   # rospy.init_node('line_following_node', anonymous=True)
    
    line_follower_object = LineFollowerNode(node_name='line_following_node')
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':

     main()
