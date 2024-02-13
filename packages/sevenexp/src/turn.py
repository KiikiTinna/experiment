#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped



class SpeedPublisherNode(DTROS):

    def __init__(self, node_name):
        super(SpeedPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # construct publisher
        self.pub = rospy.Publisher('joy_mapper_node/car_cmd', Twist2DStamped, queue_size=10)
        self.msg = Twist2DStamped()
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Use CTRL + C to stop node")



    def send_speed(self):
        while not rospy.is_shutdown():
            faster = input('Input l for left, r for right ')
            time = 0
            rospy.sleep(3.0)  
            if faster == 'l':
                self.msg.v = 0.0
                self.msg.omega = 7 
                while time <= 300:
                    self.pub.publish(self.msg)
                    time = time + 1 
                    rospy.loginfo(time)
                    if time > 300:
                        self.msg.v = 0.0
                        self.pub.publish(self.msg)               

            elif faster == 'r':
                self.msg.v = 0.0
                self.msg.omega = -7
                while time <= 400:
                    self.pub.publish(self.msg)  
                    time = time + 1
                    rospy.loginfo(time)
                    if time > 400:
                        self.msg.v = 0.0
                        self.pub.publish(self.msg)

            else:
                self.msg.v = 0.0
                self.msg.omega = 0.0 
                self.pub.publish(self.msg) 


    def shutdown(self):
        rospy.loginfo("Shutdown initiated, stopping motors")
        self.msg = Twist2DStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.v = 0.0
        self.msg.omega = 0.0
        # Publish motion stop message

        self.pub.publish(self.msg)
        #rospy.sleep(1)

    
if __name__ == '__main__':
    # create the node
    velocitynode = SpeedPublisherNode(node_name='speed_pub_node')  
    try:
        while not rospy.is_shutdown():
            velocitynode.send_speed()
            velocitynode.rate.sleep()
    except rospy.ROSInterruptException:
        raise Exception(
            "Error encountered when attempting to start")
