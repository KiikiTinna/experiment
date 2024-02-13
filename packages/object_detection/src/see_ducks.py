
#!/usr/bin/env python3

import cv2

import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType, ParamType
from duckietown_msgs.msg import BoolStamped, VehicleCorners
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage


class VehicleDetectionNode(DTROS):

    def __init__(self, node_name):
        super(VehicleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Initialize the parameters
        self.process_frequency = DTParam("~process_frequency", param_type=ParamType.FLOAT)
        self.circlepattern_dims = DTParam("~circlepattern_dims", param_type=ParamType.LIST)
        self.blobdetector_min_area = DTParam("~blobdetector_min_area", param_type=ParamType.FLOAT)
        self.blobdetector_min_dist_between_blobs = DTParam(
            "~blobdetector_min_dist_between_blobs", param_type=ParamType.FLOAT
        )
        self.veh = rospy.get_namespace().strip("/")
        
        self.cbParametersChanged()

        self.bridge = CvBridge()

        self.last_stamp = rospy.Time.now()

        # Subscriber
        self.sub_image_topic = '/'+self.veh_name+'/camera_node/image/compressed'
        self.sub_image = rospy.Subscriber(self.sub_image_topic, CompressedImage, self.image_cb, queue_size=1) 


        # Publishers

        self.pub_image_topic='/'+self.veh_name+'debug/detection_image/compressed'
        self.pub_circlepattern_image = rospy.Publisher(self.pub_image_topic, CompressedImage, queue_size=1)

        self.pub_centers = rospy.Publisher("~centers", VehicleCorners, queue_size=1)
        self.pub_detection_flag = rospy.Publisher("~detection", BoolStamped, queue_size=1)

        self.log("Initialization completed.")



    def cbParametersChanged(self):
        self.publish_duration = rospy.Duration.from_sec(1.0 / self.process_frequency.value)
        params = cv2.SimpleBlobDetector_Params()
        params.minArea = self.blobdetector_min_area.value
        params.minDistBetweenBlobs = self.blobdetector_min_dist_between_blobs.value
        self.simple_blob_detector = cv2.SimpleBlobDetector_create(params)


    def image_cb(self,image_msg):

        now = rospy.Time.now()
        if now - self.last_stamp < self.publish_duration:
            return
        else:
            self.last_stamp = now

        vehicle_centers_msg_out = VehicleCorners()
        detection_flag_msg_out = BoolStamped()
        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")

        detection, centers = cv2.findCirclesGrid(image_cv,patternSize=tuple(self.circlepattern_dims.value),flags=cv2.CALIB_CB_SYMMETRIC_GRID)

        # if the pattern is detected, cv2.findCirclesGrid returns a non-zero result, otherwise it returns 0
        # vehicle_detected_msg_out.data = detection > 0
        # self.pub_detection.publish(vehicle_detected_msg_out)

        vehicle_centers_msg_out.header = image_msg.header
        vehicle_centers_msg_out.detection.data = detection > 0
        detection_flag_msg_out.header = image_msg.header
        detection_flag_msg_out.data = detection > 0
        
     
        # if the detection is successful add the information about it,
        # otherwise publish a message saying that it was unsuccessful

        if detection > 0:
            points_list = []
            for point in centers:
                center = Point32()
                center.x = point[0, 0]
                center.y = point[0, 1]
                center.z = 0
                points_list.append(center)
            vehicle_centers_msg_out.corners = points_list
            vehicle_centers_msg_out.H = self.circlepattern_dims.value[1]
            vehicle_centers_msg_out.W = self.circlepattern_dims.value[0]
        
        self.pub_centers.publish(vehicle_centers_msg_out) 
        self.pub_detection_flag.publish(detection_flag_msg_out)  

        if self.pub_circlepattern_image.get_num_connections() > 0:
            cv2.drawChessboardCorners(image_cv, tuple(self.circlepattern_dims.value), centers, detection)
            image_msg_out = self.bridge.cv2_to_compressed_imgmsg(image_cv)
            self.pub_circlepattern_image.publish(image_msg_out)  



 if __name__ == "__main__":
    vehicle_detection_node = VehicleDetectionNode("vehicle_detection")
    rospy.spin()