#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math

rospy.init_node('sensor_unit_test', anonymous=True)



# Publisher for publishing images in rviz
point_publisher = rospy.Publisher('/camera/text_pose/points', PointCloud2, queue_size=10)
mask_publisher = rospy.Publisher('/camera/text_pose/mask', Image, queue_size=10)
# Bridge to convert ROS Image type to OpenCV Image type
cv_bridge = CvBridge()  

# Get the camera calibration parameter for the rectified image
msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, timeout=None) 
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]

def main():
    image_sub = message_filters.Subscriber('image', Image)
    info_sub = message_filters.Subscriber('camera_info', CameraInfo)
    image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
    depth_sub = message_filters.Subscriber("/camera/depth_registered/points", PointCloud2)         
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    ts.registerCallback(brand_prediction)
    rospy.spin()


def brand_prediction(rgb_data, depth_data):

    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
    except CvBridgeError as e:
        print(e)
    '''



    '''

    point_publisher.publish(depth_data)
    mask_publisher.publish(rgb_data)
    
    



if __name__=='__main__':
    main()
    
