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


# --- caffe dep ---#
import sys
sys.path.append('/home/peter/caffe/python') # pycaffe 
sys.path.append('/home/peter/caffe/fcn.berkeleyvision.org') # bvlc fcn module path
from PIL import Image
import caffe
import os
# --- caffe dep ---#





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




# load net
net = caffe.Net('voc-fcn8s/deploy.prototxt', 'voc-fcn8s/fcn8s-heavy-pascal.caffemodel', caffe.TEST)


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
	
	# load image, switch to BGR, subtract mean, and make dims C x H x W for Caffe
	im = cv_image
	in_ = np.array(im, dtype=np.float32)
	in_ = in_[:,:,::-1]
	in_ -= np.array((104.00698793,116.66876762,122.67891434))
	in_ = in_.transpose((2,0,1))
	# shape for input (data blob is N x C x H x W), set data
	net.blobs['data'].reshape(1, *in_.shape)
	net.blobs['data'].data[...] = in_
	# run net and take argmax for prediction
	net.forward()
	out = net.blobs['score'].data[0].argmax(axis=0)

	image = Image.fromarray(net.blobs[layer].data[0].argmax(0).astype(np.uint8), mode='P')
	mask = mask_convertion(image)
        cv_mask = cv2_to_imgmsg(mask, encoding="passthrough")   #grays scale

    except CvBridgeError as e:
        print(e)
    '''



    '''

    point_publisher.publish(depth_data)
    mask_publisher.publish(cv_mask)


def mask_convertion(unconverted_result):
    color, count = unconverted_result.colors()
    decreasing_count = count[np.argsort(-count)]
    decreasing_color = color[np.argsort(-count)]

    # index 0 must be background, use index 1
    mask = unconverted_result
    for height in range(0,481):
        for width in range(0,641):
		cls = rgb.getpixel((width,height))
		if cls == decreasing_color[1]:
			mask.putpixel((width,height),255)
	`	else:
			mask.putpixel((width,height),0)
   return mask
    
    



if __name__=='__main__':
    main()
    
