#!/usr/bin/env python
import rospy
import tf
import numpy as np
import tf.transformations as tfm
from apriltags_ros.msg import AprilTagDetectionArray

rospy.init_node('calibration', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
camera_pose3d = 0
count = 0
def main():
	file = open('/home/nctuece/catkin_ws/src/heightmap_creator/camera.txt', 'r+')
	data = file.readlines()
	file.close()
	camera_pose = [float(x) for x in data[0].split(' ') if x.strip()]
	while(1):
		br.sendTransform(camera_pose[0:3], camera_pose[3:7], rospy.Time.now(), '/camera_link', '/bin_center')
		rospy.sleep(0.01)
	rospy.on_shutdown(myhook)

def myhook():
    print "shutdown time!"			

if __name__=='__main__':
    main()