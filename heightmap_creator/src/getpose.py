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
	global camera_pose3d, count
	apriltag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
	

	while(count<20):
		rospy.sleep(1)
	#write to file
	#print camera_pose3d[0][0]
	file = open('/home/nctuece/catkin_ws/src/heightmap_creator/camera.txt', 'w+')
	file.write(str(camera_pose3d[0][0])+' '+str(camera_pose3d[0][1])+' '+str(camera_pose3d[0][2])+' '+str(camera_pose3d[1][0])+' '+str(camera_pose3d[1][1])+' '+str(camera_pose3d[1][2])+' '+str(camera_pose3d[1][3])+' \n')
	file.close()
	rospy.on_shutdown(myhook)

def apriltag_callback(data):
	global camera_pose3d, count
	if len(data.detections)!=0:
		detection = data.detections[0]
		if count == 0:
			print "found apriltag:", detection.id
		#print detection.pose
		if detection.id == 390:
			camera_pose3d = lr.lookupTransform('/tag_390', '/camera_link' ,rospy.Time(0))
			if count == 0:
				print camera_pose3d
			count = count + 1
			br.sendTransform(camera_pose3d[0], camera_pose3d[1], rospy.Time.now(), '/camera_link', '/bin_center')
			rospy.sleep(0.01)

def myhook():
    print "shutdown time!"			

if __name__=='__main__':
    main()