#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
// PCL specific includes

#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/Marker.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/filter.h>

//Synchronize
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//opencv, cv bridge
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>

//tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// cmath map for transformation
#include <string>
#include <cmath>
#include <map>
#include <math.h>
//Publisher

ros::Publisher align_cloud_publisher;
ros::Publisher pose_publisher;
ros::Publisher marker_publisher;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
using namespace message_filters;
using namespace cv;
using namespace std;

tf::TransformBroadcaster* br;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr object1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object2 (new pcl::PointCloud<pcl::PointXYZRGB>);


//     [fx'  0  cx' Tx]
// P = [ 0  fy' cy' Ty]
//    [ 0   0   1   0]
double fx,fy,cx,cy ;
cv_bridge::CvImagePtr cv_ptr;
int mask_is_true=0;
/* 
void  pose_estimation (const sensor_msgs::PointCloud2ConstPtr& point, const sensor_msgs::ImageConstPtr& rgb_image)//
{	

	printf("Start\n");
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8); 
	PointCloudRGB::Ptr original_point(new PointCloudRGB);
	pcl::fromROSMsg(*point,*original_point);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (original_point);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.05);
	ne.compute (*cloud_normals);


}
*/

int  test1 (const sensor_msgs::PointCloud2ConstPtr& point)
{ 
	if ( mask_is_true==1){

		printf("Start_point\n");
		int count=0;
		int mask_pixel_num=0;
		double x_sum=0; double y_sum=0;
		PointCloudRGB::Ptr original_point(new PointCloudRGB);
		pcl::fromROSMsg(*point,*original_point);
		for (int row=0;row<480;row++){
			for(int column=0;column<640;column++){	
				if	(cv_ptr->image.at<uchar>(row,column)==0){
					original_point->points[count].x= std::numeric_limits<float>::quiet_NaN();
					original_point->points[count].y= std::numeric_limits<float>::quiet_NaN();
					original_point->points[count].z= std::numeric_limits<float>::quiet_NaN();
				}
				if	(cv_ptr->image.at<uchar>(row,column)!=0){
					original_point->points[count].r= 255;
					original_point->points[count].g= 0;
					original_point->points[count].b= 0;
					x_sum+=column;
					y_sum+=row;
					mask_pixel_num++;
				}
				count++;
			}
		}
		if (mask_pixel_num<20)
			return 0;
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (original_point);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree);
	    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		ne.setRadiusSearch (0.05);
		ne.compute (*cloud_normals);
		printf("Finish_point\n");
		
		
		x_sum /=mask_pixel_num; 
		y_sum /=mask_pixel_num;
		int x_index= (int)(x_sum); 
		int y_index= (int)(y_sum); 
		int x_momentum= (int)(x_sum); 
		int y_momentum= (int)(y_sum); 
		int min_distance=10000;
		count=0;
		for (int row=0;row<480;row++){
			for(int column=0;column<640;column++){	
				if	(cv_ptr->image.at<uchar>(row,column)!=0 && !isnan(original_point->points[count].z)){
					if( (row-y_momentum)*(row-y_momentum)+(column-x_momentum)*(column-x_momentum)<min_distance){
						x_index=column;
						y_index=row;
						min_distance = (row-y_momentum)*(row-y_momentum)+(column-x_momentum)*(column-x_momentum);
					}
				}
				count++;
			}
		}


		int point_index=y_index*640+x_index;
		printf("x,y,index: %d,%d,%d\n",x_index,y_index,point_index);

		
		

		double pitch_offset = -1.5707963;
		
		printf ("x,y,z,normalx,normaly,normalz: %lf %lf %lf %lf %lf %lf\n",original_point->points[point_index].x,original_point->points[point_index].y,original_point->points[point_index].z,cloud_normals->points[point_index].normal[0],cloud_normals->points[point_index].normal[1],cloud_normals->points[point_index].normal[2]);
		
		double roll, pitch, yaw;
		double x = cloud_normals->points[point_index].normal[0];
		double z = cloud_normals->points[point_index].normal[2];
		double z_x = (cloud_normals->points[point_index].normal[2])/(cloud_normals->points[point_index].normal[0]);
		if (z<0){
			if (x<0){
				pitch_offset*=-1;
			}
			else if (x>0){
				
			}
		}
		if(z>0){
			return 0;
		}

		roll = 0;
		pitch = pitch_offset + atan(z_x);
		yaw = 3.14151926;
		
		/*
		roll = 3.14151926;
		pitch = (pitch_offset + atan(z_x));
		yaw = 1.5707963;
		*/
		std::cout << roll << " " << pitch << " " << yaw << std::endl;	
		double quat[4];
		double t0 = cos(yaw*0.5);				
		double t1 = sin(yaw*0.5);
		double t2 = cos(roll*0.5);				
		double t3 = sin(roll*0.5);
		double t4 = cos(pitch*0.5);				
		double t5 = sin(pitch*0.5);
		
		quat[3] = t0*t2*t4+t1*t3*t5;
		quat[0] = t0*t3*t4-t1*t2*t5;
		quat[1] = t0*t2*t5+t1*t3*t4;
		quat[2] = t1*t2*t4-t0*t3*t5;

		std::cout << quat[0] << " " << quat[1] << " " <<quat[2] << " " << quat[3] << std::endl;
		

		visualization_msgs::Marker marker;
   		marker.header.frame_id = "camera_rgb_optical_frame";
   		marker.header.stamp = ros::Time();
   		marker.ns = "my_namespace";
   		marker.id = 0;
   		marker.type = visualization_msgs::Marker::SPHERE;
   		marker.action = visualization_msgs::Marker::ADD;
   		marker.pose.position.x = original_point->points[point_index].x+0.1*cloud_normals->points[point_index].normal[0];
   		marker.pose.position.y = original_point->points[point_index].y+0.1*cloud_normals->points[point_index].normal[1];
  		marker.pose.position.z = original_point->points[point_index].z+0.1*cloud_normals->points[point_index].normal[2];
  		marker.pose.orientation.x = 0;
  		marker.pose.orientation.y = 0;
  		marker.pose.orientation.z = 0;
  		marker.pose.orientation.w = 1;
  		marker.scale.x = 0.01;
  		marker.scale.y = 0.01;
  		marker.scale.z = 0.01;
  		marker.color.a = 1.0;
  		marker.color.r = 1.0;
  		marker.color.g = 0.0;
  		marker.color.b = 0.0;
		

		tf::Transform transform;
		transform.setOrigin( tf::Vector3(original_point->points[point_index].x-0.01, original_point->points[point_index].y, original_point->points[point_index].z) );
		transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
		br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "/dove_tag"));
		transform.setOrigin( tf::Vector3(original_point->points[point_index].x+0.15*cloud_normals->points[point_index].normal[0]-0.01, original_point->points[point_index].y+0.15*cloud_normals->points[point_index].normal[1]-0.02, original_point->points[point_index].z+0.15*cloud_normals->points[point_index].normal[2]) );
		transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
		br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_rgb_optical_frame", "/dove_tag_middle"));


		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*original_point, *original_point, indices);
		align_cloud_publisher.publish(original_point);
		marker_publisher.publish(marker);
		//pcl::io::savePCDFileASCII ("test_pcd.pcd",*original_point);
	}
	
	return 0;
}

void  test2 (const sensor_msgs::ImageConstPtr& rgb_image)//
{ 
	printf("Start_mask\n");
	cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::TYPE_8UC1); 
	mask_is_true=1;
	return;
}


int main(int argc, char** argv){

	printf("Ros Start\n");
	ros::init(argc, argv, "sensor_unit");
	ros::NodeHandle nh; 
	tf::TransformBroadcaster broadcaster;
	br = &broadcaster;
	/////////////////////////////Declare camera subscriber/////////////////////////////

    ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/camera/text_pose/points", 1, test1);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::Image>("/camera/text_pose/mask", 1, test2);



    /*
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_camera(nh,"/camera/text_pose/points", 10);
    message_filters::Subscriber<sensor_msgs::Image> object_mask(nh, "/camera/text_pose/mask", 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy1;
    Synchronizer<MySyncPolicy1> sync1(MySyncPolicy1(5), depth_camera, object_mask); //, camera_info1
    sync1.registerCallback(boost::bind(&pose_estimation,_1,_2));  //,_3
    */

    align_cloud_publisher = nh.advertise<PointCloudRGB> ("/camera/text_pose/object_cloud", 1);
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped> ("/camera/text_pose/pose", 1);
    marker_publisher = nh.advertise<visualization_msgs::Marker> ("/camera/text_pose/marker", 1);
	ros::spin();

}

