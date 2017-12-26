#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
// PCL specific includes

#include <ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/CameraInfo.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include "pcl_ros/point_cloud.h"

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

// cmath map for transformation
#include <string>
#include <cmath>
#include <map>

//Publisher

ros::Publisher align_cloud_publisher;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
using namespace message_filters;
using namespace cv;
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr object1 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object2 (new pcl::PointCloud<pcl::PointXYZRGB>);


//     [fx'  0  cx' Tx]
// P = [ 0  fy' cy' Ty]
//    [ 0   0   1   0]
double fx,fy,cx,cy ;



void  test (const sensor_msgs::PointCloud2ConstPtr& point, const sensor_msgs::ImageConstPtr& rgb_image)//
{ 
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8); 
	PointCloudRGB::Ptr point_original(new PointCloudRGB);
	pcl::fromROSMsg(*point,*point_original);
	int count=0;
	for (int row=0;row<480;row++){
		for(int column=0;column<640;column++){	
			if (point_original->points[count].z<100)
				printf("x: %lf,Y: %lf,z: %lf\n",point_original->points[count].x,point_original->points[count].y,point_original->points[count].z);
			count++;						
		}
	}

}

void  brand_prediction (const sensor_msgs::PointCloud2ConstPtr& point, const sensor_msgs::ImageConstPtr& rgb_image)//
{ 
	



}



int main(int argc, char** argv){


	ros::init(argc, argv, "sensor_unit");
	ros::NodeHandle nh; 

	
	/////////////////////////////Declare camera 1 subscriber/////////////////////////////
   
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_camera(nh,"/camera/depth_registered/points", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_image_camera(nh, "/camera/rgb/image_color", 10);
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy1;
    Synchronizer<MySyncPolicy1> sync1(MySyncPolicy1(10), depth_camera, rgb_image_camera); //, camera_info1
    sync1.registerCallback(boost::bind(&test,_1,_2));  //,_3
    
   	align_cloud_publisher = nh.advertise<PointCloudRGB> ("/camera/align_cloud", 1);
   	align_cloud_publisher = nh.advertise<PointCloudRGB> ("/camera/align_cloud", 1);


	ros::spin();

}

