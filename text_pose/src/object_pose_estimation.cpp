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

void  test1 (const sensor_msgs::PointCloud2ConstPtr& point)
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
				if	(cv_ptr->image.at<uchar>(row,column)!=0 && original_point->points[count].z<100){
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

		
		

		

		printf ("x,y,z,normalx,normaly,normalz: %lf %lf %lf %lf %lf %lf\n",original_point->points[point_index].x,original_point->points[point_index].y,original_point->points[point_index].z,cloud_normals->points[point_index].normal[0],cloud_normals->points[point_index].normal[1],cloud_normals->points[point_index].normal[2]);
	
		geometry_msgs::PoseStamped::Ptr objectpose(new geometry_msgs::PoseStamped);
		objectpose->header = point->header;
		objectpose->pose.position.x=original_point->points[point_index].x;
		objectpose->pose.position.y=original_point->points[point_index].y;
		objectpose->pose.position.z=original_point->points[point_index].z;
		objectpose->pose.orientation.x=cloud_normals->points[point_index].normal[0];
		objectpose->pose.orientation.y=cloud_normals->points[point_index].normal[1];
		objectpose->pose.orientation.z=cloud_normals->points[point_index].normal[2];

		visualization_msgs::Marker marker;
   		marker.header.frame_id = "camera_rgb_optical_frame";
   		marker.header.stamp = ros::Time();
   		marker.ns = "my_namespace";
   		marker.id = 0;
   		marker.type = visualization_msgs::Marker::ARROW;
   		marker.action = visualization_msgs::Marker::ADD;
   		marker.pose.position.x = original_point->points[point_index].x;
   		marker.pose.position.y = original_point->points[point_index].y;
  		marker.pose.position.z = original_point->points[point_index].z;
  		marker.pose.orientation.x = cloud_normals->points[point_index].normal[0];
  		marker.pose.orientation.y = cloud_normals->points[point_index].normal[1];
  		marker.pose.orientation.z = cloud_normals->points[point_index].normal[2];
  		marker.pose.orientation.w = 1.0;
  		marker.scale.x = 1;
  		marker.scale.y = 0.1;
  		marker.scale.z = 0.1;
  		marker.color.a = 1.0;
  		marker.color.r = 0.0;
  		marker.color.g = 1.0;
  		marker.color.b = 0.0;




		pose_publisher.publish(objectpose);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*original_point, *original_point, indices);
		align_cloud_publisher.publish(original_point);
		marker_publisher.publish(marker);
		pcl::io::savePCDFileASCII ("test_pcd.pcd",*original_point);
	}
	
	return;
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

