#include <iostream> 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <fstream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/people/height_map_2d.h>
#include <pcl/filters/voxel_grid.h>
//ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"
#include <tf/transform_listener.h>
//opencv
#include <opencv2/opencv.hpp>

using namespace std;
using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
double* extrinsic;
double extrinsic_array[7];
ros::Subscriber pointcloud_subscriber;
ros::Publisher pointcloud_publisher;
ros::Publisher voxel_pub;
ros::Publisher model_coeff_pub;
ros::Publisher pcl2trans_pub;
ros::Publisher pcl2transerror_pub;
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
double* read_camera_extrinsic();
tf::StampedTransform transformtf;
tf::TransformListener* listener;
PointCloud<PointXYZRGB>::Ptr realcloud (new pcl::PointCloud<pcl::PointXYZRGB> ()); 
bool get_height_map(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
int main(int argc, char** argv){
//ros node init
	//extrinsic = read_camera_extrinsic();
	/*
	int i=0;
	for (i=0;i<7;i++){
		cout << extrinsic[i] << endl;
		extrinsic_array[i]=extrinsic[i];
	}
	*/
	ros::init (argc, argv, "get_height_map");
	ros::NodeHandle nh; 
	tf::TransformListener lr(ros::Duration(10));
	listener = &lr; 
	ros::Rate rate(1.0);
	rate.sleep();
	rate.sleep();
	//listener->lookupTransform( "/camera_rgb_optical_frame", "/map", ros::Time(0), transformtf);
	pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, pointcloud_cb);
	//pointcloud_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/camera_link/moving_object", 1, pointcloud_cb);
	pointcloud_publisher = nh.advertise<PointCloudXYZRGB> ("/cropedpointcloud", 1);
    //voxel_pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxel", 1);
	//model_coeff_pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
	pcl2trans_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl2trans", 1);
	pcl2transerror_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pcl2transerror", 1);
	ros::ServiceServer service = nh.advertiseService("/get_height_map", get_height_map);

	ros::spin ();
	return 0;
}
double* read_camera_extrinsic(){
	double data[7];
	ifstream infile("/home/nctuece/catkin_ws/src/heightmap_creator/camera.txt");
	char line[1000];
	while(infile) {
		infile.getline(line, 1000); 

		char* pch;
		const char* delim = " ";
		int count = 0;
		pch = strtok(line,delim);
		while (pch != NULL && count < 7)
		{
			cout << pch << endl;
			data[count] = atof(pch);
  //cout << data[count] << endl;
			pch = strtok (NULL, delim);
			count++;
		} 

	}
	infile.close();
	cout << endl; 
	return data;
}
void pointcloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	//cout << input->header.frame_id << endl;
	//PointCloud<PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	//PointCloud<PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	
	////////////////////get pointcloud from rostopic
	//pcl::fromROSMsg (*input, *source_cloud);
	
    ////////////////////TRANSFORM(by APi) pointcloud from camera to map////////////////////
	sensor_msgs::PointCloud2 transcloud, outputcloud;
	const string map = "/map";
	pcl_ros::transformPointCloud(map, *input, transcloud, *listener);
	pcl2trans_pub.publish(transcloud);
	

	////////////////////translate error////////////////////
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	//if needed
	//transform (0,3) = -0.012;
	pcl_ros::transformPointCloud(transform, transcloud, outputcloud);
	pcl2transerror_pub.publish(outputcloud);
	
	////////////////////crop pointcloud////////////////////
	// (x/2, y/2) = (0.12,0.20)
	
	pcl::fromROSMsg (outputcloud, *realcloud);

	pointcloud_publisher.publish(*realcloud);


	////////////////////TRANSFORMATION BY HAND(not matching truth////////////////////
    /*
	Eigen::Matrix3f rotmat = Eigen::Quaternionf(transformtf.getRotation().w(), transformtf.getRotation().x(), transformtf.getRotation().y(), transformtf.getRotation().z()).toRotationMatrix();
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform.block(0,0,3,3) = rotmat;
	transform (0,3) = transformtf.getOrigin().x();
	transform (1,3) = transformtf.getOrigin().y();
	transform (2,3) = transformtf.getOrigin().z();
	//cout << transformtf.getOrigin().x() << " " << transformtf.getOrigin().y() << " " << transformtf.getOrigin().z() << endl;
	//cout << transformtf.getRotation().x() << " " << transformtf.getRotation().y() << " " << transformtf.getRotation().z() << " " << transformtf.getRotation().w() << endl;
	transformPointCloud (*source_cloud, *transformed_cloud, transform);
	transformed_cloud->header.frame_id = "/map";
	*/
	//cout << transformed_cloud->width << " " << transformed_cloud->height << endl;
	

	//Eigen::Quaternionf q = transformed_cloud->sensor_orientation_;
	//cout << transformed_cloud->sensor_origin_ << " " <<  q.x() << " " <<  q.y() << " " <<  q.z() << " " << q.w() << endl;
	/*
	for (int i=0; i<transformed_cloud->size(); i++){
		if(transformed_cloud->points[i].g>40){
			transformed_cloud->points[i].r=0;
			transformed_cloud->points[i].g=255;
			transformed_cloud->points[i].r=0;
		}
	
	}
	
	pointcloud_publisher.publish(*transformed_cloud);
	*/
	

	////////////////////PCL HEIGHTMAP API////////////////////

	//heightmap api still not working
	/*
	pcl::people::HeightMap2D<pcl::PointXYZRGB> heightmap;
	heightmap.setInputCloud(transformed_cloud);
	vector<int> heightmapvec = heightmap.getHeightMap();
	for(int i=0; i<heightmapvec.size(); i++)
 		cout << heightmapvec[i] << endl;
 	*/
	

	////////////////////VOXEL GRID//////////////////////////
	// Container for original & filtered data

	/*
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
	pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (cloud_filtered);

  // Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
	voxel_pub.publish (output);
	*/
	
}
bool get_height_map(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
	
	if (req.data == true){
		ROS_INFO( "service: get heightmap\n");
	}

	
	int rowCount = 224;
	int colCount = 320;
	double upperleftx = -0.12;
	double upperlefty = 0.2;
	double gapx = 0.12/100;
	double gapy = 0.2/150;
	cv::Mat colorimg(rowCount, colCount, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat depthimg(rowCount, colCount, CV_8U, cv::Scalar(0));
	int indexx = 0;
	int indexy = 0;
	
	for (int i=0; i<realcloud->size(); i++){
		if((realcloud->points[i].x>-0.12) && (realcloud->points[i].x<0.12) && (realcloud->points[i].y>-0.2) && (realcloud->points[i].y<0.2)){
			indexx = int(floor((realcloud->points[i].x-upperleftx)/gapx)+12);
			indexy = int(floor((realcloud->points[i].y-upperlefty)/gapy)+10);
			colorimg.at<cv::Vec3b>(indexx, indexy)[0]=realcloud->points[i].b;
			colorimg.at<cv::Vec3b>(indexx, indexy)[1]=realcloud->points[i].g;
			colorimg.at<cv::Vec3b>(indexx, indexy)[2]=realcloud->points[i].r;
			if (depthimg.at<uchar>(indexx, indexy)<realcloud->points[i].z)
				depthimg.at<uchar>(indexx, indexy)=realcloud->points[i].z;
		}
	}
	
	cv::imwrite( "/home/nctuece/colorimage.jpg", colorimg );
	cv::imwrite( "/home/nctuece/depthimage.jpg", depthimg );
	cout << "image write to file\n";
	res.success = true;
	return true;

}
