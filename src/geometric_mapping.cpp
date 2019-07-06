#ifndef PCL_PRACTICE_CPP
#define PCL_PRACTICE_CPP

//standard functions
#include <limits>
#include <cmath>

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

//PCL libraries
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

//Eigen libraries
#include <Eigen/Core>
#include <Eigen/QR>

//Project libraries
#include "paramHandler.hpp"
#include "processCloud.hpp"

int pcl_var = 0;
//pcl::visualization::PCLVisualizer viewer;

//Create ROS publisher object
ros::Publisher pub;

//Creates parameter object
Parameters* params = nullptr;

//Define Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Convert to PCL
	pcl::fromROSMsg(*input, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped = chopCloud(params->getBoxFilterBound(), cloud);

	ROS_INFO("Box filter applied...");

	//Find Surface Normals
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals = getNormals(params->getNeighborRadius(), cloudChopped);

  	//visualize normals
  	//pclvizNormals(pcl_var, viewer, cloudChopped, cloudNormals);
	//rvizNormals(cloudChopped, cloudNormals)

	ROS_INFO("Surface normals found...");

	Eigen::Vector3f* centerAxis = getLocalFrame(cloudChopped->points.size(), cloudNormals);

	ROS_INFO("Center Axis found...");

	visualization_msgs::Marker* centerAxisDisp = rvizArrow(Eigen::Vector3f::Zero(), *centerAxis, "centerAxis");

	ROS_INFO("Center Axis Marker Made...");

	//Convert Back to ROS
	sensor_msgs::PointCloud2 output;

	bool dispArrow = true;
	if(!dispArrow) {
		pcl::toROSMsg(*cloudChopped, output);

		//Publish the data
		pub.publish(output);
	} else {
		//Publish the data
		pub.publish(*centerAxisDisp);
	}

	ROS_INFO("Published...");
}

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS
	ros::init(argc, argv, "pcl_practice_node");
	ros::NodeHandle node;

	//Parameters params;
	Parameters param(node);
	params = &param;

	ROS_INFO("Launched pcl_practice_node...");

	//Create subscriber for the input pointcloud
	ros::Subscriber sub = node.subscribe("input", 1, cloud_cb);

	//Create ROS publisher for point cloud
	if(params->getFindSurfaceNormals()) {
		pub = node.advertise<visualization_msgs::Marker>("output", 10);
	} else {
		pub = node.advertise<sensor_msgs::PointCloud2>("output", 10);
	}

	//Calls message callbacks rapidly in seperate threads
	ros::spin();
}

#endif