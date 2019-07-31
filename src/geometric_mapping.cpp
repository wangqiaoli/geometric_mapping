#ifndef PCL_PRACTICE_CPP
#define PCL_PRACTICE_CPP


//standard functions
#include <vector>
#include <deque> 
#include <limits>
#include <cmath>

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

//TF2 libraries
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_eigen/tf2_eigen.h>

//PCL libraries
#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

//Eigen libraries
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry> 
#include <eigen_conversions/eigen_msg.h>

//Project libraries
#include "paramHandler.hpp"
#include "tunnel_processing.hpp"

////////////////////////////////////////////////////////
//Declare Global Variables
////////////////////////////////////////////////////////

//Creates sliding window for registered point cloud
Window window;
tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

//Creates parameter object
Parameters* params = nullptr;

//Create ROS publisher objects
ros::Publisher regCloudPub;
ros::Publisher choppedCloudPub;
ros::Publisher normalsPub;
ros::Publisher eigenBasisPub;
ros::Publisher cylinderPub;
ros::Publisher debuggerPub;

//create PCL visualizer object
int pcl_var = 0;
pcl::visualization::PCLVisualizer* viewer = nullptr;

////////////////////////////////////////////////////////
//Declare Callback Functions
////////////////////////////////////////////////////////

//Define Cloud Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);

//Define Odometry Publisher function
void odometry_cb(const nav_msgs::Odometry& odometry);

////////////////////////////////////////////////////////
//Declare Main Function
////////////////////////////////////////////////////////

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS cloud topic
	ros::init(argc, argv, "geometric_mapping_node");
	ros::NodeHandle node("~");

	//Parameters params;
	Parameters param(node);
	params = &param;

	ROS_INFO("Launched geometric_mapping_node...\n");

	//initialize sliding window
	if(params->getWindowSize() > 1) {
		window.isRegistered = true;
		window.size = params->getWindowSize();

		//initialize tf listener node
		tfBuffer = new tf2_ros::Buffer;
		tfListener = new tf2_ros::TransformListener(*tfBuffer);
	}

	ROS_INFO_STREAM("Created " << ((window.isRegistered) ? "sliding window..." : "cloud..."));

	//inits PCL viewer
	if(params->usePCLViz()) {
		pcl::visualization::PCLVisualizer pclViewer;
		viewer = &pclViewer;
	}

	//Create subscriber for the input odometry
	if(window.isRegistered) {
		ros::Subscriber odometrySub = node.subscribe("inputOdometry", 1, odometry_cb);
	}

	//Create subscriber for the input pointcloud
	ros::Subscriber cloudSub = node.subscribe("inputCloud", 1, cloud_cb);

	//create ros publisher for chopped cloud
	if(window.isRegistered) {
		regCloudPub = node.advertise<sensor_msgs::PointCloud2>("regCloudOutput", 10);
	}

	//Create ROS publisher for box filtered point cloud
	if(params->displayClouds()) {
		choppedCloudPub = node.advertise<sensor_msgs::PointCloud2>("choppedCloudOutput", 10);
	}

	//Create ROS publisher for normals
	if(params->displayNormals()) {
		normalsPub = node.advertise<visualization_msgs::MarkerArray>("normalsOutput", 10);
	}

	//Create ROS publisher for center Axis and scaled eigenvecs
	if(params->displayCenterAxis()) {
		eigenBasisPub = node.advertise<visualization_msgs::MarkerArray>("eigenBasisOutput", 10);
	}

	//Create ROS publisher for cylinder
	if(params->displayCylinder()) {
		cylinderPub = node.advertise<visualization_msgs::Marker>("cylinderOutput", 10);
	}

	// //Create ROS publisher for debugging markers
	// if(params->displayDebugger()) {
	// 	debuggerPub = node.advertise<visualization_msgs::MarkerArray>("debuggerOutput", 10);
	// }

	//Calls message callbacks rapidly in seperate threads
	ros::spin();
}

////////////////////////////////////////////////////////
//Define Callback Functions
////////////////////////////////////////////////////////

//Define Cloud Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Convert to PCL
	pcl::fromROSMsg(*input, *cloud);

	ROS_INFO("Cloud size is:\t %d", (int) cloud->points.size());

	//create rviz debugger
	auto debuggerMarkers = boost::make_shared<visualization_msgs::MarkerArray>();

	//update sliding window if needed
	if(window.isRegistered) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		cloud = registeredCloudUpdate(
										window, 
										*tfBuffer, 
										cloudTemp
									 );

		ROS_INFO("Registered cloud size is:\t %d", (int) cloud->points.size());
		ROS_INFO_STREAM("Registered cloud frame is:\t" << cloud->header.frame_id);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped = chopCloud(
																	window,
																	*tfBuffer,
																	*params->getBoxFilterBounds(), 
																	cloud
																);

	ROS_INFO("Box filter applied...");

	ROS_INFO("Chopped cloud size is:\t %d", (int) cloudChopped->points.size());

	//Find Surface Normals
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(nullptr);
	std::vector<int> indicesMap;

	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals = getNormals(
																	params->getNeighborRadius(), 
																	cloudChopped, 
																	kdtree, 
																	indicesMap
																);

	ROS_INFO("Surface normals found...");

	auto eigenVals = boost::make_shared<Eigen::Vector3f>();
	auto eigenVecs = boost::make_shared<Eigen::Matrix3f>();

	getLocalFrame(
					cloudChopped->points.size(), 
					params->getWeightingFactor(),
					cloudNormals,
					eigenVals,
					eigenVecs
				 );

	//get center axis and assume that first eigenvec is smallest
	Eigen::Vector3f centerAxis;
	centerAxis = eigenVecs->block<3,1>(0,0);

	ROS_INFO("Center Axis found...");

	//get cylinder model using RANSAC
	auto cylinderModel = getCylinder(
										params->getDistThreshold(),
										centerAxis,
										cloudChopped,
										cloudNormals
									);

	ROS_INFO("Local Cylinder Model found...");

	if(params->displayClouds()) {
		//convert the pcl::PointCloud to ros message
		sensor_msgs::PointCloud2 regCloudROS;
		pcl::toROSMsg(*cloud, regCloudROS);

		sensor_msgs::PointCloud2 choppedCloudROS;
		pcl::toROSMsg(*cloudChopped, choppedCloudROS);

		//Publish the box filtered cloud
		regCloudPub.publish(regCloudROS);

		choppedCloudPub.publish(choppedCloudROS);

		ROS_INFO("Chopped cloud posted to rviz...");
	}

	// //visualize normals in pcl
 //  	if(params->usePCLViz()) {
 //  		pclvizNormals(pcl_var, *viewer, cloudChopped, cloudNormals);
 //  	}

	// if(params->displayNormals()) {
	//   	visualization_msgs::MarkerArray* normalsDisp = rvizNormals(
	//   																params->getLeafSize(),
	//   																cloudChopped,
	//   																kdtree,
	//   																indicesMap,
	//   																cloudNormals
	//   															  );

	// 	//Publish the normals
	// 	normalsPub.publish(*normalsDisp);
	// }

	// if(params->displayCenterAxis()) {
	// 	visualization_msgs::MarkerArray* eigenBasis = rvizEigens(*eigenVals, *eigenVecs);

	// 	//Publish the center axis
	// 	eigenBasisPub.publish(*eigenBasis);
	// }

	// if(params->displayCylinder()) {
	// 	Eigen::Vector4f color(.6, 1, .65, 0);
	// 	visualization_msgs::Marker* cylinderDisp = rvizCylinder(
	// 																*params->getBoxFilterBounds(),
	// 																*cylinderModel,
	// 																*centerAxis,
	// 																color,
	// 																"cylinderModel"
	// 															);

	// 	//Publish the data
	// 	cylinderPub.publish(*cylinderDisp);
	// }

	// //Publish the debugger markers
	// if(params->displayDebugger()) {
	// 	debuggerPub.publish(*debuggerMarkers);
	// }

	ROS_INFO("Callback ended...\n\n");
}

//Define Odometry Update function
void odometry_cb(const nav_msgs::Odometry& odometry) {
	//Removes odometry outside sliding window
	while(window.odometryWindow.size() >= window.size) {
		window.odometryWindow.pop_back();
	}

	//push new odometry onto sliding window
	window.odometryWindow.push_front(odometry);//*odometryInertial);
}

#endif