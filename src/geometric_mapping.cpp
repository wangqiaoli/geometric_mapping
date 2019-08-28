#ifndef GEOMETRIC_MAPPING_CPP
#define GEOMETRIC_MAPPING_CPP


//standard functions
#include <vector>
#include <map>
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
boost::shared_ptr<Window> window;
boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
boost::shared_ptr<tf2_ros::TransformListener> tfListener;
boost::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

//Creates parameter object
boost::shared_ptr<Parameters> params;

//Create ROS publisher objects
ros::Publisher regCloudPub;
ros::Publisher choppedCloudPub;
ros::Publisher normalsPub;
ros::Publisher eigenBasisPub;
ros::Publisher cylinderPub;
ros::Publisher segmentsPub;
ros::Publisher mapPub;
// ros::Publisher debuggerPub;

//create PCL visualizer object
int pcl_var = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

////////////////////////////////////////////////////////
//Declare Callback Functions
////////////////////////////////////////////////////////

//Define Odometry Publisher function
void odometry_cb(const nav_msgs::Odometry& inputOdometry);

//Define Cloud Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputCloud);

////////////////////////////////////////////////////////
//Declare Main Function
////////////////////////////////////////////////////////

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS cloud topic
	ros::init(argc, argv, "geometric_mapping");
	ros::NodeHandle node;

	//Parameters params;
	params = boost::make_shared<Parameters>(node);

	ROS_INFO("Launched geometric_mapping_node...\n");

	//initialize sliding window
	if(params->getWindowSize() > 1) {
		//initialize window object
		window = boost::make_shared<Window>();
		window->isRegistered = true;
		window->size = params->getWindowSize();

		//initialize tf listener node
		tfBuffer = boost::make_shared<tf2_ros::Buffer>();
		tfListener = boost::make_shared<tf2_ros::TransformListener>(*tfBuffer);

		//initialize tf broadcaster
		tfBroadcaster = boost::make_shared<tf2_ros::TransformBroadcaster>();
	}

	ROS_INFO_STREAM("Created " << ((window->isRegistered) ? "sliding window..." : "cloud..."));

	//inits PCL viewer
	if(params->usePCLViz()) {
		viewer = boost::make_shared<pcl::visualization::PCLVisualizer>();
	}

	//Create subscriber for the input odometry
	if(window->isRegistered) {
		ros::Subscriber odometrySub = node.subscribe("inputOdometry", 10, odometry_cb);
	}

	//Create subscriber for the input pointcloud
	ros::Subscriber cloudSub = node.subscribe("inputCloud", 10, cloud_cb);

	//create ros publisher for chopped cloud
	if(window->isRegistered) {
		regCloudPub = node.advertise<sensor_msgs::PointCloud2>("regCloudOutput", 10);
	}

	//Create ROS publisher for box filtered point cloud
	if(params->displayClouds()) {
		choppedCloudPub = node.advertise<sensor_msgs::PointCloud2>("choppedCloudOutput", 10);
	}

	// //Create ROS publisher for normals
	// if(params->displayNormals()) {
	// 	normalsPub = node.advertise<visualization_msgs::MarkerArray>("normalsOutput", 10);
	// }

	// //Create ROS publisher for center Axis and scaled eigenvecs
	// if(params->displayCenterAxis()) {
	// 	eigenBasisPub = node.advertise<visualization_msgs::MarkerArray>("eigenBasisOutput", 10);
	// }

	// //Create ROS publisher for cylinder
	// if(params->displayCylinder()) {
	// 	cylinderPub = node.advertise<visualization_msgs::Marker>("cylinderOutput", 10);
	// }

	// //Create ROS publisher for segment barriers
	// if(params->displayCenterAxis()) {
	// 	segmentsPub = node.advertise<visualization_msgs::MarkerArray>("segmentOutput", 10);
	// }

	// //Create ROS publisher for map
	// if(params->displayCylinder()) {
	// 	mapPub = node.advertise<visualization_msgs::MarkerArray>("mapOutput", 10);
	// }

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

//Define Odometry Update function
void odometry_cb(const nav_msgs::Odometry& odometry) {
	//Removes odometry outside sliding window
	while(window->odometryWindow.size() >= window->size) {
		window->odometryWindow.pop_back();
	}

	//set initial pose
	if(window->odometryWindow.empty()) {
		auto initPos = boost::make_shared<Eigen::Vector3d>();
		tf::pointMsgToEigen(window->odometryWindow[0].pose.pose.position, *initPos);
		window->initPos = initPos->cast<float>();

		auto initQuat = boost::make_shared<Eigen::Quaterniond>();
		tf::quaternionMsgToEigen(window->odometryWindow[0].pose.pose.orientation, *initQuat);
		window->initQuat = initQuat->cast<float>();
	}

	//push new odometry onto sliding window
	window->odometryWindow.push_front(odometry);//*odometryInertial); 

	//if odometry is over threshold then update param

}

//Define Cloud Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputCloud) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Convert to PCL
	pcl::fromROSMsg(*inputCloud, *cloud);

	ROS_INFO("Cloud size is:\t %d", (int) cloud->points.size());

	//update sliding window if needed
	if(window->isRegistered) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
		cloud = registeredCloudUpdate(
										window, 
										*tfBuffer, 
										cloudTemp
									 );

		ROS_INFO("Registered cloud size is:\t %d", (int) cloud->points.size());
		ROS_INFO_STREAM("Registered cloud frame is:\t" << cloud->header.frame_id);
	}

	//Chop point cloud in velodyne frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped = chopCloud(
																	*window,
																	*tfBuffer,
																	Eigen::Vector3f(params->getBoxFilterBounds()), 
																	cloud
																);

	//voxel filter point cloud densly
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelFiltered = voxelFilterCloud(
																				params->getLeafSize(),
																				cloudChopped
																			 );

	ROS_INFO("Chopped cloud size is:\t %d", (int) cloudVoxelFiltered->points.size());

	// //Find Surface Normals
	// pcl::PointCloud<pcl::Normal>::Ptr cloudNormals = getNormals(
	// 																params->getNeighborRadius(), 
	// 																cloudVoxelFiltered
	// 															);

	// ROS_INFO("Surface normals found...");

	// auto eigenVals = boost::make_shared<Eigen::Vector3f>();
	// auto eigenVecs = boost::make_shared<Eigen::Matrix3f>();

	// getLocalFrame(
	// 				*window,
	// 				tfBroadcaster,
	// 				cloudVoxelFiltered->points.size(), 
	// 				params->getWeightingFactor(),
	// 				cloudNormals,
	// 				eigenVals,
	// 				eigenVecs
	// 			 );

	// //get center axis and assume that first eigenvec is smallest
	// Eigen::Vector3f centerAxis;
	// centerAxis = eigenVecs->block<3,1>(0,0);

	// ROS_INFO("Center Axis found...");

	// //get local cylinder model using RANSAC
	// auto cylinderModel = getCylinder(
	// 									params->getDistThreshold(),
	// 									centerAxis,
	// 									cloudChopped,
	// 									cloudNormals
	// 								);

	// ROS_INFO("Local Cylinder Model found...");

	// //Segment the point cloud
	// std::vector<CloudSegment> cloudSegments = segmentCloud(
	// 														window,
	// 														*tfBuffer,
	// 														params->getBoxFilterBounds(),
	// 														params->getSegmentBounds(), 
	// 														cloud,
	// 														normals,
	// 														centerAxis
	// 												 	  );

	// //loop over segmented clouds and find regression
	// std::deque<MapSegment> mapSegments mapCloud(
	// 												params->getDistThreshold(),
	// 												centerAxis,
	// 												cloudSegments
	// 											);

	if(params->displayClouds()) {
		//convert the pcl::PointCloud to ros message
		sensor_msgs::PointCloud2 choppedCloudROS;
		pcl::toROSMsg(*cloudChopped, choppedCloudROS);
		choppedCloudPub.publish(choppedCloudROS);

		ROS_INFO("Chopped cloud posted to rviz...");

		//convert the pcl::PointCloud to ros message
		sensor_msgs::PointCloud2 regCloudROS;
		pcl::toROSMsg(*cloud, regCloudROS);

		//Publish the box filtered cloud
		regCloudPub.publish(regCloudROS);

		ROS_INFO("Registered cloud posted to rviz...");
	}

	// //visualize normals in pcl
  	// if(params->usePCLViz()) {
  	// 	pclvizNormals(pcl_var, *viewer, cloudChopped, cloudNormals);
  	// }

  	// //visualize normals in rviz
	// if(params->displayNormals()) {
	//   	auto normalsDisp = rvizNormals(
	//   									params->getrvizNormalSize(),
	//   									params->getrvizNormalFrequency(),
	//   									cloudChopped,
	//   									cloudNormals
	//   								  );

	// 	//Publish the normals
	// 	normalsPub.publish(*normalsDisp);
	// }

	// //visualize center axis in rviz
	// if(params->displayCenterAxis()) {
	// 	auto eigenBasis = rvizEigens(eigenVals, eigenVecs);

	// 	//Publish the center axis
	// 	eigenBasisPub.publish(*eigenBasis);
	// }

	// //visualize cylinder in rviz
	// if(params->displayCylinder()) {
	// 	auto cylinderDisp = rvizCylinder(
	// 										params->getSegmentBounds(),
	// 										*cylinderModel,
	// 										centerAxis,
	// 										Eigen::Vector4f(.6, 1, .65, 0),
	// 										"cylinderModel",
	// 										0,
	// 										"/velodyne"
	// 									);

	// 	//Publish the data
	// 	cylinderPub.publish(*cylinderDisp);
	// }

	// //visualize cloud segments
	// if(params->displaySegments()) {
	// 	auto segmentsDisp = rvizSegments(
	// 									params->getSegmentBounds(),
	// 									cloudSegments,
	// 									centerAxis,
	// 									"segments"
	// 								);

	// 	//publish segments
	// 	segmentsPub.publish(*segmentsDisp);
	// }

	// //visualiza map in rviz
	// if(params->displayMap()) {
	// 	auto mapDisp = rvizMap(
	// 							params->getSegmentBounds(),
	// 							mapSegments,
	// 							centerAxis,
	// 							"map"
	// 						  );

	// 	//publish map
	// 	mapPub.publish(*mapDisp);
	// }

	// //Publish the debugger markers
	// if(params->displayDebugger()) {
	// 	debuggerPub.publish(*debuggerMarkers);
	// }

	ROS_INFO("Callback ended...\n\n");
}

#endif