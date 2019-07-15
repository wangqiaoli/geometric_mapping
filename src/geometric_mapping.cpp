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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Eigen libraries
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Geometry> 

//Project libraries
#include "paramHandler.hpp"
#include "tunnel_processing.hpp"

//Creates sliding window for registered point cloud
Window* window = nullptr;

//Creates parameter object
Parameters* params = nullptr;

//Create ROS publisher objects
ros::Publisher cloudPub;
ros::Publisher normalsPub;
ros::Publisher centerAxisPub;
ros::Publisher cylinderPub;

//create PCL visualizer object
int pcl_var = 0;
pcl::visualization::PCLVisualizer* viewer = nullptr;

//Define Cloud Publisher function
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
	ROS_INFO("Callback started...");

	//create a container for the data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//Convert to PCL
	pcl::fromROSMsg(*input, *cloud);

	//add cloud to window
	window.cloudWindow.push_front(*cloud);

	//update sliding window if needed
	if(window->isRegistered) {
		cloud = registeredCloudUpdate(*window, *cloud);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped = chopCloud(*params->getBoxFilterBounds(), cloud);

	ROS_INFO("Box filter applied...");

	//Find Surface Normals
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(nullptr);
	std::vector<int> indicesMap;
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals = getNormals(params->getNeighborRadius(), cloudChopped, kdtree, indicesMap);

	ROS_INFO("Surface normals found...");

	Eigen::Vector3f* eigenVals = nullptr;
	Eigen::Matrix3f* eigenVecs = nullptr;

	getLocalFrame(
					cloudChopped->points.size(), 
					params->getWeightingFactor(),
					cloudNormals,
					eigenVals,
					eigenVecs
				  );

	//get center axis and assume that first eigenvec is smallest
	Eigen::Vector3f* centerAxis(new Eigen::Vector3f);
	*centerAxis = eigenVecs->block<3,1>(0,0);

	ROS_INFO("Center Axis found...");

	Eigen::VectorXf* cylinderModel = getCylinder(
													cloudChopped,
													cloudNormals
												);

	ROS_INFO("Local Cylinder Model found...");

	if(params->displayCloud()) {
		//convert the pcl::PointCloud to ros message
		sensor_msgs::PointCloud2 cloudROS;
		pcl::toROSMsg(*cloudChopped, cloudROS);

		//Publish the box filtered cloud
		cloudPub.publish(cloudROS);

		ROS_INFO("Chopped cloud posted to rviz...");
	}

	//visualize normals in pcl
  	if(params->usePCLViz()) {
  		pclvizNormals(pcl_var, *viewer, cloudChopped, cloudNormals);
  	}

	if(params->displayNormals()) {
	  	visualization_msgs::MarkerArray* normalsDisp = rvizNormals(
	  																params->getLeafSize(),
	  																cloudChopped,
	  																kdtree,
	  																indicesMap,
	  																cloudNormals
	  															  );

		//Publish the normals
		normalsPub.publish(*normalsDisp);
	}

	if(params->displayCenterAxis()) {
		visualization_msgs::MarkerArray* eigenBasis = rvizEigens(*eigenVals, *eigenVecs);

		//Publish the center axis
		centerAxisPub.publish(*eigenBasis);
	}

	if(params->displayCylinder()) {
		Eigen::Vector4f color(.6, 1, .65, 0);
		visualization_msgs::Marker* cylinderDisp = rvizCylinder(
																	*params->getBoxFilterBounds(),
																	*cylinderModel,
																	*centerAxis,
																	color,
																	"cylinderModel"
																);

		//Publish the data
		cylinderPub.publish(*cylinderDisp);
	}

	ROS_INFO("Callback ended...\n\n");
}

//Define Cloud Publisher function
void odometry_cb(const nav_msgs::Odometry odometry) {
	//push new odometry onto window
	window->odometryWindow.push_front(odometry);
}

//main function creates subscriber for the published point cloud
int main(int argc, char** argv) {
	//Initialize ROS cloud topic
	ros::init(argc, argv, "geometric_mapping_node");
	ros::NodeHandle node;

	//Parameters params;
	Parameters param(node);
	params = &param;

	ROS_INFO("Launched geometric_mapping_node...");

	//initialize sliding window
	if(params->getWindowSize() > 1) {
		window = new Window;
		window->isRegistered = true;
		window->size = params->getWindowSize();
	}

	ROS_INFO_STREAM("Created " << (window->isRegistered) ? "sliding window..." : "cloud...");

	//inits PCL viewer
	if(params->usePCLViz()) {
		pcl::visualization::PCLVisualizer pclViewer;
		viewer = &pclViewer;
	}

	//Create subscriber for the input pointcloud
	ros::Subscriber cloudSub = node.subscribe("inputCloud", 1, cloud_cb);

	//Create subscriber for the input odometry
	ros::Subscriber odometrySub = node.subscribe("inputOdometry", 1, odometry_cb);

	//Create ROS publisher for box filtered point cloud
	if(params->displayCloud()) {
		cloudPub = node.advertise<sensor_msgs::PointCloud2>("cloudOutput", 10);
	}

	//Create ROS publisher for normals
	if(params->displayNormals()) {
		normalsPub = node.advertise<visualization_msgs::MarkerArray>("normalsOutput", 10);
	}

	//Create ROS publisher for center Axis and scaled eigenvecs
	if(params->displayCenterAxis()) {
		centerAxisPub = node.advertise<visualization_msgs::MarkerArray>("eigenBasisOutput", 10);
	}

	//Create ROS publisher for cylinder
	if(params->displayCylinder()) {
		cylinderPub = node.advertise<visualization_msgs::Marker>("cylinderOutput", 10);
	}

	//Calls message callbacks rapidly in seperate threads
	ros::spin();

	//deletes the window
	delete window;
	window = nullptr;
}

#endif