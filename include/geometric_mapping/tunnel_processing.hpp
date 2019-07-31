#ifndef TUNNEL_PROCESSING_HPP
#define TUNNEL_PROCESSING_HPP


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

////////////////////////////////////////////////////////
//Declare Window struct
////////////////////////////////////////////////////////

//for sliding window in registered point clouds
struct Window {
	bool isRegistered = false;
	int size = 1;
	std::deque<pcl::PointCloud<pcl::PointXYZ>> cloudWindow;
	std::deque<nav_msgs::Odometry> odometryWindow;
	// std::map<double, uint> states; //keeps track of distance traveled since last regression and state
};

////////////////////////////////////////////////////////
//Declare Chrono Timing Functions (global vector of times)
////////////////////////////////////////////////////////

//chrono initializer


//chrono stop


////////////////////////////////////////////////////////
//Declare Point Cloud Processing Functions
////////////////////////////////////////////////////////

//Creates Registered cloud from time series data
pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloudUpdate(
															// visualization_msgs::MarkerArray*& debugMarkers,
															Window& window, 
															const tf2_ros::Buffer& tfBuffer,
															const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
															const std::string& baseFrame = "world"
						  								 );

//Chops point cloud at each timestep
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const Window& window,
												const tf2_ros::Buffer& tfBuffer,
												const Eigen::Array3f& bounds, 
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												const std::string& baseFrame = "world"
											 );

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const double& neighborRadius, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree, //set to dynamic memory in function
												std::vector<int>& indicesMap
											);

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
void getLocalFrame(
					// visualization_msgs::MarkerArray*& debugMarkers,
					const int& cloudSize, 
					const double& weightingFactor, //on [.1, .3]
					const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
					boost::shared_ptr<Eigen::Vector3f>& eigenVals,
					boost::shared_ptr<Eigen::Matrix3f>& eigenVecs
				  );

//Regression function 
boost::shared_ptr<Eigen::VectorXf> getCylinder(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const double& distThreshold,
												const Eigen::Vector3f& centerAxis,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												const pcl::PointCloud<pcl::Normal>::Ptr& normals
											  );

//state machine for detecting walls, orientation, intersections, and tunnels
// void stateMachine(	
// 					Window& window;
// 					boost::shared_ptr<Eigen::Vector3f>& eigenVals,
//					boost::shared_ptr<Eigen::Matrix3f>& eigenVecs
// 				 );

////////////////////////////////////////////////////////
//Declare Visualization Functions
////////////////////////////////////////////////////////

//Displays single arrow in rviz
boost::shared_ptr<visualization_msgs::Marker> rvizArrow(
															const Eigen::Vector3f& start, 
															const Eigen::Vector3f& end,
															const Eigen::Vector3f& scale, 
															const Eigen::Vector4f& color,
															const std::string& ns,
															const int& id = 0,
															const std::string& frame = "/world"
									 					);

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
boost::shared_ptr<visualization_msgs::MarkerArray> rvizNormals(
																const double& leafSize, 
																const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
																const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree,
																const std::vector<int>& indicesMap,
																const pcl::PointCloud<pcl::Normal>::Ptr& normals
															  );

//Displays eigenspace in rviz
boost::shared_ptr<visualization_msgs::MarkerArray> rvizEigens(
																const boost::shared_ptr<Eigen::Vector3f>& eigenVals,
																const boost::shared_ptr<Eigen::Matrix3f>& eigenVecs
															 );

//Displays segment barriers
// visualization_msgs::MarkerArray* rvizWall();

//Displays regression in rviz
boost::shared_ptr<visualization_msgs::Marker> rvizCylinder(
															const Eigen::Array3f& bounds,
															const Eigen::VectorXf& cylinderCoeffs,
															const Eigen::Vector3f& centerAxis,
															const Eigen::Vector4f& color,
															const std::string& ns,
															const int& id = 0,
															const std::string& baseFrame = "/world"
														  );

//Displays Segments
// visualization_msgs::MarkerArray* rvizSegments();

//Displays Map
// visualization_msgs::MarkerArray* rvizMap();

//Displays cloud and normals on PCL Visualizer
void pclvizNormals(
					int& pcl_var,
					pcl::visualization::PCLVisualizer& viewer, 
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
					const pcl::PointCloud<pcl::Normal>::Ptr& normals
				  );

#endif