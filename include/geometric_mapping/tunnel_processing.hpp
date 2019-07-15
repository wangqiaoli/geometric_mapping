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


////////////////////////////////////////////////////////
//Declare Timing Functions
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
//Declare Window struct
////////////////////////////////////////////////////////

//for sliding window in registered point clouds
struct Window {
	const bool isRegistered;
	const int size;
	std::deque<pcl::PointCloud<pcl::PointXYZ>> cloudWindow;
	std::deque<nav_msgs::Odometry> odometryWindow;
}

////////////////////////////////////////////////////////
//Declare Point Cloud Processing Functions
////////////////////////////////////////////////////////

//Creates Registered cloud from time series data
pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloudUpdate(Window*& window, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

//Chops point cloud at each timestep
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(const Eigen::Array3f& bounds, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(
												const double& neighborRadius, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree, //set to dynamic memory in function
												std::vector<int>& indicesMap
											);

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
void getLocalFrame(
					const int& cloudSize, 
					const double& weightingFactor, //on [.1, .3]
					const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
					Eigen::Vector3f*& eigenVals, //want to set to dynamic memory in function
					Eigen::Matrix3f*& eigenVecs //want to set to dynamic memory in function
				  );

//Regression function
Eigen::VectorXf* getCylinder(
								const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
								const pcl::PointCloud<pcl::Normal>::Ptr& normals
							);

//state machine for detecting walls, orientation, intersections, and tunnels


////////////////////////////////////////////////////////
//Declare Visualization Functions
////////////////////////////////////////////////////////

//Displays single arrow in rviz
visualization_msgs::Marker* rvizArrow(
										const Eigen::Vector3f& start, 
										const Eigen::Vector3f& end,
										const Eigen::Vector3f& scale, 
										const Eigen::Vector4f& color,
										const std::string& ns,
										const int& id = 0,
										const std::string& frame = "/velodyne"
									 );

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
visualization_msgs::MarkerArray* rvizNormals(
												const double& leafSize, 
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
												const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree,
												const std::vector<int>& indicesMap,
												const pcl::PointCloud<pcl::Normal>::Ptr& normals
											);

//Displays eigenspace in rviz
visualization_msgs::MarkerArray* rvizEigens(const Eigen::Vector3f& eigenVals, const Eigen::Matrix3f& eigenVecs);

//Displays regression in rviz
visualization_msgs::Marker* rvizCylinder(
											const Eigen::Array3f& bounds,
											const Eigen::VectorXf& cylinderCoeffs,
											const Eigen::Vector3f& centerAxis,
											const Eigen::Vector4f& color,
											const std::string& ns,
											const int& id = 0,
											const std::string& frame = "/velodyne"
										);

//Displays cloud and normals on PCL Visualizer
void pclvizNormals(
					int& pcl_var,
					pcl::visualization::PCLVisualizer& viewer, 
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
					const pcl::PointCloud<pcl::Normal>::Ptr& normals
				  );

#endif