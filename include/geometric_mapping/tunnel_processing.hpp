ifndef PROCESSCLOUD_HPP
#define PROCESSCLOUD_HPP

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

////////////////////////////////////////////////////////
//Declare Point Cloud Processing Functions
////////////////////////////////////////////////////////

//Chops point cloud at each timestep
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(const double& bound ,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(const double& neighborRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
Eigen::Vector3f* getLocalFrame(const int& cloudSize, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);
//		add weighting factor

//Regression function


////////////////////////////////////////////////////////
//Declare Visualization Functions
////////////////////////////////////////////////////////

//Displays single arrow in rviz
visualization_msgs::Marker* rvizArrow(
										const Eigen::Vector3f& start, 
										const Eigen::Vector3f& end,
										const std::string& ns,
										const std::string& frame = "/velodyne"
									  );

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
visualization_msgs::MarkerArray* rvizNormals(
												const double& leafSize, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
												pcl::PointCloud<pcl::Normal>::Ptr normals
											);

//Displays regression in rviz


//Displays cloud and normals on PCL Visualizer
void pclvizNormals(
					int& pcl_var,
					pcl::visualization::PCLVisualizer& viewer, 
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
					const pcl::PointCloud<pcl::Normal>::Ptr& normals
				  );

#endif