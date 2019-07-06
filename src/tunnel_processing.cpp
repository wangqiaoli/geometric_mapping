#ifndef PROCESSCLOUD_CPP
#define PROCESSCLOUD_CPP

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
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(const double& bound ,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	//Apply Box Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-bound, -bound, -bound, 1.0));
	boxFilter.setMax(Eigen::Vector4f(bound, bound, bound, 1.0));
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cloudChopped);

	return cloudChopped;
}

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(const double& neighborRadius, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	//Create the normal estimation class
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> find_normals;
  	find_normals.setInputCloud(cloud);

  	//create KD tree of the point cloud
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  	find_normals.setSearchMethod(tree);

  	//Find normals using nearest neighbor parameter
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  	find_normals.setRadiusSearch(neighborRadius);
  	find_normals.compute(*cloud_normals);

  	// std::cout << cloud_normals->at(11) << std::endl;
  	// std::cout << "Cloud size is:" << cloud_normals->points.size() << std::endl;
  	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  	pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, indices->indices);
  	// std::cout << "New cloud size is:" << cloud_normals->points.size() << std::endl;
  	// std::cout << cloud_normals->at(11) << std::endl << std::endl;

  	//remove points from indices list
  	//could use this to display normals at each voxel
  	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
  	extractIndices.setInputCloud(cloud);
  	extractIndices.setIndices(indices);
  	//extractIndices.setNegative(true);
  	extractIndices.filter(*cloud);
  	// std::cout << "Cloud size is " << point_cloud->points.size() << std::endl;

  	return cloud_normals;
}

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
Eigen::Matrix3f* getLocalFrame(const int& cloudSize, const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals) {
	//CENTER Axis -> USE EIGEN
	//initialize weight matrix with curvatures and weight factor
	Eigen::MatrixXf weights = Eigen::MatrixXf::Zero(cloudSize, cloudSize); //(nxn)
	float weightingFactor = 0.2; //paper says in range [.1,.3]

	std::cout << "Weights made of size:\t" << cloudSize << std::endl;

	for(int i = 0; i < cloudSize; i++) {
		// std::cout << "Curvature #" << i << " is: \t" << cloud_normals->at(i).curvature << std::endl;
		weights(i,i) = std::exp(std::pow(cloud_normals->at(i).curvature + .001 / weightingFactor, 2));
	}

	//Initialize normals with normal vectors (nx3)
	Eigen::MatrixXf normals(cloudSize, 3);
	for(int i = 0; i < cloudSize; i++) {
		normals(i,0) = cloud_normals->at(i).normal[0];
		normals(i,1) = cloud_normals->at(i).normal[1];
		normals(i,2) = cloud_normals->at(i).normal[2];
		// std::cout << "Normal #" << i << " is: \t" << normals.row(i) << std::endl;
	} // std::cout << "Normals printed" << std::endl;

	//create weighted normals matrix (nx3)
	Eigen::MatrixXf weightedNormals = weights * normals; //(nxn) x (nx3)

	ROS_INFO("Weighted normals found");

	//Form normal intensity matrix: (3x3) = (3xn) x (nx3)
	Eigen::MatrixXf intensity =  weightedNormals.transpose() * weightedNormals;

	ROS_INFO("Intensity found");

	//Take eigenvectors of the intensity matrix
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(intensity);

	Eigen::MatrixXf eigenVals = eigenSolver.eigenvalues(); //(3x1)
	std::cout << "Eigenvalues are:\n" << eigenSolver.eigenvalues() << std::endl;

	// Eigen::MatrixXf eigenVecs = eigenSolver.eigenvectors(); //(3x3)
	// std::cout << "Eigenvectors are:\n" << eigenSolver.eigenvectors() << std::endl;

	//use minimum eigenvector as center axis and display on rviz
	Eigen::Vector3f* centerAxis(new Eigen::Vector3f);
	*centerAxis = eigenSolver.eigenvectors().block<3,1>(0,0); // first eigenvec is always min

	std::cout << "Center Axis is:\n" << centerAxis->transpose() << std::endl;

	return centerAxis;
}

////////////////////////////////////////////////////////
//Declare Visualization Functions
////////////////////////////////////////////////////////

//Displays single arrow in rviz
visualization_msgs::Marker* rvizArrow(
										const Eigen::Vector3f& start, 
										const Eigen::Vector3f& end,
										const std::string& ns,
										const std::string& frame = "/velodyne"
									  ) {

	visualization_msgs::Marker* centerAxisVec(new visualization_msgs::Marker);

	//set normal parameters
	centerAxisVec->header.frame_id = frame;
	centerAxisVec->header.stamp = ros::Time::now();
	centerAxisVec->header.seq = 0;
	centerAxisVec->ns = ns;
	centerAxisVec->id = 0;
	centerAxisVec->type = visualization_msgs::Marker::ARROW;
	centerAxisVec->action = visualization_msgs::Marker::ADD;

	//set start [0] (zero coords velodyne frame) and end [1] points of arrow
	centerAxisVec->points.resize(2);

	centerAxisVec->points[0].x = start(0);
	centerAxisVec->points[0].y = start(1);
	centerAxisVec->points[0].z = start(2);

	centerAxisVec->points[1].x = end(0);
	centerAxisVec->points[1].y = end(1);
	centerAxisVec->points[1].z = end(2);

	//set normal scales
	centerAxisVec->scale.x = 0.05;
	centerAxisVec->scale.y = 0.08;
	centerAxisVec->scale.z = 0.01;

	//set normal colors
	centerAxisVec->color.a = 1.0;
	centerAxisVec->color.r = 0.0;
	centerAxisVec->color.g = 0.0;
	centerAxisVec->color.b = 1.0;

	return centerAxisVec;
}

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
visualization_msgs::MarkerArray* rvizNormals(
												const double& leafSize, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
												pcl::PointCloud<pcl::Normal>::Ptr normals
											) {
 //  	//Implement VoxelGrid Filter
	// pcl::PointCloud<pcl::PointXYZ> cloudVoxelFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	// double leafSize = params->getLeafSize();

	// pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> voxelGridFilter; //create voxelgrid object
	// voxelGridFilter.setInputCloud(cloudBoxFilteredPtr);
	// voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
	// voxelGridFilter.filter(*cloudVoxelFiltered);

	// pcl::PCLPointCloud2ConstPtr cloudVoxelFilteredPtr(cloudVoxelFiltered);

	// ROS_INFO("VoxelGrid filter applied...");

	// //Use KD tree to build cloud of nearest neighbors to voxelgrid cloud


	// //Display normals of the nearest neighbor cloud

	int width = 0;
	visualization_msgs::MarkerArray* normalVecs(new visualization_msgs::MarkerArray);
	// for(int i = 0; i < cloud->points().size(); i++) {
	// 	rvizArrow()
	// }

	return normalVecs;
}

//Displays cloud and normals on PCL Visualizer
void pclvizNormals(
					int& pcl_var,
					pcl::visualization::PCLVisualizer& viewer, 
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
					const pcl::PointCloud<pcl::Normal>::Ptr& normals
				  ) {
  	viewer.setBackgroundColor (0, 0, 0); //black
  	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud" + std::to_string(pcl_var));
  	//PCL_VISUALIZER_POINT_SIZE is int starting from 1
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + std::to_string(pcl_var));
  	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, .1, "normals" + std::to_string(pcl_var));
  	viewer.addCoordinateSystem (1.0);
  	viewer.initCameraParameters();
  	viewer.spinOnce();
  	pcl_var++;
}

#endif