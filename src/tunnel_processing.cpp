#ifndef TUNNEL_PROCESSING_CPP
#define TUNNEL_PROCESSING_CPP

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
#include "tunnel_processing.hpp"

////////////////////////////////////////////////////////
//Declare Point Cloud Processing Functions
////////////////////////////////////////////////////////

//Creates Registered cloud from time series data
pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloudUpdate(Window& window, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	if(window.isRegistered) {
		//add cloud to window
		window.cloudWindow.push_front(*cloud);

		//init registered cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));

		//concatinate registered cloud into single local cloud using affine transformations
		Eigen::Vector3f endPose(
									window.odometryWindow[0].pose.pose.position.x,
									window.odometryWindow[0].pose.pose.position.y,
									window.odometryWindow[0].pose.pose.position.z
							   );

		Eigen::Quaternionf endQuat(
									window.odometryWindow[0].pose.pose.orientation.w,
									window.odometryWindow[0].pose.pose.orientation.x,
									window.odometryWindow[0].pose.pose.orientation.y,
									window.odometryWindow[0].pose.pose.orientation.z
								  );

		//DO I NEED TO USE TF2?????

		ROS_INFO("CURRENT WINDOW SIZE: %d", window.size);

		for(int i = 1; i < window.size; i++) {
			//initialize new pose and orientation from the odometry window
			Eigen::Vector3f startPose(
										window.odometryWindow[i].pose.pose.position.x,
										window.odometryWindow[i].pose.pose.position.y,
										window.odometryWindow[i].pose.pose.position.z
							   	    );

			Eigen::Quaternionf startQuat(
										window.odometryWindow[i].pose.pose.orientation.w,
										window.odometryWindow[i].pose.pose.orientation.x,
										window.odometryWindow[i].pose.pose.orientation.y,
										window.odometryWindow[i].pose.pose.orientation.z
								  	  );

			//find difference in pose and orientation relative to the start frame
			Eigen::Quaternionf diffQ = endQuat.inverse() * startQuat;
			Eigen::Vector3f diffV = endPose - startPose;

			//pose in new coordinate system
			Eigen::Vector3f newDiffPose = diffV.transpose() * endQuat.toRotationMatrix();

			//Create affine transformation from start (previous) frame to end (current) frame
			Eigen::Affine3d affine = Eigen::Affine3d::Identity();
			affine.translation() << newDiffPose[0], newDiffPose[1], newDiffPose[2]; //add pose in end frame
			affine.rotate(diffQ.toRotationMatrix()); //add rotation from start frame to end frame

			pcl::PointCloud<pcl::PointXYZ> transformedCloud;
			pcl::transformPointCloud(*window[i], transformedCloud, affine);

			//concatinates current cloud and all previous clouds processed so far
			*registeredCloud += transformedCloud;
		}

		return registeredCloud;
	} else {
		return cloud;
	}
}

//Chops point cloud at each timestep
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(const Eigen::Array3f& bounds ,const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	//Apply Box Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-bounds(0), -bounds(1), -bounds(2), 1.0));
	boxFilter.setMax(Eigen::Vector4f(bounds(0), bounds(1), bounds(2), 1.0));
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cloudChopped);

	return cloudChopped;
}

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(
												const double& neighborRadius, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
												pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree,
												std::vector<int>& indicesMap
											) {
	//Create the normal estimation class
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> findNormals;
  	findNormals.setInputCloud(cloud);

  	//create KD tree of the point cloud
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  	findNormals.setSearchMethod(tree);

  	kdtree = tree;

  	//Find normals using nearest neighbor parameter
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  	findNormals.setRadiusSearch(neighborRadius);
  	findNormals.compute(*cloud_normals);

  	// std::cout << cloud_normals->at(11) << std::endl;
  	// std::cout << "Cloud size is:" << cloud_normals->points.size() << std::endl;
  	pcl::PointIndices::Ptr indicesBanned(new pcl::PointIndices);
  	pcl::removeNaNNormalsFromPointCloud(*cloud_normals, *cloud_normals, indicesBanned->indices);
  	// std::cout << "New cloud size is:" << cloud_normals->points.size() << std::endl;
  	// std::cout << cloud_normals->at(11) << std::endl << std::endl;

  	//remove points from indices list
  	//could use this to display normals at each voxel
  	pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
  	extractIndices.setInputCloud(cloud);
  	extractIndices.setIndices(indicesBanned);
  	extractIndices.filter(*cloud);
  	// std::cout << "Cloud size is " << point_cloud->points.size() << std::endl;

  	//make indices map
  	int banCounter = 0;
  	for(int i = 0 ; i < cloud->points.size(); i++) {
  		bool banned = false;
  		for(int j = i; j < indicesBanned->indices.size(); j++) { //j=i assumes ordered indices and cloud
  			if(i == indicesBanned->indices.at(j)) {
  				banned  = true;
  				continue;
  			}
		}

  		if(!banned) {
  			indicesMap.push_back(banCounter);
  		} else {
  			indicesMap.push_back(-1);
  			banCounter++;
  		}
  	}

  	return cloud_normals;
}

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
void getLocalFrame(
					const int& cloudSize, 
					const double& weightingFactor, //on [.1, .3]
					const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
					Eigen::Vector3f*& eigenVals,
					Eigen::Matrix3f*& eigenVecs
				  ) {
	//initialize weight matrix with curvatures and weight factor
	Eigen::MatrixXf weights = Eigen::MatrixXf::Zero(cloudSize, cloudSize); //(nxn)

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

	Eigen::Vector3f* eigenValues(new Eigen::Vector3f);
	*eigenValues = eigenSolver.eigenvalues(); //(3x1)
	std::cout << "Eigenvalues are:\n" << eigenSolver.eigenvalues().transpose() << std::endl;

	Eigen::Matrix3f* eigenVectors(new Eigen::Matrix3f);
	*eigenVectors = eigenSolver.eigenvectors(); //(3x3)
	std::cout << "Eigenvectors are:\n" << eigenSolver.eigenvectors() << std::endl;

	// //use minimum eigenvector as center axis and display on rviz
	// Eigen::Vector3f* centerAxis(new Eigen::Vector3f);
	// *centerAxis = eigenVectors->block<3,1>(0,0); // first eigenvec is always min

	std::cout << "Center Axis is:\n" << eigenVectors->block<3,1>(0,0).transpose() << std::endl;

	//return values
	eigenVals = eigenValues;
	eigenVecs = eigenVectors;
}

//Regression function using RANSAC
Eigen::VectorXf* getCylinder(
								const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
								const pcl::PointCloud<pcl::Normal>::Ptr& normals
							) {
	//Create segmentation object 
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> RANSAC;

	// pcl::ExtractIndices<PointT> extract;
	pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr cylinderInliers(new pcl::PointIndices);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	RANSAC.setOptimizeCoefficients(true);
	RANSAC.setModelType(pcl::SACMODEL_CYLINDER);
	RANSAC.setMethodType(pcl::SAC_RANSAC);
	RANSAC.setNormalDistanceWeight(0.1);
	RANSAC.setMaxIterations(10000);
	RANSAC.setDistanceThreshold(0.05);
	RANSAC.setRadiusLimits(0, 15);
	RANSAC.setInputCloud(cloud);
	RANSAC.setInputNormals(normals);

	// Obtain the cylinder inliers and coefficients
	RANSAC.segment(*cylinderInliers, *cylinderCoefficients);

	//convert pcl::ModelCoefficients to eigen
	Eigen::VectorXf cylinderCoeffs(7);
	cylinderCoeffs(0) = cylinderCoefficients->values[0];
	cylinderCoeffs(1) = cylinderCoefficients->values[1];
	cylinderCoeffs(2) = cylinderCoefficients->values[2];
	cylinderCoeffs(3) = cylinderCoefficients->values[3];
	cylinderCoeffs(4) = cylinderCoefficients->values[4];
	cylinderCoeffs(5) = cylinderCoefficients->values[5];
	cylinderCoeffs(6) = cylinderCoefficients->values[6];

	// Write the cylinder inliers to disk (DEBUGGING)
	// pcl::PointCloud<PointT>::Ptr cloudCylinder(new pcl::PointCloud<PointT>());

	// extract.setInputCloud(cloud);
	// extract.setIndices(cylinderInliers);
	// extract.setNegative(false);
	// extract.filter(*cloudCylinder);

	// std::cout << "\nCylinder cloud size:\n" 
	// 		  << cloudCylinder->points.size() 
	// 		  << "\ncoefficients:\n" 
	// 		  << cylinderCoeffs->transpose()
	// 		  << std::endl;

	return (new Eigen::VectorXf(cylinderCoeffs));
}

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
										const int& id,
										const std::string& frame
									  ) {
	//declare marker
	visualization_msgs::Marker* centerAxisVec(new visualization_msgs::Marker);

	//set normal parameters
	centerAxisVec->header.frame_id = frame;
	centerAxisVec->header.stamp = ros::Time::now();
	centerAxisVec->header.seq = 0;
	centerAxisVec->ns = ns;
	centerAxisVec->id = id;
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
	centerAxisVec->scale.x = scale(0);
	centerAxisVec->scale.y = scale(1);
	centerAxisVec->scale.z = scale(2);

	//set normal colors
	centerAxisVec->color.a = color(0);
	centerAxisVec->color.r = color(1);
	centerAxisVec->color.g = color(2);
	centerAxisVec->color.b = color(3);

	return centerAxisVec;
}

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
visualization_msgs::MarkerArray* rvizNormals(
												const double& leafSize, 
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
												const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree,
												const std::vector<int>& indicesMap,
												const pcl::PointCloud<pcl::Normal>::Ptr& normals
											) {
	//Implement VoxelGrid Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelFiltered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter; //create voxelgrid object
	voxelGridFilter.setInputCloud(cloud);
	voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
	voxelGridFilter.filter(*cloudVoxelFiltered);

	ROS_INFO("VoxelGrid filter applied...");

	//Add normals of the nearest neighbor cloud to MarkerArray
	visualization_msgs::MarkerArray* normalVecs(new visualization_msgs::MarkerArray);
	normalVecs->markers.resize(cloudVoxelFiltered->points.size());

	std::cout << "Size\t" << cloudVoxelFiltered->points.size() << std::endl;
	std::cout << "SizeN\t" << normals->points.size() << std::endl;

	Eigen::Vector3f startVec = Eigen::Vector3f::Zero();
	Eigen::Vector3f endVec = Eigen::Vector3f::Zero();
	Eigen::Vector3f scaleVec(0.025, 0.075, 0.0625);
	Eigen::Vector4f colorVec(1, 0, 0, 1);

	int KNN = 1;
	int kdint = 0; //garbage
	std::vector<int> kIndices;
	std::vector<float> kDists; //garbage
	for(int i = 0; i < cloudVoxelFiltered->points.size(); i++) {
		//Use KD tree to build cloud of nearest neighbors to KNN cloud
		kdint = kdtree->nearestKSearch(cloudVoxelFiltered->points[i], KNN, kIndices, kDists);

		std::cout << "DEBUG\t:" << i << std::endl;

		if(kdint != 0 && indicesMap.at(i) != -1) {
			//init start point in eigen
			startVec(0) = cloudVoxelFiltered->points[i].x;
			startVec(1) = cloudVoxelFiltered->points[i].y;
			startVec(2) = cloudVoxelFiltered->points[i].z;

			std::cout << "DEBUG2\t:" << kdint << std::endl;
			std::cout << "DEBUG2\t:" << kIndices.at(0) << std::endl;

			//init endpoint in eigen
			endVec(0) = cloudVoxelFiltered->points[i].x + .5*normals->at(kIndices.at(0) - indicesMap.at(i)).normal[0];
			endVec(1) = cloudVoxelFiltered->points[i].y + .5*normals->at(kIndices.at(0) - indicesMap.at(i)).normal[1];
			endVec(2) = cloudVoxelFiltered->points[i].z + .5*normals->at(kIndices.at(0) - indicesMap.at(i)).normal[2];

			std::cout << "DEBUG3\t:" << std::endl;

			normalVecs->markers[i] = *rvizArrow(startVec, endVec, scaleVec, colorVec, "normals", i);
		}
	}

	ROS_INFO("Normals posted to rviz...");

	return normalVecs;
}

//Displays eigenspace in rviz
visualization_msgs::MarkerArray* rvizEigens(const Eigen::Vector3f& eigenVals, const Eigen::Matrix3f& eigenVecs) {
	visualization_msgs::MarkerArray* eigenBasis(new visualization_msgs::MarkerArray);
	eigenBasis->markers.resize(3); //for E1E2E3 basis

	//normalize eigenvals and use to set legnth of arrow basis
	Eigen::Vector3f eigenValNorms = (1/eigenVals.norm()) * eigenVals.cwiseAbs();

	// std::cout << "\n\neigenNorms:" << eigenValNorms.transpose() << std::endl;
	// std::cout << "eigen2:" << eigenVecs.block<3,1>(0,1) << std::endl;
	// std::cout << "eigen3:" << eigenVecs.block<3,1>(0,2) << std::endl;
	// std::cout << "vectest:" << Eigen::Vector4f(1.0, 1.0, 0.0, 0.0) << std::endl;

	for(int i = 0; i < 3; i++) {
		//arrow legnth, arrow width, cone height proportional to eigenvals
		Eigen::Vector3f scale(
								0.1 - (0.05 * eigenValNorms(i)), 
								0.3 - (0.15 * eigenValNorms(i)), 
								0.25 - (0.125 * eigenValNorms(i))
							 );

		Eigen::Vector4f color; //ARGB -> E1E2E3
		if(i == 0) {
			color = Eigen::Vector4f(1.0, 1.0, 0.0, 0.0);
		} else if(i == 1) {
			color = Eigen::Vector4f(1.0, 0.0, 1.0, 0.0);
		} else {
			color = Eigen::Vector4f(1.0, 0.0, 0.0, 1.0);
		}

		eigenBasis->markers[i] = *rvizArrow(
												Eigen::Vector3f::Zero(), 
												(1 + .5*eigenValNorms(i)) * eigenVecs.block<3,1>(0,i), //vecs by col
												scale,
												color,
												"eigenBasis",
												i
											);
	}

	ROS_INFO("Eigenbasis posted to rviz...");

	return eigenBasis;
}

//Displays Regression in rviz
visualization_msgs::Marker* rvizCylinder(
											const Eigen::Array3f& bounds,
											const Eigen::VectorXf& cylinderCoeffs,
											const Eigen::Vector3f& centerAxis, 
											const Eigen::Vector4f& color,
											const std::string& ns,
											const int& id,
											const std::string& frame
										) {
	//declare marker
	visualization_msgs::Marker* cylinder(new visualization_msgs::Marker);

	//set normal parameters
	cylinder->header.frame_id = frame;
	cylinder->header.stamp = ros::Time::now();
	cylinder->header.seq = 0;
	cylinder->ns = ns;
	cylinder->id = id;
	cylinder->type = visualization_msgs::Marker::CYLINDER;
	cylinder->action = visualization_msgs::Marker::ADD;

	//set pose inline with center axis and convert to quaternion
	cylinder->pose.position.x = cylinderCoeffs[0];
	cylinder->pose.position.y = cylinderCoeffs[1];
	cylinder->pose.position.z = cylinderCoeffs[2];

	//convert coeffs to quaternions
	Eigen::Quaternionf orientation;
	orientation.setFromTwoVectors(Eigen::Vector3f::Zero(), centerAxis);

	cylinder->pose.orientation.x = orientation.x();
	cylinder->pose.orientation.y = orientation.y();
	cylinder->pose.orientation.z = orientation.z();
	cylinder->pose.orientation.w = orientation.w();

	//scale coefficients (diameter, direction, height)
	cylinder->scale.x = 2*cylinderCoeffs[6];
	cylinder->scale.y = 2*cylinderCoeffs[6];
	cylinder->scale.z = bounds(2);

	//set normal colors
	cylinder->color.a = color(0);
	cylinder->color.r = color(1);
	cylinder->color.g = color(2);
	cylinder->color.b = color(3);

	return cylinder;
}

//Displays cloud and normals on PCL Visualizer
void pclvizNormals(
					int& pcl_var,
					pcl::visualization::PCLVisualizer& viewer, 
					const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
					const pcl::PointCloud<pcl::Normal>::Ptr& normals
				  ) {
  	viewer.setBackgroundColor(0, 0, 0); //black
  	viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud" + std::to_string(pcl_var));
  	//PCL_VISUALIZER_POINT_SIZE is int starting from 1
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud" + std::to_string(pcl_var));
  	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, .1, "normals" + std::to_string(pcl_var));
  	viewer.addCoordinateSystem(1.0);
  	viewer.initCameraParameters();
  	viewer.spinOnce();
  	pcl_var++;
}

#endif