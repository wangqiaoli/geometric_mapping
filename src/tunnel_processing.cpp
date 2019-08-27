#ifndef TUNNEL_PROCESSING_CPP
#define TUNNEL_PROCESSING_CPP

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
//Declare Chrono Timing Functions
////////////////////////////////////////////////////////



////////////////////////////////////////////////////////
//Declare Point Cloud Processing Functions
////////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilterCloud(
														const double& leafSize,
														const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
											   		) {
	//Implement VoxelGrid Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxelFiltered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> voxelGridFilter; //create voxelgrid object
	voxelGridFilter.setInputCloud(cloud);
	voxelGridFilter.setLeafSize(leafSize, leafSize, leafSize);
	voxelGridFilter.filter(*cloudVoxelFiltered);

	ROS_INFO("VoxelGrid filter applied...");

	return cloudVoxelFiltered;
}

//Creates Registered cloud from time series data
pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloudUpdate(
															// visualization_msgs::MarkerArray*& debugMarkers,
															boost::shared_ptr<Window>& window,
															const tf2_ros::Buffer& tfBuffer,
															const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
															const std::string& baseFrame
														  ) {
	ROS_INFO("Creating registered cloud...");

	//retrieve the broadcasted transform to the world frame
	geometry_msgs::TransformStamped transformStamped;
    try {
      	transformStamped = tfBuffer.lookupTransform(
      													baseFrame, 
      													cloud->header.frame_id.substr(1), 
      													pcl_conversions::fromPCL(cloud->header.stamp)
      												);
    } catch(tf2::TransformException &ex) {
      	ROS_INFO("%s",ex.what());
      	ros::Duration(1.0).sleep();
      	return cloud;
    }

    //transform to the world frame
    auto transfPos = boost::make_shared<Eigen::Vector3d>();
    tf::vectorMsgToEigen(transformStamped.transform.translation, *transfPos);

    auto transfQuat = boost::make_shared<Eigen::Quaterniond>();
    tf::quaternionMsgToEigen(transformStamped.transform.rotation, *transfQuat);

    ROS_INFO_STREAM("Cloud frame is:\t" << cloud->header.frame_id);
    std::cout << "Transform pose is:\n" 
              << transfPos->transpose() 
              << "\nTransform rotation is:\n" 
              << transfQuat->w() 
              << " " << transfQuat->vec().transpose()
              << std::endl;

    //Create affine transformation from start (previous) frame to end (current) frame
	Eigen::Affine3f affine = Eigen::Affine3f::Identity();
	affine.translation() << transfPos->cast<float>()[0], transfPos->cast<float>()[1], transfPos->cast<float>()[2];
	affine.rotate(transfQuat->cast<float>().toRotationMatrix());

	std::cout << "Transform is:\n" << affine.matrix() << std::endl;

	//Apply Transformation to the world frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *registeredCloud, affine);
	registeredCloud->header.frame_id = "/" + baseFrame; //update frame ID

	ROS_INFO("Affine Transformation applied...");

	//pop oldest cloud if window is full
	while(window->cloudWindow.size() >= window->size) {
		window->cloudWindow.pop_back();
	}

	//add cloud to window and 
	window->cloudWindow.push_front(*registeredCloud);

	ROS_INFO_STREAM("Registered cloud frame is:\t" << registeredCloud->header.frame_id);

	//set loop for either window.size iterations or for number of iterations passed since start
	int slidingWindowIts = window->size;
	if(window->cloudWindow.size() < slidingWindowIts) {
		slidingWindowIts = window->cloudWindow.size();
		ROS_INFO("Window will be %d timesteps...", (int) window->cloudWindow.size());
	}

	//Use odometry to convert previous cloud to it's position in the world frame
	// Eigen::Vector3d endPos;
 //    tf::pointMsgToEigen(window.odometryWindow[0].pose.pose.position, endPos);

 //    Eigen::Quaterniond endQuat;
 //    tf::quaternionMsgToEigen(window.odometryWindow[0].pose.pose.orientation, endQuat);

	//concatinates current cloud and all previous clouds in world frame
	for(int i = 1; i < slidingWindowIts; i++) {
		// //Get start pose
		// Eigen::Vector3d startPos;
	 //    tf::pointMsgToEigen(window.odometryWindow[0].pose.pose.position, startPos);

	 //    Eigen::Quaterniond startQuat;
	 //    tf::quaternionMsgToEigen(window.odometryWindow[0].pose.pose.orientation, startQuat);

	 //    //Find difference vector and quaternion for coordinate transformation
		// Eigen::Vector3d diffPos = startPos - endPos; 
	 //    Eigen::Quaterniond diffQuat = endQuat.inverse() * startQuat;

		// //create affine transformation to the pose from the world frame
		// Eigen::Affine3d affine = Eigen::Affine3d::Identity();
		// affine.translation() << diffPos[0], diffPos[1], diffPos[2];
		// affine.rotate(diffQuat.toRotationMatrix());

		// pcl::PointCloud<pcl::PointXYZ> transformedCloud;
		// pcl::transformPointCloud(window.cloudWindow[i], transformedCloud, affine);

		//concatinate cloud into registered cloud sliding window
		*registeredCloud += window->cloudWindow[i];

		ROS_INFO("Previous cloud #%d added to registered cloud...", i);
	}

	//publish transform for local world frame so rviz shows local frame


	return registeredCloud;
}

//Chops point cloud at each timestep
pcl::PointCloud<pcl::PointXYZ>::Ptr chopCloud(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const Window& window,
												const tf2_ros::Buffer& tfBuffer,
												const Eigen::Vector3f& bounds,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												const std::string& baseFrame,
												const std::string& boxFrame,
												const boost::optional<Eigen::Vector3f>& offset,
												const boost::optional<Eigen::Affine3d>& transform
											 ) {
	//Apply Box Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudChopped(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(-bounds(0), -bounds(1), -bounds(2), 1.0));
	boxFilter.setMax(Eigen::Vector4f(bounds(0), bounds(1), bounds(2), 1.0));

	if(window.isRegistered) { //transforms box to velodyne
		//declare transform
		auto transfPos = boost::make_shared<Eigen::Vector3d>();
		auto transfQuat = boost::make_shared<Eigen::Quaterniond>();

		//if transform is passed use it
		if(transform) {
			*transfPos = transform.get().translation();
			*transfQuat = Eigen::Quaterniond(transform.get().rotation());
		} else {
			//retrieve the broadcasted transform to the world frame
			geometry_msgs::TransformStamped transformStamped;
		    try {
		      	transformStamped = tfBuffer.lookupTransform(
		      													cloud->header.frame_id.substr(1),
		      													boxFrame, 
		      													pcl_conversions::fromPCL(cloud->header.stamp)
		      												);
		    } catch(tf2::TransformException &ex) {
		      	ROS_INFO("%s",ex.what());
		      	ros::Duration(1.0).sleep();
		    }

		    //transform to velodyne frame
		    tf::vectorMsgToEigen(transformStamped.transform.translation, *transfPos);
		    tf::quaternionMsgToEigen(transformStamped.transform.rotation, *transfQuat);
		}

	   	//sets offset to center of segment along center axis
	    if(offset) {
	    	*transfPos += offset.get().cast<double>();
	    }

	    boxFilter.setTranslation(transfPos->cast<float>());
	    boxFilter.setRotation(transfQuat->cast<float>().toRotationMatrix().eulerAngles(0, 1, 2));
	}

	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cloudChopped);

	ROS_INFO("Box filter applied...");

	return cloudChopped;
}

//Calculates and returns surface normals of point cloud and removes NAN points from the cloud
pcl::PointCloud<pcl::Normal>::Ptr getNormals(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const double& neighborRadius, 
												pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud
											) {
	//Create the normal estimation class
  	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> findNormals;
  	findNormals.setInputCloud(cloud);

  	//create KD tree of the point cloud
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  	findNormals.setSearchMethod(tree);

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

  	return cloud_normals;
}

//Calculates center axis of the tunnel using the eigenvalues and eigenvectors of the normal cloud
void getLocalFrame(
					// visualization_msgs::MarkerArray*& debugMarkers,
					const Window& window,
					boost::shared_ptr<tf2_ros::TransformBroadcaster>& tfBroadcaster,
					const int& cloudSize, 
					const double& weightingFactor, //on [.1, .3]
					const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
					boost::shared_ptr<Eigen::Vector3f>& eigenVals,
					boost::shared_ptr<Eigen::Matrix3f>& eigenVecs,
					const std::string& baseFrame,
					const std::string& eigenFrame
				  ) {
	//initialize weight matrix with curvatures and weight factor
	Eigen::MatrixXf weights = Eigen::MatrixXf::Zero(cloudSize, cloudSize); //(nxn)

	ROS_INFO("Weights made of size:\t %d", (int) cloudSize);

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

	auto eigenValues = boost::make_shared<Eigen::Vector3f>(eigenSolver.eigenvalues()); //(3x1)
	std::cout << "Eigenvalues are:\n" << *eigenValues << std::endl;

	auto eigenVectors = boost::make_shared<Eigen::Matrix3f>(eigenSolver.eigenvectors()); //(3x3)
	std::cout << "Eigenvectors are:\n" << *eigenVectors << std::endl;

	// //use minimum eigenvector as center axis and display on rviz
	// Eigen::Vector3f* centerAxis(new Eigen::Vector3f);
	// *centerAxis = eigenVectors->block<3,1>(0,0); // first eigenvec is always min

	std::cout << "Center Axis is:\n" << eigenVectors->block<3,1>(0,0).transpose() << std::endl;

	//Get current pose from odometry
	Eigen::Vector3d curPos;
    tf::pointMsgToEigen(window.odometryWindow[0].pose.pose.position, curPos);

    Eigen::Quaterniond curQuat;
    tf::quaternionMsgToEigen(window.odometryWindow[0].pose.pose.orientation, curQuat);

    //Create difference between init and current
    Eigen::Vector3d diffPos = window.initPos.cast<double>() - curPos;
    Eigen::Quaterniond diffQuat = curQuat.inverse() * window.initQuat.cast<double>();

	//create local coordinate frame in tf
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.frame_id = baseFrame;
	transformStamped.child_frame_id = eigenFrame;
	transformStamped.transform.translation.x = diffPos(0);
	transformStamped.transform.translation.y = diffPos(1);
	transformStamped.transform.translation.z = diffPos(2);
	transformStamped.transform.rotation.x = diffQuat.x();
	transformStamped.transform.rotation.y = diffQuat.y();
	transformStamped.transform.rotation.z = diffQuat.z();
	transformStamped.transform.rotation.w = diffQuat.w();
	transformStamped.header.stamp = window.odometryWindow[0].header.stamp; //ros::Time::now();
	
	tfBroadcaster->sendTransform(transformStamped);

	ROS_INFO_STREAM("TF Transform posted from /" << baseFrame << " to /" << eigenFrame << "...");

	//return values
	eigenVals = eigenValues;
	eigenVecs = eigenVectors;
}

//segments pointcloud according to local axis
std::deque<CloudSegment> segmentCloud(
										const Window& window, 
										const tf2_ros::Buffer& tfBuffer,
										const Eigen::Array3f& cloudBounds,
										const Eigen::Array3f& segBounds, 
										const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
										const pcl::PointCloud<pcl::Normal>::Ptr& normals,
										const Eigen::Vector3f& centerAxis,
										const std::string& baseFrame,
										const std::string& eigenFrame
									  ) {
	//declare temp cloud and normals
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
	pcl::PointCloud<pcl::Normal>::Ptr normalTemp(new pcl::PointCloud<pcl::Normal>(*normals));

	//declare vector of segments
	std::deque<CloudSegment> segments;

	//get bounds vectors
	Eigen::Vector3f cloudBoundVec(cloudBounds);
	Eigen::Vector3f segBoundVec(segBounds);

	//get eigenbasis transform for box filter
	geometry_msgs::TransformStamped transformStamped;
	try {
		  transformStamped = tfBuffer.lookupTransform(
		      											baseFrame,
		      											eigenFrame, 
		      											pcl_conversions::fromPCL(cloud->header.stamp)
		      										 );
	} catch(tf2::TransformException &ex) {
		   ROS_INFO("%s",ex.what());
		   ros::Duration(1.0).sleep();
	}

	//Convert to eigen
	Eigen::Vector3d transfPos;
	tf::vectorMsgToEigen(transformStamped.transform.translation, transfPos);

	Eigen::Quaterniond transfQuat;
	tf::quaternionMsgToEigen(transformStamped.transform.rotation, transfQuat);

	//get affine transformation to eigen frame
	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	transform.translation() << transfPos[0], transfPos[1], transfPos[2];
	transform.rotate(transfQuat.toRotationMatrix());

	//loop over cloud until segmented
	double len = -cloudBoundVec(0); //meters in x axis in local frame
	bool outOfTheLoop = false;
	bool cloudSegmented = false;
	while(cloudSegmented) {
		//declare cloud segments
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSeg(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr normalSeg(new pcl::PointCloud<pcl::Normal>);

		if(outOfTheLoop) {
			//set segments equal to remaining cloud
			*cloudSeg = *cloudTemp;
			*normalSeg = *normalTemp;

			//change segBoundVec for last segment to remaining length
			segBoundVec(0) = (cloudBoundVec(0) - len) / 2;
		}

		//Get center offset of segment
		Eigen::Vector3f center = transform.cast<float>() * Eigen::Vector3f((len + segBoundVec(0)), 0, 0);

		//box filter segment along local axis
		cloudSeg = chopCloud(
								window,
								tfBuffer,
								segBoundVec,
								cloud,
								baseFrame,
								eigenFrame,
								center, //box filter around center of segment
								transform
							);

		//Get indices of segment
		pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
  		extractIndices.setInputCloud(cloudSeg);
  		extractIndices.setIndices(pcl::PointIndices::Ptr()); //empty indices list
  		extractIndices.setNegative(true); //set all points to be removed

  		auto segIndices = extractIndices.getRemovedIndices(); //extract all indices

  		if(!outOfTheLoop) {
	  		//remove indices from seg cloud
	  		pcl::ExtractIndices<pcl::PointXYZ> removeCloudSegment;
	  		removeCloudSegment.setInputCloud(cloudTemp);
	  		removeCloudSegment.setIndices(segIndices);
	  		removeCloudSegment.filter(*cloudTemp);
	  	}

  		//extract corresponding normal segment (assumes indices of cloud and normals are the same)
  		pcl::ExtractIndices<pcl::Normal> extractNormalSegment;
  		extractNormalSegment.setInputCloud(normalTemp);
  		extractNormalSegment.setIndices(segIndices);
  		extractNormalSegment.setNegative(true); //set all non-seg normals to be removed
  		extractNormalSegment.filter(*normalSeg);

  		if(!outOfTheLoop) {
			//extract indices from normal seg cloud
			pcl::ExtractIndices<pcl::Normal> removeNormalSegment;
	  		removeNormalSegment.setInputCloud(normalTemp);
	  		removeNormalSegment.setIndices(segIndices);
	  		removeNormalSegment.filter(*normalTemp);
	  	}

		//Make cloud segment object and push to vector
		CloudSegment segmentTemp; //object of pointers to dynamic memory
		segmentTemp.position = boost::make_shared<Eigen::Vector3f>(transfPos.cast<float>() + center); //center of segment
		segmentTemp.orientation = boost::make_shared<Eigen::Quaternionf>(transfQuat.cast<float>());
		segmentTemp.cloudSeg = cloudSeg;
		segmentTemp.normalSeg = normalSeg;

		segments.push_front(segmentTemp);

		//activated in last loop
		if(outOfTheLoop) {
			cloudSegmented = true;
		}

		//set box filter bounds for next segment
		if((cloudBounds(0) - len) < 2*segBoundVec(0)) { //if at end of cloud but 2 lens away
			outOfTheLoop = true;
		}

		len += 2 * segBoundVec(0);
	}

	return segments;
}

//Regression function using RANSAC
boost::shared_ptr<Eigen::VectorXf> getCylinder(
												// visualization_msgs::MarkerArray*& debugMarkers,
												const double& distThreshold,
												const Eigen::Vector3f& centerAxis,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
												const pcl::PointCloud<pcl::Normal>::Ptr& normals
											  ) {
	//VoxelGrid for efficiency????????????????

	//Create segmentation object
	pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>::Ptr model(new pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>(cloud));
	model->setInputNormals(normals);
	model->setAxis(centerAxis);

	// Create the segmentation object for cylinder segmentation and set all the parameters
	pcl::RandomSampleConsensus<pcl::PointXYZ> RANSAC(model);
	RANSAC.setDistanceThreshold(distThreshold);

	// Obtain the cylinder coefficients
	auto cylinderCoeffs = boost::make_shared<Eigen::VectorXf>();
	RANSAC.getModelCoefficients(*cylinderCoeffs);

	return cylinderCoeffs;
}

//Loops through segments and finds regression for each
std::deque<MapSegment> mapCloud(
									const double& distThreshold,
									const Eigen::Vector3f& centerAxis,
									const std::deque<CloudSegment>& segments
								) {
	//initialize map
	std::deque<MapSegment> map;

	//loop through segments to find regression and set coeffs to map
	for(int i = 0; i < segments.size(); i++) {
		//get cylinder model using RANSAC
		auto cylinderModel = getCylinder(
											distThreshold,
											centerAxis,
											segments.at(i).cloudSeg,
											segments.at(i).normalSeg
										);

		ROS_INFO("Local Cylinder Model #%d found...", i);

		//initialise MapSegment object
		MapSegment mapSegTemp;
		mapSegTemp.position = segments.at(i).position;
		mapSegTemp.orientation = segments.at(i).orientation;
		mapSegTemp.cylinderCoeffs = cylinderModel;

		//push object onto map
		map.push_front(mapSegTemp);
	}

	return map;
}

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
															const int& id,
															const std::string& frame
									 					) {
	//declare marker
	auto arrow = boost::make_shared<visualization_msgs::Marker>();

	//set normal parameters
	arrow->header.frame_id = frame;
	arrow->header.stamp = ros::Time::now();
	arrow->header.seq = 0;
	arrow->ns = ns;
	arrow->id = id;
	arrow->type = visualization_msgs::Marker::ARROW;
	arrow->action = visualization_msgs::Marker::ADD;

	//set start [0] (zero coords velodyne frame) and end [1] points of arrow
	arrow->points.resize(2);

	arrow->points[0].x = start(0);
	arrow->points[0].y = start(1);
	arrow->points[0].z = start(2);

	arrow->points[1].x = end(0);
	arrow->points[1].y = end(1);
	arrow->points[1].z = end(2);

	//set normal scales
	arrow->scale.x = scale(0);
	arrow->scale.y = scale(1);
	arrow->scale.z = scale(2);

	//set normal colors
	arrow->color.a = color(0);
	arrow->color.r = color(1);
	arrow->color.g = color(2);
	arrow->color.b = color(3);

	return arrow;
}

//Displays single plane in rviz
boost::shared_ptr<visualization_msgs::Marker> rvizCube(
														const Eigen::Vector3f& position,
														const Eigen::Quaternionf& orientation,
														const Eigen::Vector3f& scale, 
														const Eigen::Vector4f& color,
														const std::string& ns,
														const int& id,
														const std::string& frame
									 				  ) {
	//declare marker
	auto plane = boost::make_shared<visualization_msgs::Marker>();

	//set normal parameters
	plane->header.frame_id = frame;
	plane->header.stamp = ros::Time::now();
	plane->header.seq = 0;
	plane->ns = ns;
	plane->id = id;
	plane->type = visualization_msgs::Marker::CUBE; //make very thin
	plane->action = visualization_msgs::Marker::ADD;

	//set pose
	plane->pose.position.x = position(0);
	plane->pose.position.y = position(1);
	plane->pose.position.z = position(2);

	plane->pose.orientation.x = orientation.x();
	plane->pose.orientation.y = orientation.y();
	plane->pose.orientation.z = orientation.z();
	plane->pose.orientation.w = orientation.w();

	//set normal scales
	plane->scale.x = scale(0); //very small for plane
	plane->scale.y = scale(1);
	plane->scale.z = scale(2);

	//set normal colors
	plane->color.a = color(0);
	plane->color.r = color(1);
	plane->color.g = color(2);
	plane->color.b = color(3);

	return plane;
}

//Displays surface normals in rviz using VoxelGrid Filter and KD tree to free computing power
boost::shared_ptr<visualization_msgs::MarkerArray> rvizNormals(
																const double& normalSize,
																const int& normalFrequency,
																const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
																const pcl::PointCloud<pcl::Normal>::Ptr& normals
															  ) {
	//Add normals of the nearest neighbor cloud to MarkerArray
	auto normalVecs = boost::make_shared<visualization_msgs::MarkerArray>();
	normalVecs->markers.resize(cloud->points.size());

	std::cout << "Size\t" << cloud->points.size() << std::endl;
	std::cout << "SizeN\t" << normals->points.size() << std::endl;

	Eigen::Vector3f startVec = Eigen::Vector3f::Zero();
	Eigen::Vector3f endVec = Eigen::Vector3f::Zero();
	Eigen::Vector3f scaleVec(0.025, 0.075, 0.0625);
	Eigen::Vector4f colorVec(1, 0, 0, 1);

	for(int i = 0; i < cloud->points.size(); i++) {
		if (i > normals->points.size()) {
			break;
		}

		//init start point in eigen
		startVec(0) = cloud->points[i].x;
		startVec(1) = cloud->points[i].y;
		startVec(2) = cloud->points[i].z;

		//init endpoint in eigen
		int index = (1 + i) * normalFrequency; //normals should correspond to points
		endVec(0) = cloud->points[i].x + normalSize * normals->at(index).normal[0];
		endVec(1) = cloud->points[i].y + normalSize * normals->at(index).normal[1];
		endVec(2) = cloud->points[i].z + normalSize * normals->at(index).normal[2];

		normalVecs->markers[i] = *rvizArrow(startVec, endVec, scaleVec, colorVec, "normals", i);
	}

	ROS_INFO("Normals posted to rviz...");

	return normalVecs;
}

//Displays eigenspace in rviz
boost::shared_ptr<visualization_msgs::MarkerArray> rvizEigens(
																const boost::shared_ptr<Eigen::Vector3f>& eigenVals,
																const boost::shared_ptr<Eigen::Matrix3f>& eigenVecs
															 ) {
	auto eigenBasis = boost::shared_ptr<visualization_msgs::MarkerArray>();
	eigenBasis->markers.resize(3); //for E1E2E3 basis

	//normalize eigenvals and use to set legnth of arrow basis
	Eigen::Vector3f eigenValNorms = (1/eigenVals->norm()) * eigenVals->cwiseAbs();

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
												(1 + .5*eigenValNorms(i)) * eigenVecs->block<3,1>(0,i), //vecs by col
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
boost::shared_ptr<visualization_msgs::Marker> rvizCylinder(
															const Eigen::Array3f& segBounds,
															const Eigen::VectorXf& cylinderCoeffs,
															const Eigen::Vector3f& centerAxis, 
															const Eigen::Vector4f& color,
															const std::string& ns,
															const int& id,
															const std::string& frame
														  ) {
	//declare marker
	auto cylinder = boost::make_shared<visualization_msgs::Marker>();

	//set normal parameters
	cylinder->header.frame_id = frame;
	cylinder->header.stamp = ros::Time::now();
	cylinder->header.seq = 0;
	cylinder->ns = ns;
	cylinder->id = id;
	cylinder->type = visualization_msgs::Marker::CYLINDER;
	cylinder->action = visualization_msgs::Marker::ADD;

	//set pose inline with center axis and convert to quaternion
	cylinder->pose.position.x = -5;//cylinderCoeffs[0];
	cylinder->pose.position.y = 0;//cylinderCoeffs[1];
	cylinder->pose.position.z = 0;//cylinderCoeffs[2];

	//convert coeffs to quaternions
	Eigen::Quaternionf orientation;
	orientation.setFromTwoVectors(Eigen::Vector3f::Zero(), centerAxis);

	cylinder->pose.orientation.x = 0;//orientation.x();
	cylinder->pose.orientation.y = .7071;//orientation.y();
	cylinder->pose.orientation.z = 0;//orientation.z();
	cylinder->pose.orientation.w = .7071;//orientation.w();

	//scale coefficients (diameter, direction, height)
	cylinder->scale.x = 3;//2*cylinderCoeffs[6];
	cylinder->scale.y = 3;//2*cylinderCoeffs[6];

	Eigen::Vector3f boundVec(segBounds);
	cylinder->scale.z = 5;//boundVec(2);

	//set normal colors
	cylinder->color.a = color(0);
	cylinder->color.r = color(1);
	cylinder->color.g = color(2);
	cylinder->color.b = color(3);

	return cylinder;
}

//Displays Segments as planes on boundaries
boost::shared_ptr<visualization_msgs::MarkerArray> rvizSegments(
																	const Eigen::Array3f& segBounds,
																	const std::deque<CloudSegment>& segments,
																	const Eigen::Vector3f& centerAxis,
																	const std::string& ns
											 					) {
	//declare segment barrier marker array
	auto map = boost::make_shared<visualization_msgs::MarkerArray>();
	map->markers.resize(segments.size());

	//create plane markers for each segment barrier
	Eigen::Vector3f segBoundVec(segBounds);
	for(int i = 0; i < segments.size(); i++) {
		map->markers[i] = *rvizCube( 
										*(segments.at(i).position) - Eigen::Vector3f(segBoundVec(0),0,0), //left bound of segment box
										*(segments.at(i).orientation),
										Eigen::Vector3f(.01, segBoundVec(1), segBoundVec(2)), //scale to bounds YZ plane
										Eigen::Vector4f(.6 , 1.0, 1.0, 1.0), //gray
										ns,
										i
									);
	}
}

//Display geometric map
boost::shared_ptr<visualization_msgs::MarkerArray> rvizMap(
															const Eigen::Array3f& segBounds,
															const std::deque<MapSegment>& segments,
															const Eigen::Vector3f& centerAxis,
															const std::string& ns
										 				  ) {
	//declare cylinder marker array
	auto map = boost::make_shared<visualization_msgs::MarkerArray>();
	map->markers.resize(segments.size());

	//create cylinder markers for each segment
	for(int i = 0; i < segments.size(); i++) {
		map->markers[i] = *rvizCylinder(
											segBounds,
											*(segments.at(i).cylinderCoeffs),
											centerAxis, 
											Eigen::Vector4f(0.6, 1.0, .65, 0.0), //orange
											ns,
											i
										);
	}

	return map;
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