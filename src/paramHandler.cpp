#ifndef PARAMHANDLER_CPP
#define PARAMHANDLER_CPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//.h File
#include "paramHandler.hpp"

//Deals with the parameter data
Parameters::Parameters(ros::NodeHandle& node) {
	if(node.getParam("boxFilterBound", boxFilterBound)) {
		ROS_INFO("boxFilterBound set to:\t %f", boxFilterBound);
	} else {
		ROS_INFO("ERROR: boxFilterBound set to default...");
	}

	if(node.getParam("voxelGridLeafSize", leafSize)) {
		ROS_INFO("leafSize set to:\t %f", leafSize);
	} else {
		ROS_INFO("ERROR: leafSize set to default...");
	}

	if(node.getParam("neighborRadius", neighborRadius)) {
		ROS_INFO("neighborRadius set to:\t %f", neighborRadius);
	} else {
		ROS_INFO("ERROR: neighborRadius set to default...");
	}

	if(node.getParam("weightingFactor", weightingFactor)) {
		ROS_INFO("weightingFactor set to:\t %f", weightingFactor);
	} else {
		ROS_INFO("ERROR: weightingFactor set to default...");
	}

	if(node.getParam("displayCloud", rvizCloud)) {
		ROS_INFO("displayCloud set to:\t %d", rvizCloud);
	} else {
		ROS_INFO("ERROR: displayCloud set to default...");
	}

	if(node.getParam("displayNormals", rvizNormals)) {
		ROS_INFO("displayNormals set to:\t %d", rvizNormals);
	} else {
		ROS_INFO("ERROR: displayNormals set to default...");
	}

	if(node.getParam("displayCenterAxis", rvizCenterAxis)) {
		ROS_INFO("displayCenterAxis set to:\t %d", rvizCenterAxis);
	} else {
		ROS_INFO("ERROR: displayCenterAxis set to default...");
	}

	if(node.getParam("displayCylinder", rvizCylinder)) {
		ROS_INFO("displayCylinder set to:\t %d", rvizCylinder);
	} else {
		ROS_INFO("ERROR: displayCylinder set to default...");
	}

	if(node.getParam("usePCLViz", pclviz)) {
		ROS_INFO("pclviz set to:\t %d", pclviz);
	} else {
		ROS_INFO("ERROR: pclviz set to default...");
	}
}

double Parameters::getBoxFilterBound() {
	return boxFilterBound;
}

double Parameters::getLeafSize() {
	return leafSize;
}

double Parameters::getNeighborRadius() {
	return neighborRadius;
} 

double Parameters::getWeightingFactor() {
	return weightingFactor;
}	

bool Parameters::displayCloud() {
	return rvizCloud;
}

bool Parameters::displayNormals() {
	return rvizNormals;
}

bool Parameters::displayCenterAxis() {
	return rvizCenterAxis;
}

bool Parameters::displayCylinder() {
	return rvizCylinder;
}

bool Parameters::usePCLViz() {
	return pclviz;
}


#endif