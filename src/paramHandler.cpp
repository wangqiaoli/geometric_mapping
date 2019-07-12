#ifndef PARAMHANDLER_CPP
#define PARAMHANDLER_CPP

//Standard libs
#include <cstdlib>

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Eigen libraries
#include <Eigen/Core>

//.h File
#include "paramHandler.hpp"

//Deals with the parameter data
Parameters::Parameters(ros::NodeHandle& node) {
	if(node.getParam("windowSize", windowSize)) {
		ROS_INFO("windowSize set to:\t %f", windowSize);
	} else {
		ROS_INFO("ERROR: windowSize set to default...");
	}

	std::string bounds;
	Eigen::Array3f boundsVector;
	if(node.getParam("boxFilterBound", bounds)) {
		char* cbounds = new char[bounds.length()+1];
		std::strcpy(cbounds, bounds.c_str());
		char* pend;
		boundsVector = Eigen::Array3f(std::strtof(cbounds, &pend), std::strtof(pend, &pend), std::strtof(pend, nullptr));

		ROS_INFO("boxFilterBound set to:\t %f, %f, %f", boundsVector(0), boundsVector(1), boundsVector(2));
	} else {
		boundsVector = Eigen::Array3f(2.0, 5.0, 10.0);

		ROS_INFO("ERROR: boxFilterBound set to default...");
	}

	boxFilterBounds = new Eigen::Array3f(boundsVector);

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

int Parameters::getWindowSize() {
	return windowSize;
}

Eigen::Array3f* Parameters::getBoxFilterBounds() {
	return boxFilterBounds;
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