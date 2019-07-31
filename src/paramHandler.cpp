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
	if(node.getParam("baseFrame", baseFrame)) {
		ROS_INFO("baseFrame set to:\t %s",baseFrame.c_str());
	} else {
		ROS_INFO("ERROR: baseFrame set to default...");
	}

	if(node.getParam("inputFrame", inputFrame)) {
		ROS_INFO("inputFrame set to:\t %s",inputFrame.c_str());
	} else {
		ROS_INFO("ERROR: inputFrame set to default...");
	}

	if(node.getParam("windowSize", windowSize)) {
		ROS_INFO("windowSize set to:\t %d", windowSize);
	} else {
		ROS_INFO("ERROR: windowSize set to default...");
	}

	std::string bounds;
	Eigen::Array3f boundsVector;
	if(node.getParam("boxFilterBoundXYZ", bounds)) {
		char* cbounds = new char[bounds.length()+1];
		std::strcpy(cbounds, bounds.c_str());
		char* pend;

		float x = std::strtof(cbounds, &pend) / 2; //divide by 2 so that it will be more accurate
		float y = std::strtof(pend, &pend) / 2;
		float z = std::strtof(pend, nullptr) / 2;
		boundsVector = Eigen::Array3f(x, y, z);

		ROS_INFO("boxFilterBound set to:\t %f, %f, %f", x, y, z);
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

	if(node.getParam("normalDistWeight", normalDistWeight)) {
		ROS_INFO("normalDistWeight set to:\t %f", normalDistWeight);
	} else {
		ROS_INFO("ERROR: normalDistWeight set to default...");
	}

	if(node.getParam("maxIterations", maxIterations)) {
		ROS_INFO("maxIterations set to:\t %d", maxIterations);
	} else {
		ROS_INFO("ERROR: maxIterations set to default...");
	}

	if(node.getParam("distThreshold", distThreshold)) {
		ROS_INFO("distThreshold set to:\t %f", distThreshold);
	} else {
		ROS_INFO("ERROR: distThreshold set to default...");
	}

	std::string radiuslimits;
	Eigen::Array2f radiusLimitsVector;
	if(node.getParam("radiusLimits", radiuslimits)) {
		char* climits = new char[radiuslimits.length()+1];
		std::strcpy(climits, radiuslimits.c_str());
		char* pend;

		float lb = std::strtof(climits, &pend);
		float ub = std::strtof(pend, nullptr);
		radiusLimitsVector = Eigen::Array2f(lb, ub);

		ROS_INFO("radiusLimits set to:\t %f, %f", lb, ub);
	} else {
		radiusLimitsVector = Eigen::Array2f(0.0, 15.0);

		ROS_INFO("ERROR: radiusLimits set to default...");
	}

	radiusLimits = new Eigen::Array2f(radiusLimitsVector);

	if(node.getParam("displayDebugger", rvizDebugger)) {
		ROS_INFO("displayDebugger set to:\t %d", rvizDebugger);
	} else {
		ROS_INFO("ERROR: displayDebugger set to default...");
	}

	if(node.getParam("displayClouds", rvizClouds)) {
		ROS_INFO("displayClouds set to:\t %d", rvizClouds);
	} else {
		ROS_INFO("ERROR: displayClouds set to default...");
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

std::string Parameters::getBaseFrame() {
	return baseFrame;
}

std::string Parameters::getInputFrame() {
	return inputFrame;
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

double Parameters::getNormalDistWeight() {
	return normalDistWeight;
} 

int Parameters::getMaxIterations() {
	return maxIterations;
} 

double Parameters::getDistThreshold() {
	return distThreshold;
} 

Eigen::Array2f* Parameters::getRadiusLimits() {
	return radiusLimits;
}

bool Parameters::displayDebugger() {
	return rvizDebugger;
}

bool Parameters::displayClouds() {
	return rvizClouds;
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