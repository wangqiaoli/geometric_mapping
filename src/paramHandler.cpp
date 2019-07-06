#ifndef PARAMHANDLER_CPP
#define PARAMHANDLER_CPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//.h File
#include "paramHandler.hpp"

//Deals with the parameter data
Parameters::Parameters(ros::NodeHandle& node) {
	if(node.getParam("applyBoxFilter", applyBoxFilter)) {
		ROS_INFO("applyBoxFilter set to:\t %d", applyBoxFilter);
	} else {
		ROS_INFO("ERROR: applyBoxFilter set to default...");
	}

	if(node.getParam("boxFilterBound", boxFilterBound)) {
		ROS_INFO("boxFilterBound set to:\t %f", boxFilterBound);
	} else {
		ROS_INFO("ERROR: boxFilterBound set to default...");
	}

	if(node.getParam("applyVoxelGridFilter", applyVoxelGridFilter)) {
		ROS_INFO("applyVoxelGridFilter set to:\t %d", applyVoxelGridFilter);
	} else {
		ROS_INFO("ERROR: applyVoxelGridFilter set to default...");
	}

	if(node.getParam("voxelGridLeafSize", leafSize)) {
		ROS_INFO("leafSize set to:\t %f", leafSize);
	} else {
		ROS_INFO("ERROR: leafSize set to default...");
	}

	// if(node.getParam("pclviz", pclviz)) {
	// 	ROS_INFO("pclviz set to:\t %d", pclviz);
	// } else {
	// 	ROS_INFO("ERROR: pclviz set to default...");
	// }

	if(node.getParam("findSurfaceNormals", findSurfaceNormals)) {
		ROS_INFO("findSurfaceNormals set to:\t %d", findSurfaceNormals);
	} else {
		ROS_INFO("ERROR: findSurfaceNormals set to default...");
	}

	if(node.getParam("neighborRadius", neighborRadius)) {
		ROS_INFO("neighborRadius set to:\t %f", neighborRadius);
	} else {
		ROS_INFO("ERROR: neighborRadius set to default...");
	}

	if(node.getParam("findCenterAxis", findCenterAxis)) {
		ROS_INFO("findCenterAxis set to:\t %d", findCenterAxis);
	} else {
		ROS_INFO("ERROR: findCenterAxis set to default...");
	}
}

bool Parameters::getApplyBoxFilter() {
	return applyBoxFilter;
} 

double Parameters::getBoxFilterBound() {
	return boxFilterBound;
}

bool Parameters::getApplyVoxelGridFilter() {
	return applyVoxelGridFilter;
} 

double Parameters::getLeafSize() {
	return leafSize;
}  

// bool getPCLViz() {
// 	return pclviz;
// }

bool Parameters::getFindSurfaceNormals() {
	return findSurfaceNormals;
} 

double Parameters::getNeighborRadius() {
	return neighborRadius;
}

bool Parameters::getFindCenterAxis() {
	return findCenterAxis;
} 

#endif