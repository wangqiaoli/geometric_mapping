#ifndef PARAMHANDLER_HPP
#define PARAMHANDLER_HPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Parameter Handler Class
class Parameters {
	public:
		Parameters(ros::NodeHandle& node);
		bool getApplyBoxFilter();
		double getBoxFilterBound(); 
		bool getApplyVoxelGridFilter();
		double getLeafSize();
		// bool getPCLViz();
		bool getFindSurfaceNormals();
		double getNeighborRadius(); 
		bool getFindCenterAxis();

	private:
		bool debug = true;
		bool applyBoxFilter = true;
		double boxFilterBound = 5.0;
		bool applyVoxelGridFilter = true;
		double leafSize = .1;
		// bool pclviz = false;
		bool findSurfaceNormals = true;
		double neighborRadius = .03;
		bool findCenterAxis = true;
};

//MAKE CLASS FOR POINT CLOUD HANDLING

#endif