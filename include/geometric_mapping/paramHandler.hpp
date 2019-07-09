#ifndef PARAMHANDLER_HPP
#define PARAMHANDLER_HPP

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Parameter Handler Class
class Parameters {
	public:
		Parameters(ros::NodeHandle& node);

		double getBoxFilterBound();
		double getLeafSize();
		double getNeighborRadius(); 
		double getWeightingFactor();
		
		bool displayCloud();
		bool displayNormals();
		bool displayCenterAxis();
		bool displayCylinder();

		bool usePCLViz();

	private:
		double boxFilterBound = 5.0;
		double leafSize = .1;
		double neighborRadius = .03;
		double weightingFactor = .2;

		bool rvizCloud = true;
		bool rvizNormals = true;
		bool rvizCenterAxis = true;
		bool rvizCylinder = true;

		bool pclviz = false;
};

#endif