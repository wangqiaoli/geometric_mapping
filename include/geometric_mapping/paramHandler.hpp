#ifndef PARAMHANDLER_HPP
#define PARAMHANDLER_HPP

//Standard libs
#include <cstdlib>

//Declare ROS c++ library
#include <ros/ros.h>
#include <ros/console.h>

//Eigen libraries
#include <Eigen/Core>

//Parameter Handler Class
class Parameters {
	public:
		Parameters(ros::NodeHandle& node);

		int getWindowSize();
		Eigen::Array3f* getBoxFilterBounds();
		double getLeafSize();
		double getNeighborRadius(); 
		double getWeightingFactor();

		double getNormalDistWeight(); 
  		int getMaxIterations(); 
  		double getDistThreshold(); 
  		Eigen::Array2f* getRadiusLimits();
		
		bool displayCloud();
		bool displayNormals();
		bool displayCenterAxis();
		bool displayCylinder();

		bool usePCLViz();

	private:
		int windowSize = 10;
		Eigen::Array3f* boxFilterBounds;
		double leafSize = .1;
		double neighborRadius = .03;
		double weightingFactor = .2;

		double normalDistWeight = 0.1;
  		int maxIterations = 10000;
  		double distThreshold = 0.05;
  		Eigen::Array2f* radiusLimits;

		bool rvizCloud = true;
		bool rvizNormals = true;
		bool rvizCenterAxis = true;
		bool rvizCylinder = true;

		bool pclviz = false; 
};

#endif