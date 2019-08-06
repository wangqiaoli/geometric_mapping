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

		double getTimer();
		std::string getBaseFrame();
		std::string getInputFrame();
		int getWindowSize();
		Eigen::Array3f getBoxFilterBounds();
		Eigen::Array3f getSegmentBounds();
		double getLeafSize();
		double getrvizNormalSize();
		int getrvizNormalFrequency();
		double getNeighborRadius(); 
		double getWeightingFactor();

		double getNormalDistWeight(); 
  		int getMaxIterations(); 
  		double getDistThreshold(); 
  		Eigen::Array2f getRadiusLimits();
		
		bool displayDebugger();
		bool displayClouds();
		bool displayNormals();
		bool displayCenterAxis();
		bool displayCylinder();
		bool displaySegments();
		bool displayMap();

		bool usePCLViz();

	private:
		double timer = 5.0;
		std::string baseFrame = "world";
		std::string inputFrame = "world";
		int windowSize = 10;
		boost::shared_ptr<Eigen::Array3f> boxFilterBounds; //make unique
		boost::shared_ptr<Eigen::Array3f> segmentBounds; //make unique
		double leafSize = .1;
		double rvizNormalSize = 0.5;
		int rvizNormalFrequency = 50;
		double neighborRadius = .03;
		double weightingFactor = .2;

		double normalDistWeight = 0.1;
  		int maxIterations = 10000;
  		double distThreshold = 0.05;
  		boost::shared_ptr<Eigen::Array2f> radiusLimits;

  		bool rvizDebugger = true;
		bool rvizClouds = true;
		bool rvizNormals = true;
		bool rvizCenterAxis = true;
		bool rvizCylinder = true;
		bool rvizSegments = true;
		bool rvizMap = true;

		bool pclviz = false; 
};

#endif