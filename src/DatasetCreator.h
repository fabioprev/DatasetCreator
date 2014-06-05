#pragma once

#include <opencv2/core/core.hpp>
#include <iostream>

#define ERR(x)  std::cerr << "\033[22;31;1m" << x << "\033[0m";
#define INFO(x) std::cerr << "\033[22;37;1m" << x << "\033[0m";

class DatasetCreator
{
	private:
		std::vector<std::pair<int,std::vector<cv::Point2f> > > allTrajectories;
		std::vector<cv::Point2f> currentTrajectory;
		std::map<int,std::pair<int,std::pair<int,int> > > trackColorMap;
		cv::Mat currentTrajectoryImage, datasetImage, image;
		unsigned int maximumTrajectoryPoints;
		int currentTrackIndex;
		bool recording;
		
		static void mouseCallback(int event, int x, int y, int, void* datasetCreatorInstance);
		
		void doMouseCallback(int event, int x, int y);
		void generateXmlDataset();
		
	public:
		DatasetCreator(const std::string& imagePath);
		
		~DatasetCreator();
		
		void exec();
};
