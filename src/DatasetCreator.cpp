#include "DatasetCreator.h"
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <fstream>

using namespace std;
using namespace cv;

DatasetCreator::DatasetCreator(const string& imagePath) : maximumTrajectoryPoints(0), currentTrackIndex(1), recording(false)
{
	Mat commands;
	
	image = imread(imagePath);
	
	if (image.empty())
	{ 
		ERR("Error loading image: " << imagePath << ". Exiting..." << endl);
		
		exit(-1);
	}
	
	currentTrajectoryImage = image.clone();
	datasetImage = image.clone();
	
	namedWindow("Current Trajectory",1);
	namedWindow("Dataset",1);
	namedWindow("Commands",1);
	
	setMouseCallback("Current Trajectory",DatasetCreator::mouseCallback,this);
	
	imshow("Current Trajectory",currentTrajectoryImage);
	imshow("Dataset",datasetImage);
	
	commands = Mat::zeros(135,497,CV_8UC3);
	
	putText(commands,"'e':",Point(10,20),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,0));
	putText(commands,"'n':",Point(10,45),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,0));
	putText(commands,"'q':",Point(10,70),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,0));
	putText(commands,"'r':",Point(10,95),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,0));
	putText(commands,"'s':",Point(10,120),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,0));
	
	putText(commands," stop drawing the current track (resume by pressing 'n')",Point(32,20),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
	putText(commands," start drawing a new track",Point(32,45),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
	putText(commands," quit without saving the non-already saved tracks",Point(32,70),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
	putText(commands," reset the current track",Point(32,95),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
	putText(commands," save the current track",Point(32,120),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
	
	imshow("Commands",commands);
	moveWindow("Commands",0,0);
	
	int counter = 1;
	
	for (int i = 0; i < 256; i += 63)
	{
		for (int j = 0; j < 256; j += 63)
		{
			for (int k = 63; k < 256; k += 63, ++counter)
			{
				trackColorMap.insert(make_pair(counter,make_pair(i,make_pair(j,k))));
			}
		}
	}
}

DatasetCreator::~DatasetCreator() {;}

void DatasetCreator::doMouseCallback(int event, int x, int y)
{
	if ((event == EVENT_MOUSEMOVE) && (recording))
	{
		currentTrajectory.push_back(Point2f(x,y));
		
		map<int,pair<int,pair<int,int> > >::iterator colorTrack = trackColorMap.find(currentTrackIndex);
		
		circle(currentTrajectoryImage,Point(x,y),1,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),2);
		
		imshow("Current Trajectory",currentTrajectoryImage);
	}
}

void DatasetCreator::exec()
{
	char key = '\0';
	
	while (key != 'q')
	{
		key = waitKey(0);
		
		if (key == 'e') recording = false;
		else if (key == 'n') recording = true;
		else if (key == 'r')
		{
			recording = false;
			
			currentTrajectory.clear();
			
			currentTrajectoryImage = image.clone();
			
			imshow("Current Trajectory",currentTrajectoryImage);
		}
		else if (key == 's')
		{
			recording = false;
			
			if (currentTrajectory.empty()) continue;
			
			INFO("Saving trajectory...");
			
			map<int,pair<int,pair<int,int> > >::iterator colorTrack = trackColorMap.find(currentTrackIndex);
			
			for (vector<Point2f>::const_iterator it = currentTrajectory.begin(); it != currentTrajectory.end(); ++it)
			{
				circle(datasetImage,Point(it->x,it->y),1,cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),2);
			}
			
			currentTrajectoryImage = image.clone();
			
			imshow("Current Trajectory",currentTrajectoryImage);
			imshow("Dataset",datasetImage);
			
			allTrajectories.push_back(make_pair(currentTrackIndex,currentTrajectory));
			
			if (currentTrajectory.size() > maximumTrajectoryPoints) maximumTrajectoryPoints = currentTrajectory.size();
			
			currentTrajectory.clear();
			
			++currentTrackIndex;
			
			INFO("done." << endl);
		}
	}
	
	generateXmlDataset();
}

void DatasetCreator::generateXmlDataset()
{
	if (allTrajectories.empty()) return;
	
	pair<int,Point2f> allTrajectoryPoints[maximumTrajectoryPoints][allTrajectories.size()];
	ofstream datasetXml;
	unsigned int i, j;
	
	for (i = 0; i < maximumTrajectoryPoints; ++i)
	{
		for (j = 0; j < allTrajectories.size(); ++j)
		{
			allTrajectoryPoints[i][j] = make_pair(-1,Point2f(-1,-1));
		}
	}
	
	i = 0;
	j = 0;
	
	for (vector<pair<int,vector<Point2f> > >::const_iterator it = allTrajectories.begin(); it != allTrajectories.end(); ++it, ++j)
	{
		i = 0;
		
		for (vector<Point2f>::const_iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2, ++i)
		{
			allTrajectoryPoints[i][j] = make_pair(it->first,*it2);
		}
	}
	
	struct stat temp;
	
	if (stat("../xml",&temp) == -1)
	{
		mkdir("../xml",0700);
	}
	
	datasetXml.open("../xml/dataset.xml");
	
	if (!datasetXml.is_open())
	{
		ERR("A problem occurred when creating the ../xml/dataset.xml file. Exiting...");
		
		return;
	}
	
	INFO(endl << "Saving xml file...");
	
	datasetXml << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << endl;
	datasetXml << "<dataset>" << endl;
	
	for (i = 0; i < maximumTrajectoryPoints; ++i)
	{
		datasetXml << "   <frame number=\"" << i << "\">" << endl;
		datasetXml << "      <objectlist>" << endl;
		
		for (j = 0; j < allTrajectories.size(); ++j)
		{
			if (allTrajectoryPoints[i][j].first != -1)
			{
				datasetXml << "         <object id=\"" << allTrajectoryPoints[i][j].first << "\">" << endl;
				datasetXml << "            <box xc=\"" << allTrajectoryPoints[i][j].second.x << "\" yc=\"" << allTrajectoryPoints[i][j].second.y << "\"/>" << endl;
				datasetXml << "         </object>" << endl;
			}
		}
		
		datasetXml << "      </objectlist>" << endl;
		datasetXml << "   </frame>" << endl;
	}
	
	datasetXml << "</dataset>" << endl;
	
	datasetXml.close();
	
	INFO("..done." << endl);
}

void DatasetCreator::mouseCallback(int event, int x, int y, int, void* datasetCreatorInstance)
{
	DatasetCreator* datasetCreator = static_cast<DatasetCreator*>(datasetCreatorInstance);
	
	datasetCreator->doMouseCallback(event,x,y);
}
