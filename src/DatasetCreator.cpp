#include "DatasetCreator.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/stat.h>
#include <fstream>

using namespace std;
using namespace cv;

DatasetCreator::DatasetCreator(const string& imagePath) : maximumTrajectoryPoints(0), currentTrackIndex(1), recording(false)
{
	Mat commands;
	int offset;
	
	image = imread(imagePath);
	
	if (image.empty())
	{ 
		ERR("Error loading image: " << imagePath << ". Exiting..." << endl);
		
		exit(-1);
	}
	
	currentTrajectoryImage = image.clone();
	datasetImage = image.clone();
	
	commands = Mat::zeros(135,497,CV_8UC3);
	
	namedWindow("Current Trajectory",1);
	namedWindow("Dataset",1);
	namedWindow("Commands",1);
	
	setMouseCallback("Current Trajectory",DatasetCreator::mouseCallback,this);
	
	offset = 50;
	
	imshow("Current Trajectory",currentTrajectoryImage);
	moveWindow("Current Trajectory",offset,commands.rows * 1.5);
	
	imshow("Dataset",datasetImage);
	moveWindow("Dataset",currentTrajectoryImage.cols + offset + 1,commands.rows * 1.5);
	
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
	
	srand(0);
}

DatasetCreator::~DatasetCreator() {;}

void DatasetCreator::doMouseCallback(int event, int x, int y)
{
	if ((event == EVENT_MOUSEMOVE) && (recording))
	{
		int currentWidth, currentHeight;
		
		currentWidth = width + (-3 + rand() % 7);
		currentHeight = height + (-5 + rand() % 11);
		
		currentTrajectory.push_back(make_pair(Point2f(x,y),Point2i(currentWidth,currentHeight)));
		
		map<int,pair<int,pair<int,int> > >::iterator colorTrack = trackColorMap.find(currentTrackIndex);
		
		rectangle(currentTrajectoryImage,Rect(x - (currentWidth / 2),y - currentHeight,currentWidth,currentHeight),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),1);
		
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
		else if (key == 'n')
		{
			width = 20 + (rand() % 20);
			height = 50 + (rand() % 30);
			
			recording = true;
		}
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
			
			for (vector<pair<Point2f,Point2i> >::const_iterator it = currentTrajectory.begin(); it != currentTrajectory.end(); ++it)
			{
				rectangle(datasetImage,Rect(it->first.x - (it->second.x / 2),it->first.y - it->second.y,it->second.x,it->second.y),cvScalar(colorTrack->second.first,colorTrack->second.second.first,colorTrack->second.second.second),1);
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
	
	pair<int,pair<Point2f,Point2i> > allTrajectoryPoints[maximumTrajectoryPoints][allTrajectories.size()];
	ofstream datasetXml;
	unsigned int i, j;
	
	for (i = 0; i < maximumTrajectoryPoints; ++i)
	{
		for (j = 0; j < allTrajectories.size(); ++j)
		{
			allTrajectoryPoints[i][j] = make_pair(-1,make_pair(Point2f(-1,-1),Point2i(-1,-1)));
		}
	}
	
	i = 0;
	j = 0;
	
	for (vector<pair<int,vector<pair<Point2f,Point2i> > > >::const_iterator it = allTrajectories.begin(); it != allTrajectories.end(); ++it, ++j)
	{
		i = 0;
		
		for (vector<pair<Point2f,Point2i> >::const_iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2, ++i)
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
				datasetXml << "            <box h=\"" << allTrajectoryPoints[i][j].second.second.y << "\" w=\"" << allTrajectoryPoints[i][j].second.second.x << "\""
						   << " xc=\"" << allTrajectoryPoints[i][j].second.first.x << "\" yc=\"" << allTrajectoryPoints[i][j].second.first.y << "\""
						   << " hxc=\"" << allTrajectoryPoints[i][j].second.first.x << "\" hyc=\"" << (allTrajectoryPoints[i][j].second.first.y - allTrajectoryPoints[i][j].second.second.y) << "\""
						   << " b=\"" << allTrajectoryPoints[i][j].second.first.x << "\"/>" << endl;
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
