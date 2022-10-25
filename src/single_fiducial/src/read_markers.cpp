#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "read_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  std::fstream fin;
  try{
  	fin.open("/home/irsyad/vision_fiducial_ws/src/single_fiducial/src/include/markerslist.csv", std::ios::in);
  	}
  catch(char *excp){
  	ROS_INFO("ERROR: ");
  	}
  //<std::vector<std::string>> content;
  std::vector<std::string> row;
  std::string line, word, temp;
  
	while(ros::ok()) 
	{
		while(fin >> temp)
		{
			row.clear();
			getline(fin, line);
			std::stringstream s(line);
			while(getline(s,word,','))
			{
				row.push_back(word);
			}
			//ROS_INFO("%s %s %s %s %s %s %s",row[0].c_str(),row[1].c_str(),row[2].c_str(),row[3].c_str(),row[4].c_str(),row[5].c_str(),row[6].c_str());
		}
		
		ROS_INFO("RUNNING");
		r.sleep();
	}
}
