#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "legged_robot/Position.h"

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

legged_robot::Position position_msg;

int main(int argc, char **argv)
{
  int32_t rpos = 0;
  string inputS;

  ros::init(argc, argv, "FL_Read_Position");
  ros::NodeHandle n;
  ros::Publisher read_position_pub = n.advertise<legged_robot::Position>("FL_Position", 1000);
  ros::Rate loop_rate(1000);
  
  ROS_INFO("Reading Desired Position.");
  while (ros::ok())
  {
	getline (cin,inputS);
	if(inputS == "q")
		break;
	else
	{
		stringstream ss;
		ss<<inputS;
		ss>>rpos; //convert string into int and store it in "asInt"
		ss.str(""); //clear the stringstream
		ss.clear(); 
		
		if( (rpos !=0) || (rpos == 0 && inputS == "0") )
		{
			//cout << "Read : " << rpos << endl;
			position_msg.desiredPos = rpos;
			read_position_pub.publish(position_msg);
		}
	}
	
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
