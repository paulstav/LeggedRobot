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
/*

	ROS node that reads the value of the desired position of the front left (FL) in encoder counts from standard input
	and published the received position in the FL_Position topic.
	
	This node was used for debugging PID controller and should not be run otherwise.

*/

// Global variables
legged_robot::Position position_msg;

int main(int argc, char **argv)
{
  int32_t rpos = 0;
  string inputS;

  // Initialize ROS node
  ros::init(argc, argv, "FL_Read_Position");
  ros::NodeHandle n;
  // Publish for desired position message
  ros::Publisher read_position_pub = n.advertise<legged_robot::Position>("FL_Position", 1000);
  ros::Rate loop_rate(1000);
  
  ROS_INFO("Reading Desired Position.");
  while (ros::ok())
  {
	// Read a line from standard input and parse the desired position
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
			// Publish the received desired position
			position_msg.desiredPos = rpos;
			read_position_pub.publish(position_msg);
		}
	}
	
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
