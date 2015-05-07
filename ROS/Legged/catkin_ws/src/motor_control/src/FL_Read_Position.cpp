#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "motor_control/Position.h"

#include <iostream>

motor_control::Position position_msg;

int main(int argc, char **argv)
{
  int32_t rpos = 0;

  ros::init(argc, argv, "FL_Read_Position");
  ros::NodeHandle n;
  ros::Publisher read_position_pub = n.advertise<motor_control::Position>("FL_Position", 1000);
  ros::Rate loop_rate(1000);
  
  ROS_INFO("Reading Desired Position.");
  while (ros::ok())
  {
	cin >> rpos;
	position_msg.desiredPos = rpos;
	read_position_pub.publish(position_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
