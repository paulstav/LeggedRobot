#include <iostream>
#include <inttypes.h>
#include <armadillo>

/*
#include "ros/ros.h"
#include "legged_robot/RobotInertiaMeasurements.h"

#include <sstream>
*/
/*

	Test node for angle calculations.

*/
/*
// Global variables
int imu_msg_counter = 0;  // IMU message counter variable


// Callback function for reception of inertia raw value message from topic
void imuCallback(const legged_robot::RobotInertiaMeasurements::ConstPtr& msg)
{
	imu_msg_counter++;
}
*/

using namespace std;
using namespace arma;


mat Skew(vec vIn);

// Main Function
int main(int argc, char **argv)
{
  /*
  // Initialize ROS node
  ros::init(argc, argv, "TestArmadillo");
  ros::NodeHandle n;
  // Initialize the subscribers for robot inertia measurements
  ros::Subscriber robot_imu_sub = n.subscribe("Robot_Inertia", 1000, imuCallback);
  ros::Rate loop_rate(12000);
  
  ROS_INFO("Starting armadillo test node.");
  while (ros::ok())
  {
	if(imu_msg_counter > 100)
	{
		imu_msg_counter = 0;
		//ROS_INFO("ACC (mg)   : %lf %lf %lf", rb_imu_msg.accXmg, rb_imu_msg.accYmg, rb_imu_msg.accZmg);
        //ROS_INFO("GYRO (deg) : %lf %lf %lf", rb_imu_msg.gyroXdeg, rb_imu_msg.gyroYdeg, rb_imu_msg.gyroZdeg);
	}
	
    ros::spinOnce();

    loop_rate.sleep();
  }
  */ 
  
  mat A(3,3, fill::zeros);
  vec Vt(3);
  Vt[0] = 1.0;
  Vt[1] = 2.0;
  Vt[2] = 3.0;
  
  
  cout << "Starting test" << endl;
  
  cout << "A" << endl << A << endl;
  
  A = Skew(Vt);
  
  cout << "A" << endl << A << endl;

  return 0;
}

mat Skew(vec vIn)
{
	mat skM(3,3);
	
	skM << 0 << -vIn[2] << vIn[1] << endr << vIn[2] << 0 << -vIn[0] << endr << -vIn[1] << vIn[0] << 0 << endr;
	
	return skM;
}
