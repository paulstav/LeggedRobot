#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "legged_robot/Force.h"
#include "legged_robot/AccGyro.h"
#include "legged_robot/RobotInertiaMeasurements.h"
#include "legged_robot/RobotForces.h"

#include <sstream>

/*

	Sensor preprocessing node. Converts raw sensor measurements from subscribed topics to real measurements.

*/

// Global variables
int force_msg_counter = 0;  // Force message counter variable
int imu_msg_counter = 0;  // IMU message counter variable
bool force_calib_done = false;  // Flag to notify when force sensor measurement calibration is complete
int32_t forceAvg[6] = {0,0,0,0,0,0};  // Variable to hold accumulated force data
uint16_t ADCbias[6] = {0,0,0,0,0,0};  // Variable to hold the calculated ADC bias
uint16_t forceVals[6] = {0,0,0,0,0,0};  // Variable to hold the biased force measurements
double mitroo[6][6] = {{0.00714348836843724, 1.46675711496553e-05, -0.000708974719448603, -0.000394408977602031, 0.000212063382020297, -6.56628720645252e-05},{-0.000136788026667987, -0.00416486665290656, -9.63441018561965e-05, -0.000426888985059494, 6.48995382187101e-05, -9.61315740403023e-06},{-5.61175298334688e-05, -9.07224476669784e-05, -0.00525683484821953, -0.000918945078073619, 3.64329604507650e-05, -8.43138280556477e-05},{-0.000931524439675322, -0.0222112385558622, -0.00231067796677438, -0.0423560450215219, 0.000962219888163593, -0.000600888385029064},{0.0390197856132641, -0.000593839447940435, -0.00424160444274681, -0.00189446128883103, 0.0435724813121189, 0.000181573361554948},{-0.00386589814443687, -0.000378418170717930, 0.00233881542273756, 0.00248542598857867, -0.000378530307836139, 0.0533940436359266}};  //Variable to multiply raw force measurements to get actual sizes

// Variables for messages to be published
legged_robot::RobotForces rb_force_msg;
legged_robot::RobotInertiaMeasurements rb_imu_msg;
ros::Publisher robot_imu_pub;
ros::Publisher robot_force_pub;

// Callback function for reception of force value message from topic
void forceCallback(const legged_robot::Force::ConstPtr& msg)
{	
	if(force_calib_done)
	{
		// Subtract the ADC bias offset from the measurement
		forceVals[0] = (uint16_t)msg->Fx - ADCbias[0];
		forceVals[1] = (uint16_t)msg->Fy - ADCbias[1];
		forceVals[2] = (uint16_t)msg->Fz - ADCbias[2];
		forceVals[3] = (uint16_t)msg->Mx - ADCbias[3];
		forceVals[4] = (uint16_t)msg->My - ADCbias[4];
		forceVals[5] = (uint16_t)msg->Mz - ADCbias[5];
		
		rb_force_msg.realFx = 1.0*forceVals[0]*mitroo[0][0] + 1.0*forceVals[1]*mitroo[0][1] + 1.0*forceVals[2]*mitroo[0][2] + 1.0*forceVals[3]*mitroo[0][3] + 1.0*forceVals[4]*mitroo[0][4] + 1.0*forceVals[5]*mitroo[0][5];
		rb_force_msg.realFy = 1.0*forceVals[0]*mitroo[1][0] + 1.0*forceVals[1]*mitroo[1][1] + 1.0*forceVals[2]*mitroo[1][2] + 1.0*forceVals[3]*mitroo[1][3] + 1.0*forceVals[4]*mitroo[1][4] + 1.0*forceVals[5]*mitroo[1][5];
		rb_force_msg.realFz = 1.0*forceVals[0]*mitroo[2][0] + 1.0*forceVals[1]*mitroo[2][1] + 1.0*forceVals[2]*mitroo[2][2] + 1.0*forceVals[3]*mitroo[2][3] + 1.0*forceVals[4]*mitroo[2][4] + 1.0*forceVals[5]*mitroo[2][5];
		rb_force_msg.realMx = 1.0*forceVals[0]*mitroo[3][0] + 1.0*forceVals[1]*mitroo[3][1] + 1.0*forceVals[2]*mitroo[3][2] + 1.0*forceVals[3]*mitroo[3][3] + 1.0*forceVals[4]*mitroo[3][4] + 1.0*forceVals[5]*mitroo[3][5];
		rb_force_msg.realMy = 1.0*forceVals[0]*mitroo[4][0] + 1.0*forceVals[1]*mitroo[4][1] + 1.0*forceVals[2]*mitroo[4][2] + 1.0*forceVals[3]*mitroo[4][3] + 1.0*forceVals[4]*mitroo[4][4] + 1.0*forceVals[5]*mitroo[4][5];
		rb_force_msg.realMz = 1.0*forceVals[0]*mitroo[5][0] + 1.0*forceVals[1]*mitroo[5][1] + 1.0*forceVals[2]*mitroo[5][2] + 1.0*forceVals[3]*mitroo[5][3] + 1.0*forceVals[4]*mitroo[5][4] + 1.0*forceVals[5]*mitroo[5][5];
		
		robot_force_pub.publish(rb_force_msg);
	}
	
	force_msg_counter++;
	if((force_msg_counter > 20) && !force_calib_done)
	{
		// Accumulated 1000 force sensor measurements
		forceAvg[0] += (uint16_t)msg->Fx;
		forceAvg[1] += (uint16_t)msg->Fy;
		forceAvg[2] += (uint16_t)msg->Fz;
		forceAvg[3] += (uint16_t)msg->Mx;
		forceAvg[4] += (uint16_t)msg->My;
		forceAvg[5] += (uint16_t)msg->Mz;
		
		if(force_msg_counter == 1020)
		{
			// Find ADCbias by averaging 1000 measurements
			ADCbias[0] = (uint16_t)(forceAvg[0]/1000);
			ADCbias[1] = (uint16_t)(forceAvg[1]/1000);
			ADCbias[2] = (uint16_t)(forceAvg[2]/1000);
			ADCbias[3] = (uint16_t)(forceAvg[3]/1000);
			ADCbias[4] = (uint16_t)(forceAvg[4]/1000);
			ADCbias[5] = (uint16_t)(forceAvg[5]/1000);
			
			force_calib_done = true;
		}
	}
	
}

// Callback function for reception of inertia raw value message from topic
void accgyroCallback(const legged_robot::AccGyro::ConstPtr& msg)
{
	if((int8_t)msg->imu_dev == 1)
	{
		rb_imu_msg.accXmg = ((int16_t)msg->accX*1.0)*0.8192;
		rb_imu_msg.accYmg = ((int16_t)msg->accY*1.0)*0.8192;
		rb_imu_msg.accZmg = ((int16_t)msg->accZ*1.0)*0.8192;
		rb_imu_msg.gyroXdeg = ((int16_t)msg->gyroX*1.0)*0.013108;
		rb_imu_msg.gyroYdeg = ((int16_t)msg->gyroY*1.0)*0.013108;
		rb_imu_msg.gyroZdeg = ((int16_t)msg->gyroZ*1.0)*0.013108;
	}
	if((int8_t)msg->imu_dev == 2)
	{
		rb_imu_msg.accXmg = ((int16_t)msg->accX*1.0);
		rb_imu_msg.accYmg = ((int16_t)msg->accY*1.0);
		rb_imu_msg.accZmg = ((int16_t)msg->accZ*1.0);
		rb_imu_msg.gyroXdeg = ((int16_t)msg->gyroX*1.0)*0.05;
		rb_imu_msg.gyroYdeg = ((int16_t)msg->gyroY*1.0)*0.05;
		rb_imu_msg.gyroZdeg = ((int16_t)msg->gyroZ*1.0)*0.05;
	}
	
	robot_imu_pub.publish(rb_imu_msg);
	
	imu_msg_counter++;
}

// Main Function
int main(int argc, char **argv)
{
  
  // Initialize ROS node
  ros::init(argc, argv, "RobotSensorPreprocess");
  ros::NodeHandle n;
  // Initialize the publisher for robot inertia measurements data post
  robot_imu_pub = n.advertise<legged_robot::RobotInertiaMeasurements>("Robot_Inertia", 1000);
  // Initialize the publisher for robot real force data post
  robot_force_pub = n.advertise<legged_robot::RobotForces>("Robot_Forces", 1000);
  // Initialize the subscribers for raw imu and force data reception
  ros::Subscriber raw_imu_sub = n.subscribe("IMU_feedback", 1000, accgyroCallback);
  ros::Subscriber raw_force_sub = n.subscribe("Force_feedback", 1000, forceCallback);
  ros::Rate loop_rate(12000);
  
  ROS_INFO("Starting sensor pre-processing node.");
  while (ros::ok())
  {
	if(imu_msg_counter > 900)
	{
		imu_msg_counter = 0;
		ROS_INFO("ACC (mg)   : %lf %lf %lf", rb_imu_msg.accXmg, rb_imu_msg.accYmg, rb_imu_msg.accZmg);
        ROS_INFO("GYRO (deg) : %lf %lf %lf", rb_imu_msg.gyroXdeg, rb_imu_msg.gyroYdeg, rb_imu_msg.gyroZdeg);
	}
	
	if(force_calib_done && (force_msg_counter > 300))
	{
		imu_msg_counter = 0;
		ROS_INFO("F(x,y,z)  : %lf %lf %lf", rb_force_msg.realFx, rb_force_msg.realFy, rb_force_msg.realFz);
        ROS_INFO("M(x,y,z)  : %lf %lf %lf", rb_force_msg.realMx, rb_force_msg.realMy, rb_force_msg.realMz);
	}
	
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
