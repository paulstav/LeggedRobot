#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "legged_robot/Encoder.h"
#include "legged_robot/PWM.h"

#include <sstream>

#define GEAR_RATIO 51
#define SATURATION_POS 10*GEAR_RATIO

int16_t acc_error, prev_error;
double moveVelocity;
int32_t desiredPos;
int msgs = 0;
bool sendMsg = false;
legged_robot::PWM pwm_msg;

void InitErrors()
{
	acc_error = prev_error = 0;
}

int8_t PID_controller (double P_Gain, double I_Gain, double D_Gain, double saturation, int32_t Ref_point, int32_t Enc_value) 
{
	int8_t output = 0;
	int32_t error = 0;
	double Pterm, Dterm, Iterm, PIDsum;
	
	// Error calculation
	error = Ref_point - Enc_value;
	Pterm = error * P_Gain;
	Dterm = (error-prev_error) * D_Gain;
	acc_error += error;
	Iterm = acc_error * I_Gain;
	
	// P I D output calculation
	PIDsum = Pterm + Iterm + Dterm;
	
	//Saturation Filter
	
	if(PIDsum > saturation)
	{
		PIDsum = saturation;
	}
	else if(PIDsum < - saturation)
	{
		PIDsum = -saturation;
	}
	
	//Update error
	prev_error = error;
	
	output = (int8_t)((PIDsum/saturation)*100);
	
	return output;
}

void encoderCallback(const legged_robot::Encoder::ConstPtr& msg)
{
 //ROS_INFO("I heard: [%u]", msg->encoder);
 //msgs++;
 pwm_msg.pwm_duty = PID_controller (0.05, 0.0, 0.0, moveVelocity, desiredPos, (int32_t)msg->encoder);
 sendMsg = true;
 //if(msgs == 1000)
 //{
	 //ROS_INFO("PWM: [%d]", pwm_msg.pwm_duty);
	 //msgs = 0;
 //}
}

int main(int argc, char **argv)
{
  int i, msg_count;
  legged_robot::Encoder encoder_msg;
  moveVelocity = SATURATION_POS;
  desiredPos = 0;
  
  msg_count = 0;
  
  InitErrors();

  ros::init(argc, argv, "BR_motor_pid");
  ros::NodeHandle n;
  ros::Publisher motor_pid_pub = n.advertise<legged_robot::PWM>("BR_pwm_feedback", 1000);
  ros::Subscriber motor_interface_sub = n.subscribe("BR_encoder_feedback", 1000, encoderCallback);
  ros::Rate loop_rate(12000);
  
  ROS_INFO("Staring communication with BR Motor Controller node.");
  while (ros::ok())
  {
	if(sendMsg)
	{
		sendMsg = false;
		motor_pid_pub.publish(pwm_msg);
	}
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
