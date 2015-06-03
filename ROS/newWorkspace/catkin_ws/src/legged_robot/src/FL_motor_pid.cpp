#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>


#include "ros/ros.h"
#include "legged_robot/Encoder.h"
#include "legged_robot/PWM.h"
#include "legged_robot/Position.h"

#include <sstream>

#define GEAR_RATIO 51
#define SATURATION_POS 10*GEAR_RATIO

/*

	Front Left Leg ROS node that reads encoder position in counts from FL_encoder_feedback topic and based on the desired position read from the FL_Position topic, runs a PID controller to calculate the PWM command. The calculated command is published to the FL_pwm_feedback topic.

*/

// Global variables
int16_t acc_error, prev_error; // Acceleration and previous error values
double moveVelocity; // The maximum velocity of movement
int32_t desiredPos;  // Holds the desired position
bool sendMsg = false; // Flag set high when we want to transmit a message via UDP
legged_robot::PWM pwm_msg;  // PWM message for topic

// PID error initialization function
void InitErrors()
{
	acc_error = prev_error = 0;
}

// The PID controller function 
// Input : P,I,D gains, saturation , reference point(desired) and current point (Enc_value)
// Output : PWM duty cycle
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

// Callback function for reception of Encoder value message from topic
void encoderCallback(const legged_robot::Encoder::ConstPtr& msg)
{
 // Calculate the PWM duty cycle from the PID function and raise flag to send PWM command message
 pwm_msg.pwm_duty = PID_controller (0.05, 0.0, 0.0, moveVelocity, desiredPos, (int32_t)msg->encoder);
 sendMsg = true;
}

// Callback function for reception of desired position message from topic
void positionCallback(const legged_robot::Position::ConstPtr& msg)
{
 // Extract the value and set it to the corresponding variable
 desiredPos = (int32_t)msg->desiredPos;
}

// Main Function
int main(int argc, char **argv)
{
  int i, msg_count;
  legged_robot::Encoder encoder_msg;
  moveVelocity = SATURATION_POS;
  desiredPos = 0;
  
  msg_count = 0;
  
  InitErrors();

  // Initialize ROS node
  ros::init(argc, argv, "FL_motor_pid");
  ros::NodeHandle n;
  // Initialize the publisher for PWM data post
  ros::Publisher motor_pid_pub = n.advertise<legged_robot::PWM>("FL_pwm_feedback", 1000);
  // Initialize the subscribers for Encoder data reception and desired position reception
  ros::Subscriber motor_interface_sub = n.subscribe("FL_encoder_feedback", 1000, encoderCallback);
  ros::Subscriber motor_position_sub = n.subscribe("FL_Position", 1000, positionCallback);
  ros::Rate loop_rate(12000);
  
  ROS_INFO("Staring communication with FL Motor Controller node.");
  while (ros::ok())
  {
	// If a PWM command has been calculated publish it to the topic
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
