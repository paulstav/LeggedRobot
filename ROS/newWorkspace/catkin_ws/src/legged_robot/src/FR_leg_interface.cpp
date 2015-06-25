#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <inttypes.h>
#include <errno.h>
#include <error.h>

#include "dm7820_library.h"


#include "ros/ros.h"
#include "legged_robot/LegEncoders.h"
#include "legged_robot/LegCommand.h"

#include <sstream>

/*

	Front Right Leg Interface ROS node that reads encoders positions in counts from DM35820 card and post it to FR_encoders_feedback topic. It also reads the PWM command from the FR_command_dispatch topic and set the corresponding output to the DM35820 card.
	
	Board ID - 1 
	
	P2.0 - PWM HIP
	P2.2 - PWM KNEE
	P2.8 - DIR HIP
	P2.9 - DIR KNEE
	
	ENCODER HIP
	P0.0 - A
	P0.2 - B
	
	ENCODER KNEE
	P0.8 - A
	P0.10 - B
	
	ENCODER HEEL
	P1.0 - A
	P1.2 - B
	
	ENCODER SPRING
	P1.8 - A
	P1.10 - B

*/

#define PWM_GEN_PERIOD 1000
#define DIR_HIP 0x0100
#define DIR_KNEE 0x0200

// Global variables
bool sendMsg = false; // Flag set high when we want to transmit a message via topic
legged_robot::LegEncoders encoders_msg;  // Command message for topic
DM7820_Board_Descriptor *board;  // Digital I/O board descriptor and status variables
DM7820_Error dm7820_status;
int16_t pwm_periodA, pwm_periodB;

// Callback function for reception of leg command message from topic
void commandCallback(const legged_robot::LegCommand::ConstPtr& msg)
{
 // Set the received pwm values to the DM35820 interface
 sendMsg = true;
 //pwm_periodA = (int16_t)(((double)msg.pwm_duty_hip / 100.0 )  * ((double)PWM_GEN_PERIOD));
 pwm_periodA = ((int16_t)msg->pwm_duty_hip * 10);
 //pwm_periodB = (int16_t)(((doublemsg.pwm_duty_knee / 100.0 )  * ((double)PWM_GEN_PERIOD));
 pwm_periodB = ((int16_t)msg->pwm_duty_knee * 10);
}


// Main Function
int main(int argc, char **argv)
{
  int i, msg_count;
  dm7820_incenc_phase_filter phase_filter;
  unsigned long int minor_number = 1;
  uint16_t value_0_a, value_0_b, value_1_a, value_1_b;
  uint16_t pwm_dutyA, pwm_dutyB, DIRstatus;
  
  msg_count = 0;
  

  // Initialize ROS node
  ros::init(argc, argv, "FR_leg_interface");
  ros::NodeHandle n;
  // Initialize the publisher for encoder data post
  ros::Publisher motor_interface_pub = n.advertise<legged_robot::LegEncoders>("FR_encoders_feedback", 1000);
  // Initialize the subscribers for PWM command data reception
  ros::Subscriber motor_command_sub = n.subscribe("FR_command_dispatch", 1000, commandCallback);
  
  ros::Rate loop_rate(5000);
  
  ROS_INFO("Opening device with minor number %lu ...\n",minor_number);

  dm7820_status = DM7820_General_Open_Board(minor_number, &board);
  DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");
  
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");
  
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF, DM7820_STDIO_MODE_INPUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF, DM7820_STDIO_MODE_INPUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
  
  ROS_INFO("Initializing incremental encoders...\n");

  ROS_INFO("Setting master clock ...\n");
  dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");
  dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

  ROS_INFO("Disabling value register hold ...\n");
  dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");
  dm7820_status = DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_1, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

  ROS_INFO("Configuring encoders...\n");

  DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);
  
  dm7820_status = DM7820_IncEnc_Configure(board, DM7820_INCENC_ENCODER_0, phase_filter, DM7820_INCENC_INPUT_SINGLE_ENDED, 0x00, DM7820_INCENC_CHANNEL_INDEPENDENT, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");
  dm7820_status = DM7820_IncEnc_Configure(board, DM7820_INCENC_ENCODER_1, phase_filter, DM7820_INCENC_INPUT_SINGLE_ENDED, 0x00, DM7820_INCENC_CHANNEL_INDEPENDENT, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

  ROS_INFO("Setting initial value for channels...\n");
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_CHANNEL_A, 0x8000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_CHANNEL_B, 0x8000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_CHANNEL_A, 0x8000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");
  dm7820_status = DM7820_IncEnc_Set_Independent_Value(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_CHANNEL_B, 0x8000);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Independent_Value()");
  
  ROS_INFO("Enabling incremental encoders ...\n");
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");
  dm7820_status = DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0xFF);
  DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");
  
  ROS_INFO("Disabling pulse width modulators ...\n");
  dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");
  
  ROS_INFO("Initializing digital I/O port 2 ...\n");
  ROS_INFO("Setting bits to peripheral output ...\n");
  dm7820_status = DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF, DM7820_STDIO_MODE_PER_OUT);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
  ROS_INFO("Setting bits to output PWM peripheral ...\n");
  dm7820_status = DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0x00FF, DM7820_STDIO_PERIPH_PWM);
  DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");
  ROS_INFO("Initializing pulse width modulator 0 ...\n");
  ROS_INFO("Setting period master clock ...\n");
  dm7820_status = DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_PERIOD_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");
  ROS_INFO("Setting period to 20 KHz...\n");
  dm7820_status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 1000);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");
  ROS_INFO("Setting width master clock ...\n");
  dm7820_status = DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_WIDTH_MASTER_25_MHZ);
  DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");
  
  ROS_INFO("Staring communication with FR leg interface node.");
  
  while (ros::ok())
  {
	// Get encoder values
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_CHANNEL_A, &value_0_a);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_0, DM7820_INCENC_CHANNEL_B, &value_0_b);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_CHANNEL_A, &value_1_a);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");
	dm7820_status = DM7820_IncEnc_Get_Independent_Value(board, DM7820_INCENC_ENCODER_1, DM7820_INCENC_CHANNEL_B, &value_1_b);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Independent_Value()");
	
	// Publish encoder values to the topic
	encoders_msg.encoder_hip = (int32_t)value_0_a;
	encoders_msg.encoder_knee = (int32_t)value_0_b;
	encoders_msg.encoder_heel = (int32_t)value_1_a;
	encoders_msg.encoder_spring = (int32_t)value_1_b;
	motor_interface_pub.publish(encoders_msg);
	
	msg_count++;
	if(msg_count == 1000)
	{
		msg_count = 0;
		ROS_INFO("Encoders : %d %d %d %d\n", (int16_t)(value_0_a - 0x8000), (int16_t)(value_0_b - 0x8000), (int16_t)(value_1_a - 0x8000), (int16_t)(value_1_b - 0x8000));
	}
	// If a PWM command has been received send it to the drive
	if(sendMsg)
	{
		sendMsg = false;
		DIRstatus = 0;
		if(pwm_periodA < 0)
		{
			pwm_dutyA = (uint16_t)(-1 * pwm_periodA);
		}
		else
		{
			pwm_dutyA = (uint16_t)(pwm_periodA);
			DIRstatus |= DIR_HIP;
		}
		if(pwm_periodB < 0)
		{
			pwm_dutyB = (uint16_t)(-1 * pwm_periodB);
		}
		else
		{
			pwm_dutyB = (uint16_t)(pwm_periodB);
			DIRstatus |= DIR_KNEE;
		}
		dm7820_status = DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_2, DIRstatus);
		DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");
		dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, pwm_dutyA);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
		dm7820_status = DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0, DM7820_PWM_OUTPUT_A, pwm_dutyB);
		DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");
	}
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
