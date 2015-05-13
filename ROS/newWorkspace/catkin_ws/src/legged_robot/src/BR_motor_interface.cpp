#include "arpa/inet.h"
#include "netinet/in.h"
#include "stdio.h"
#include "sys/types.h"
#include "sys/socket.h"
#include "unistd.h"
#include "string.h"
#include "stdlib.h"
#include "signal.h"
#include "unistd.h"
#include "fcntl.h"
#include <stdint.h>
#include <inttypes.h>

#define BUFLEN 512
#define PORT 2020

#define ASYNC

#define PORT_BRD 2019
#define BRD_IP "192.168.1.13"

#include "ros/ros.h"
#include "legged_robot/Encoder.h"
#include "legged_robot/PWM.h"

#include <sstream>

bool gotMsg = false;
int sock;
uint32_t encoderPos = 0;
int msgs = 0;
struct sockaddr_in si_pwm;
ssize_t SendPWMBytes = 2;
char SendBufferPWM[6];
int broad, slen=sizeof(si_pwm);

void error(char *s)
{
    perror(s);
    exit(1);
}

void sigio_handler(int sig)
{
   char buffer[BUFLEN]="";
   unsigned char val[4];
   struct sockaddr_in si_other;
   unsigned int slen=sizeof(si_other);
   ssize_t rcvbytes = 0;

   if ((rcvbytes = recvfrom(sock, &buffer, BUFLEN, 0, (struct sockaddr *)&si_other, &slen))==-1)
       error("recvfrom()");
   else
   {
     if(buffer[0] == 0x42)
     {
	   val[3] = (unsigned char)buffer[4];
	   val[2] = (unsigned char)buffer[3];
	   val[1] = (unsigned char)buffer[2];
	   val[0] = (unsigned char)buffer[1];
       /*encoderPos = buffer[2];
       encoderPos = (encoderPos << 8) | buffer[1];*/
	   /*encoderPos = val[0];
	   encoderPos = (encoderPos & 0x000000FF);
       encoderPos = ((encoderPos << 8) & 0xFF00) | val[1];*/
	   memcpy(&encoderPos, &val, 4);
       gotMsg = true;
     }
   }
}

int enable_asynch(int sock)
{
  int stat = -1;
  int flags;
  struct sigaction sa;

  flags = fcntl(sock, F_GETFL);
  fcntl(sock, F_SETFL, flags | O_ASYNC); 

  sa.sa_flags = 0;
  sa.sa_handler = sigio_handler;
  sigemptyset(&sa.sa_mask);

  if (sigaction(SIGIO, &sa, NULL))
    error("Error:");

  if (fcntl(sock, F_SETOWN, getpid()) < 0)
    error("Error:");

  if (fcntl(sock, F_SETSIG, SIGIO) < 0)
    error("Error:");
  return 0;
}

void pwmCallback(const legged_robot::PWM::ConstPtr& msg)
{
	SendBufferPWM[1] = (int8_t)msg->pwm_duty;
	if (sendto(broad, SendBufferPWM, SendPWMBytes, 0, (struct sockaddr *)&si_pwm, slen)==-1)
		error("sendto()");
	msgs++;
	if(msgs == 5000)
	{
		msgs = 0;
		ROS_INFO("I heard: [%d]", msg->pwm_duty);
	}
}

int main(int argc, char **argv)
{
  struct sockaddr_in si_me, si_other;
  int i, slen=sizeof(si_other), msg_count;
  char buf[BUFLEN], strout[28];
  
  msg_count = 0;
  memset(SendBufferPWM, 0, 6);
  
  if ((broad=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
       error("socket");
	   
  memset((char *) &si_pwm, 0, sizeof(si_pwm));
  si_pwm.sin_family = AF_INET;
  si_pwm.sin_port = htons(PORT_BRD);
   
  if (inet_aton(BRD_IP, &si_pwm.sin_addr)==0) {
       error("inet_aton() failed\n");
       exit(1);
  }
  
  SendBufferPWM[0] = 0x31;

  ros::init(argc, argv, "BR_motor_interface");
  ros::NodeHandle n;
  ros::Publisher motor_interface_pub = n.advertise<legged_robot::Encoder>("BR_encoder_feedback", 1000);
  ros::Subscriber motor_pid_sub = n.subscribe("BR_pwm_feedback", 1000, pwmCallback);
  ros::Rate loop_rate(10000);

  while (!ros::ok());
  
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
     error("socket");

  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, (struct sockaddr *)&si_me, sizeof(si_me))==-1)
      error("bind");

  enable_asynch(sock);
  
  ROS_INFO("Staring communication with BR Tiva board.");
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    legged_robot::Encoder encoder_msg;
    encoder_msg.encoder = 0;

    //ROS_INFO("%d", encoder_msg.encoder);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    if(gotMsg){
	  msg_count++;
      encoder_msg.encoder = encoderPos;
      motor_interface_pub.publish(encoder_msg);
	  if(msg_count >= 5000)
	  {
		msg_count = 0;
		//sprintf(strout,"%d",(uint32_t)encoder_msg.encoder);
		//ROS_INFO("%s", strout);
		ROS_INFO("%u", (uint32_t)encoder_msg.encoder);
	  }
      gotMsg = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
