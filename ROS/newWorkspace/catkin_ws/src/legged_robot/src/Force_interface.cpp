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

// UDP buffer length
#define BUFLEN 512
// UDP port to receive from
#define PORT 2024

// Asynchronous UDP communication
#define ASYNC

// Tiva IMU board IP
#define BRD_IP "192.168.1.15"

#include "ros/ros.h"
#include "legged_robot/Force.h"

#include <sstream>

/*

	ROS node that reads Force sensor data (F and M x,y,z) via UDP from the Tiva board
	and publishes the received data to the IMU_feedback topic.

*/

// Global variables
bool gotMsg = false;   // Flag set high when message is received from UDP
int sock;   // The socket identifier for UDP communication
int msgs = 0;  // Incoming message counter
uint16_t force[6] = {0,0,0,0,0,0};  // Place raw force data here

// Generic error function
void error(char *s)
{
    perror(s);
    exit(1);
}

// Signal handler for asynchronous UDP
void sigio_handler(int sig)
{
   char buffer[BUFLEN]="";
   unsigned char val[2];
   struct sockaddr_in si_other;
   unsigned int slen=sizeof(si_other);
   ssize_t rcvbytes = 0;

   // Receive available bytes from UDP socket
   if ((rcvbytes = recvfrom(sock, &buffer, BUFLEN, 0, (struct sockaddr *)&si_other, &slen))==-1)
       error("recvfrom()");
   else
   {
     //ROS_INFO("%d bytes");
	 // Parse data , 6 int16 values
     if(buffer[0] == 0x47 && rcvbytes == 13)
     {
	   val[1] = (unsigned char)buffer[1];
	   val[0] = (unsigned char)buffer[2];
	   memcpy(&force[0], &val, 2);
       val[1] = (unsigned char)buffer[3];
	   val[0] = (unsigned char)buffer[4];
	   memcpy(&force[1], &val, 2);
	   val[1] = (unsigned char)buffer[5];
	   val[0] = (unsigned char)buffer[6];
	   memcpy(&force[2], &val, 2);
	   val[1] = (unsigned char)buffer[7];
	   val[0] = (unsigned char)buffer[8];
	   memcpy(&force[3], &val, 2);
	   val[1] = (unsigned char)buffer[9];
	   val[0] = (unsigned char)buffer[10];
	   memcpy(&force[4], &val, 2);
	   val[1] = (unsigned char)buffer[11];
	   val[0] = (unsigned char)buffer[12];
	   memcpy(&force[5], &val, 2);
	   // Raise flag that we received a message
       gotMsg = true;
     }
   }
}

// Function to enable asynchronous UDP communication
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

// Main Function
int main(int argc, char **argv)
{
  struct sockaddr_in si_me, si_other;
  int i, slen=sizeof(si_other), msg_count;
  char buf[BUFLEN], strout[28];
  legged_robot::Force force_msg;
  double realForce[6];
  
  msg_count = 0;
  

  // Initialize ROS node
  ros::init(argc, argv, "Force_interface");
  ros::NodeHandle n;
  // Initialize the publisher for Force data post
  ros::Publisher force_interface_pub = n.advertise<legged_robot::Force>("Force_feedback", 1000);
  ros::Rate loop_rate(10000);

  // Wait for ROS node to initialize
  while (!ros::ok());
  
  // Initialize UDP communication
  if ((sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
     error("socket");

  memset((char *) &si_me, 0, sizeof(si_me));
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, (struct sockaddr *)&si_me, sizeof(si_me))==-1)
      error("bind");

  enable_asynch(sock);
  
  ROS_INFO("Staring communication with Force Tiva board.");
  while (ros::ok())
  {
	// Initialize ROS message values
    force_msg.Fx = 0;
    force_msg.Fy = 0;
    force_msg.Fz = 0;
    force_msg.Mx = 0;
    force_msg.My = 0;
    force_msg.Mz = 0;

	// If we got a new message, publish to topic and print values every 500 messages
    if(gotMsg){
	  msg_count++;
      force_msg.Fx = force[0];
      force_msg.Fy = force[1];
      force_msg.Fz = force[2];
      force_msg.Mx = force[3];
      force_msg.My = force[4];
      force_msg.Mz = force[5];
      force_interface_pub.publish(force_msg);
	  if(msg_count >= 3000)
	  {
		msg_count = 0;

		ROS_INFO("Force : %u %u %u %u %u %u", (uint16_t)force_msg.Fx,(uint16_t)force_msg.Fy,(uint16_t)force_msg.Fz,(uint16_t)force_msg.Mx,(uint16_t)force_msg.My,(uint16_t)force_msg.Mz);
	  }
      gotMsg = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
