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
#define PORT 2022

#define ASYNC

#define BRD_IP "192.168.1.14"

#include "ros/ros.h"
#include "legged_robot/AccGyro.h"

#include <sstream>

bool gotMsg = false;
int sock;
int msgs = 0;
int16_t acc[3] = {0,0,0};
int16_t gyro[3] = {0,0,0};

void error(char *s)
{
    perror(s);
    exit(1);
}

void sigio_handler(int sig)
{
   char buffer[BUFLEN]="";
   unsigned char val[2];
   struct sockaddr_in si_other;
   unsigned int slen=sizeof(si_other);
   ssize_t rcvbytes = 0;

   if ((rcvbytes = recvfrom(sock, &buffer, BUFLEN, 0, (struct sockaddr *)&si_other, &slen))==-1)
       error("recvfrom()");
   else
   {
     if(buffer[0] == 0x43 && rcvbytes == 13)
     {
	   val[1] = (unsigned char)buffer[2];
	   val[0] = (unsigned char)buffer[1];
	   memcpy(&acc[0], &val, 2);
           val[1] = (unsigned char)buffer[4];
	   val[0] = (unsigned char)buffer[3];
	   memcpy(&acc[1], &val, 2);
	   val[1] = (unsigned char)buffer[6];
	   val[0] = (unsigned char)buffer[5];
	   memcpy(&acc[2], &val, 2);
	   val[1] = (unsigned char)buffer[8];
	   val[0] = (unsigned char)buffer[7];
	   memcpy(&gyro[0], &val, 2);
	   val[1] = (unsigned char)buffer[10];
	   val[0] = (unsigned char)buffer[9];
	   memcpy(&gyro[1], &val, 2);
	   val[1] = (unsigned char)buffer[12];
	   val[0] = (unsigned char)buffer[11];
	   memcpy(&gyro[2], &val, 2);
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

int main(int argc, char **argv)
{
  struct sockaddr_in si_me, si_other;
  int i, slen=sizeof(si_other), msg_count;
  char buf[BUFLEN], strout[28];
  legged_robot::AccGyro accgyro_msg;
  double realAcc[3], realGyro[3];
  
  msg_count = 0;
  

  ros::init(argc, argv, "IMU_interface");
  ros::NodeHandle n;
  ros::Publisher imu_interface_pub = n.advertise<legged_robot::AccGyro>("IMU_feedback", 1000);
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
  
  ROS_INFO("Staring communication with IMU Tiva board.");
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    accgyro_msg.accX = 0;
    accgyro_msg.accY = 0;
    accgyro_msg.accZ = 0;
    accgyro_msg.gyroX = 0;
    accgyro_msg.gyroY = 0;
    accgyro_msg.gyroZ = 0;


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    if(gotMsg){
	  msg_count++;
          accgyro_msg.accX = acc[0];
          accgyro_msg.accY = acc[1];
          accgyro_msg.accZ = acc[2];
          accgyro_msg.gyroX = gyro[0];
          accgyro_msg.gyroY = gyro[1];
          accgyro_msg.gyroZ = gyro[2];
          imu_interface_pub.publish(accgyro_msg);
	  if(msg_count >= 500)
	  {
		msg_count = 0;
		realAcc[0] = (accgyro_msg.accX*1.0)*0.8192;
                realAcc[1] = (accgyro_msg.accY*1.0)*0.8192;
                realAcc[2] = (accgyro_msg.accZ*1.0)*0.8192;
                realGyro[0] = (accgyro_msg.gyroX*1.0)*0.013108;
                realGyro[1] = (accgyro_msg.gyroY*1.0)*0.013108;
                realGyro[2] = (accgyro_msg.gyroZ*1.0)*0.013108;
		ROS_INFO("ACC : %d %d %d", (int16_t)accgyro_msg.accX,(int16_t)accgyro_msg.accY,(int16_t)accgyro_msg.accZ);
		ROS_INFO("ACC real : %f %f %f", realAcc[0],realAcc[1],realAcc[2]);
                ROS_INFO("GYRO : %d %d %d", (int16_t)accgyro_msg.gyroX,(int16_t)accgyro_msg.gyroY,(int16_t)accgyro_msg.gyroZ);
		ROS_INFO("GYRO real : %f %f %f", realGyro[0],realGyro[1],realGyro[2]);
	  }
      gotMsg = false;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
