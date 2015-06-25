#include <iostream>
#include <inttypes.h>
#include <armadillo>
#include <cmath>

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

double factorial(uint16_t n);
mat Skew(mat vIn);
mat GAMMA(mat omega, double dt, double k);
mat quaternion2Matrix(mat quat);
mat measur(mat qk, mat pk1, mat pk2, mat rk);
mat rot_vector2quaternion(mat vIn);

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
  
  //mat A(3,3, fill::zeros);
  mat Vt(3,1), p1_es(3,1), p2_es(3,1), r_es(3,1);
  mat q(4,1);
  Vt[0] = q[0] = 1.0;
  Vt[1] = q[1] = 2.0;
  Vt[2] = q[2] = 3.0;
  q[3] = 4.0;
  p1_es[0] = 0.27;
  p1_es[1] = 0.0;
  p1_es[2] = 0.195;
  p2_es[0] = 0.27;
  p2_es[1] = 0.0;
  p2_es[2] = -0.195;
  r_es[0] = 0.0;
  r_es[1] = 0.3;
  r_es[2] = 0.0;
  
  
  cout << "Starting test" << endl;
  
  //mat A = Skew(Vt);
  //mat A = GAMMA(Vt, 0.001, 2);
  //mat A = quaternion2Matrix(q);
  //mat A = measur(q, p1_es, p2_es, r_es);
  mat A = rot_vector2quaternion(Vt);
  
  cout << "A" << endl << A << endl;
  
  //A = GAMMA(Vt, 0.001, 2);
  
  //cout << "A" << endl << A << endl;
  
  return 0;
}

double factorial(uint16_t n)
{
	double ret = 1.0;
	
	while(n>0)
	{
		ret *= (double)n;
		n--;
	}
	
	return ret;
}

mat Skew(mat vIn)
{
	mat skM(3,3);
	
	skM << 0 << -vIn[2] << vIn[1] << endr << vIn[2] << 0 << -vIn[0] << endr << -vIn[1] << vIn[0] << 0 << endr;
	
	return skM;
}

mat GAMMA(mat omega, double dt, double k)
{
	mat retM(3,3, fill::zeros);
	double b = ((int32_t)k)%2;
	double m = (k-b)/2;
	double sum = 0.0, sign, ser_prod, skew_gain, skew_sqr_gain, cosw, sinw;
	double norma = norm(omega);
	uint16_t n;

	if(b == 0.0)
	{
		if(norma > 0.1)
		{
			sinw = pow(-1.0, m)*sin(norma*dt);
			for(n=0; n<m; n++)
			{
				sign = pow(-1.0, (double)(m+n+1.0));
				ser_prod = pow((norma*dt),(double)(2.0*n+1.0));
				sum += sign*ser_prod/factorial((uint16_t)(2*n+1));
			}
			
			skew_gain = 1.0/pow(norma,(2.0*m+1.0))*(sinw+sum);
			
			sum = 0.0;
			
			cosw = pow(-1.0,(m+1.0))*cos(norma*dt);
			for(n=0; n<=m; n++)
			{
				sign = pow(-1.0,(m+n));
				ser_prod = pow((norma*dt), (double)(2.0*n));
				sum += sign*ser_prod/factorial((uint16_t)(2*n));
			}

			skew_sqr_gain = 1.0/pow(norma,(2.0*m+2.0))*(cosw+sum);	
		}
		else
		{
			skew_gain = pow(dt,(2.0*m+1.0))/factorial((uint16_t)(2.0*m+1.0));
			skew_sqr_gain = pow(-1.0,(2.0*m+1.0))*(pow(dt,(2.0*m+2.0))/factorial((uint16_t)(2.0*m+2.0)));
		}
	}
	else
	{
		if(norma > 0.1)
		{
			cosw = pow(-1.0,(m+1.0))*cos(norma*dt);
			for(n=0; n<=m; n++)
			{
				sign = pow(-1.0, (double)(m+n));
				ser_prod = pow((norma*dt),(double)(2.0*n));
				sum += sign*ser_prod/factorial((uint16_t)(2*n));
			}
			
			skew_gain = 1.0/pow(norma,(2.0*m+2.0))*(cosw+sum);
			
			sum = 0.0;
			
			sinw = pow(-1.0, m+1.0)*sin(norma*dt);
			for(n=0; n<=m; n++)
			{
				sign = pow(-1.0,(m+n));
				ser_prod = pow((norma*dt), (double)(2.0*n+1.0));
				sum += sign*ser_prod/factorial((uint16_t)(2*n+1));
			}

			skew_sqr_gain = 1.0/pow(norma,(2.0*m+3.0))*(sinw+sum);
			
		}
		else
		{
			skew_gain = pow(-1.0,(2.0*m+1.0))*(pow(dt,2.0*m+2.0)/factorial((uint16_t)(2.0*m+2.0)));
			skew_sqr_gain = 0;
		}
	}
	
	retM = (pow(dt, k)/factorial((uint16_t)k))*eye<mat>(3,3) + skew_gain*Skew(omega) + skew_sqr_gain*Skew(omega)*Skew(omega);
	
	return retM;
}

mat quaternion2Matrix(mat quat)
{
	mat q13(3,1);
	mat M(3,3);
	q13[0] = quat[0];
	q13[1] = quat[1];
	q13[2] = quat[2];
	
	
	M = (2*quat[3]*quat[3] - 1)*eye<mat>(3,3) + 2*(q13*q13.t()) + 2*quat[3]*Skew(q13);
	
	return M.t();
	
}

mat measur(mat qk, mat pk1, mat pk2, mat rk)
{
	mat H(6,18), H1(3,18), H2(3,18);
	mat hz1(3,6), hz2(3,6), hz3(3,6), hz4(3,12);
	
	mat Cq = quaternion2Matrix(qk);
	
	hz1 = join_horiz(-Cq, zeros<mat>(3,3));
	hz2 = join_horiz(Skew(Cq*(pk1-rk)), Cq);
	hz3 = join_horiz(zeros<mat>(3,3), zeros<mat>(3,3));
	hz4 = join_horiz(hz1, hz2);
	H1 = join_horiz(hz4, hz3);
	
	hz1 = join_horiz(-Cq, zeros<mat>(3,3));
	hz2 = join_horiz(Skew(Cq*(pk2-rk)), zeros<mat>(3,3));
	hz3 = join_horiz(Cq, zeros<mat>(3,3));
	hz4 = join_horiz(hz1, hz2);
	H2 = join_horiz(hz4, hz3);
	
	H = join_vert(H1,H2);
	
	return H;
}

mat rot_vector2quaternion(mat vIn)
{
	mat M(1,4);
	double q1, q2, q3, q4;
	double norma = sqrt(vIn[0]*vIn[0] + vIn[1]*vIn[1] + vIn[2]*vIn[2]);
	
	if(norma > 0.1)
	{
		q1 = vIn[0]*sin(norma/2.0)/norma;
		q2 = vIn[1]*sin(norma/2.0)/norma;
		q3 = vIn[2]*sin(norma/2.0)/norma;
		q4 = cos(norma/2.0);
	}
	else
	{
		q1 = vIn[0]/2.0;
		q2 = vIn[1]/2.0;
		q3 = vIn[2]/2.0;
		q4 = 1.0;
	}
	
	M << q1 << q2 << q3 << q4 << endr;
	
	return M.t();
}
