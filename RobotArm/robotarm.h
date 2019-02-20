#pragma once

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "numerical.h"

using namespace std;
typedef unsigned int uint;

class RobotArm
{
public:
	RobotArm(uint numbody, uint DOF);
	~RobotArm();
	//void run();
	void run_kinematics();
	void run_inverse_kinematics();

private:
	void tilde(double *a, double *b) {
		*(b++) = 0;	*(b++) = -a[2];	*(b++) = a[1];
		*(b++) = a[2];	*(b++) = 0;	*(b++) = -a[0];
		*(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
	}

	class Body
	{
	public:
		Body() {u_vec[0] = 0; u_vec[1] = 0; u_vec[2] = 1; };
		~Body() {};
		// base body information
		double A0[9], C01[9], s01p[3], J0p[9], r0[3], m0;
		// body initial data
		double qi = 0, qi_dot = 0, mi = 0, qi_init;
		double ri[3], ri_dot[3], wi[3], rhoip[3], sijp[3], Jip[9], Cii[9], Cij[9];
		// Orientation
		double Aijpp[9], Ai[9], Hi[3], u_vec[3];
		// Position
		double sij[3], rhoi[3], ric[3], rit[9];
		// Velocity State
		double Bi[6], Yih[6];
		// Cartesian velocity state
		double Ti[36], wit[9], Yib[6], ric_dot[3];
		// Mass & Force
		double Jic[9], rict[9], rict_dot[9], Mih[36], Fic[3], Tic[3], Qih[6], Qih_g[6], Qih_c[6], Tg, Tc;
		// Velocity Coupling
		double rit_dot[9], dHi[3], Di[6];
		// System EQM
		double Ki[36], Li[6], Li_g[6], Li_c[6];
		// Acceleration
		double qi_ddot;
		// End point
		double re[3], Ae[9], roll, pitch, yaw;
		// Jacobian
		double oi[3], zit[9], Jvi[3];
	};

	uint num_body, dof;
	double *PH, *PH_pos, *PH_ori, *delta_q, *J, *JD;

	// system variable
	double start_time, end_time, h, g, t_current;
	// state vector
	double *Y, *Yp;
	// file
	char file_name[256];
	FILE *fp;

	Body *body;
	Numerical *numeric;

 	void kinematics_analysis();
	void inverse_kinematics_analysis(double pos_d[3], double ori_d[3]);
		void calJacobian(double des_pos[3]);

	void save_data();
 	//void analysis();
 	//	void Y2qdq();
		//void inverse_kinematics_analysis(double des_pos[3], double des_ang[3]);
	//void save_data();
	
};

