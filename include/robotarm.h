#pragma once

#include <iostream>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#include "numerical.h"

using namespace std;
typedef unsigned int uint;

class RobotArm
{
public:
	RobotArm(uint numbody, uint DOF);
	~RobotArm();
	void run_kinematics();
	void run_inverse_kinematics();

private:
    void tilde(float *a, float *b) {
		*(b++) = 0;	*(b++) = -a[2];	*(b++) = a[1];
		*(b++) = a[2];	*(b++) = 0;	*(b++) = -a[0];
		*(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
	}

	class Body
	{
	public:
        Body();
        ~Body();
		// base body information
        float A0[9], C01[9], s01p[3], J0p[9], r0[3], m0, A0_C01[9], C01_A01pp[9];
		// body initial data
        float qi, qi_dot, mi, qi_init;
        float ri[3], ri_dot[3], wi[3], rhoip[3], sijp[3], Jip[9], Cii[9], Cij[9], Ai_Cij[9], Cij_Aijpp[9];
		// Orientation
        float Aijpp[9], Ai[9], Hi[3], u_vec[3];
		// Position
        float sij[3], rhoi[3], ric[3], rit[9];
		// Velocity State
        float Bi[6], Yih[6];
		// Cartesian velocity state
        float Ti[36], wit[9], Yib[6], ric_dot[3];
		// Mass & Force
        float Jic[9], rict[9], rict_dot[9], Mih[36], Fic[3], Tic[3], Qih[6], Qih_g[6], Qih_c[6], Tg, Tc;
		// Velocity Coupling
        float rit_dot[9], dHi[3], Di[6];
		// System EQM
        float Ki[36], Li[6], Li_g[6], Li_c[6];
		// Acceleration
        float qi_ddot;
		// End point
        float Ce[9], sep[3], se[3], re[3], Ae[9], roll, pitch, yaw;
		// Jacobian
        float Jvi[3], Jwi[3], re_qi[3], Ae_qi[9], r6_qi[3], A6_qi[9], Aijpp_qi[9];
        float Ae_qi_31, Ae_qi_32, Ae_qi_33, Ae_qi_21, Ae_qi_11, roll_qi, pitch_qi, yaw_qi;

        float oi[3], zi[3], zit[3];
	};

	uint num_body, dof;
    float *PH, *PH_pos, *PH_ori, *delta_q, *J, *JD;

	// system variable
    float start_time, end_time, h, g, t_current;
	// state vector
    float *Y, *Yp;
	// file
	char file_name[256];
	FILE *fp;

	Body *body;
	Numerical *numeric;

    float lamda;

 	void kinematics();
    void inverse_kinematics(float pos_d[3], float ori_d[3]);
        void jacobian();

	void save_data();	
};

