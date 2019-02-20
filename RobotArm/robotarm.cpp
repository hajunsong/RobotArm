#include "robotarm.h"

void load_data(string file_name, unsigned int row, unsigned int col, double *data) {
	FILE *fp_in;
	const int buffer = 10000;
	char *ptr, basic[buffer], *token;
	fopen_s(&fp_in, file_name.c_str(), "r");
	uint i = 0, j = 0;
	while (fgets(basic, buffer, fp_in) != NULL)
	{
		j = 0;
		ptr = strtok_s(basic, "\t", &token);
		while (ptr != NULL) {
			data[i * col + j] = atof(ptr);
			ptr = strtok_s(NULL, "\t", &token);
			j++;
		}
		i++;
		if (i >= row) break;
	}
	fclose(fp_in);
}

RobotArm::RobotArm(uint numbody, uint DOF) {
	num_body = numbody;
	dof = DOF;

	PH = new double[num_body * dof];
	PH_pos = new double[num_body * 3];
	PH_ori = new double[num_body * 3];
	delta_q = new double[dof];
	J = new double[num_body * dof];
	JD = new double[dof * num_body];

	body = new Body[num_body];

	// read data
	start_time = 0;
	end_time = 1;
	h = 0.001;
	g = -9.80665;

	double *A0_ptr = body[0].A0;
	*(A0_ptr++) = 1;	*(A0_ptr++) = 0;	*(A0_ptr++) = 0;
	*(A0_ptr++) = 0;	*(A0_ptr++) = 1;	*(A0_ptr++) = 0;
	*(A0_ptr++) = 0;	*(A0_ptr++) = 0;	*(A0_ptr) = 1;
	double *r0_ptr = body[0].r0;
	*(r0_ptr++) = 2.7E-002;	*(r0_ptr++) = 2.7E-002;	*(r0_ptr++) = 5.2204989872E-002;
	double *C01_ptr = body[0].C01;
	*(C01_ptr++) = 1;	*(C01_ptr++) = 0;	*(C01_ptr++) = 0;
	*(C01_ptr++) = 0;	*(C01_ptr++) = 1;	*(C01_ptr++) = 0;
	*(C01_ptr++) = 0;	*(C01_ptr++) = 0;	*(C01_ptr++) = 1;
	double *s01p_ptr = body[0].s01p;
	*(s01p_ptr++) = 0;	*(s01p_ptr++) = 0;	*(s01p_ptr++) = 0.0557950101;

	double *Cij_ptr, *sijp_ptr;
	switch (num_body) {
		case 6:
			// body 6 variable
			body[5].qi = 0;
			body[5].qi_dot = 0;
			body[5].mi = 0.8107375798;
			body[5].qi_init = body[5].qi;
			Cij_ptr = body[5].Cij;
			*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 1;
			sijp_ptr = body[5].sijp;
			*(sijp_ptr++) = 0.1517;	*(sijp_ptr++) = -1.0E-003;	*(sijp_ptr++) = 1.0E-003;
			goto CASE5;
		CASE5:
		case 5:
			// body 5 variable
			body[4].qi = -0.261799387779586;
			body[4].qi_dot = 0;
			body[4].mi = 1.2199454236;
			body[4].qi_init = body[4].qi;
			Cij_ptr = body[4].Cij;
			*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = -1;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;	*(Cij_ptr) = 0;
			sijp_ptr = body[4].sijp;
			*(sijp_ptr++) = 0.0510000000;	*(sijp_ptr++) = -0.0020000000;	*(sijp_ptr++) = 0.0595000000;
			goto CASE4;
		CASE4:
		case 4:
			// body 4 variable
			body[3].qi = 0;
			body[3].qi_dot = 0;
			body[3].mi = 6.3213616986E-002;
			body[3].qi_init = body[3].qi;
			Cij_ptr = body[3].Cij;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = 0;	*(Cij_ptr++) = 1;
			*(Cij_ptr++) = 0;	*(Cij_ptr++) = -1;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 1;	*(Cij_ptr++) = 0;	*(Cij_ptr) = 0;
			sijp_ptr = body[3].sijp;
			*(sijp_ptr++) = -0.0570000000;	*(sijp_ptr++) = 0.0000000017;	*(sijp_ptr++) = 0.0429991604;
			goto CASE3;
		CASE3:
		case 3:
			// body 3 variable
			body[2].qi = -1.04719755117959;
			body[2].qi_dot = 0;
			body[2].mi = 2.5064428652;
			body[2].qi_init = body[2].qi;
			Cij_ptr = body[2].Cij;
			*(Cij_ptr++) = 0.0000000000;	*(Cij_ptr++) = 0.7071067812;	*(Cij_ptr++) = 0.7071067812;
			*(Cij_ptr++) = 0.0000000000;	*(Cij_ptr++) = -0.7071067812;	*(Cij_ptr++) = 0.7071067812;
			*(Cij_ptr++) = 1;				*(Cij_ptr++) = 0.0000000000;	*(Cij_ptr) = 0.0000000000;
			sijp_ptr = body[2].sijp;
			*(sijp_ptr++) = 0.2353297685;	*(sijp_ptr++) = 0.1929768844;	*(sijp_ptr++) = 0.0050000000;
			goto CASE2;
		CASE2:
		case 2:
			// body 2 variable
			body[1].qi = -0.261799387779586;
			body[1].qi_dot = 0;
			body[1].mi = 1.4358314521;
			body[1].qi_init = body[1].qi;
			Cij_ptr = body[1].Cij;
			*(Cij_ptr++) = 0.707106781184743;	*(Cij_ptr++) = -0.707106781188352;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 0.707106781188352;	*(Cij_ptr++) = 0.707106781184743;	*(Cij_ptr++) = 0;
			*(Cij_ptr++) = 0;					*(Cij_ptr++) = 0;					*(Cij_ptr) = 1;
			sijp_ptr = body[1].sijp;
			*(sijp_ptr++) = 0.3342028602;	*(sijp_ptr++) = 2.9981665639E-002;	*(sijp_ptr++) = 0;
			goto CASE1;
		CASE1:
		case 1:
			// body 1 variable
			body[0].qi = 0;
			body[0].qi_dot = 0;
			body[0].mi = 0.8465182847;
			body[0].qi_init = body[0].qi;
			Cij_ptr = body[0].Cij;
			*(Cij_ptr++) = 1;						*(Cij_ptr++) = -1.02068239344618e-11;	*(Cij_ptr++) = -1.02068239345139e-11;
			*(Cij_ptr++) = 1.02068239344618e-11;	*(Cij_ptr++) = -5.10341196736114e-12;	*(Cij_ptr++) = 1;
			*(Cij_ptr++) = -1.02068239345139e-11;	*(Cij_ptr++) = -1;						*(Cij_ptr++) = -5.10341196725696e-12;
			sijp_ptr = body[0].sijp;
			*(sijp_ptr++) = 0.0000000000;	*(sijp_ptr++) = 1.2E-002;	*(sijp_ptr++) = 3.0E-002;
	}

	// define Y vector
	Y = new double[2 * num_body];
	Yp = new double[2 * num_body];

	if (num_body == 1) {
		Y[0] = body[0].qi;
		Y[1] = body[0].qi_dot;
	}
	else {
		double *Y_ptr = Y;
		for (uint i = 0; i < num_body; i++) {
			*(Y_ptr++) = body[i].qi;
		}
		for (uint i = 0; i < num_body; i++) {
			*(Y_ptr++) = body[i].qi_dot;
		}
	}

	numeric = new Numerical();
}

RobotArm::~RobotArm() {
	delete[] PH;
	delete[] PH_pos;
	delete[] PH_ori;
	delete[] delta_q;
	delete[] J;
	delete[] JD;

	delete[] Y;
	delete[] Yp;

	delete[] body;
	delete numeric;
}

void RobotArm::run_kinematics()
{
	sprintf_s(file_name, 256, "hj_kinematics_result.txt");
	fopen_s(&fp, file_name, "w+");

	double *input = new double[1001 * 7];

	load_data("q_input.txt", 1001, 7, input);

	for(uint indx = 0; indx < 1001; indx++) {
		body[0].qi = input[indx*7+1];
		body[1].qi = input[indx*7+2];
		body[2].qi = input[indx*7+3];
		body[3].qi = input[indx*7+4];
		body[4].qi = input[indx*7+5];
		body[5].qi = input[indx*7+6];

		kinematics_analysis();

		save_data();

		printf("Time : %.3f[s]\n", t_current);

		t_current += h;
	}

	delete[] input;
	fclose(fp);
}

void RobotArm::run_inverse_kinematics() {
	sprintf_s(file_name, 256, "hj_inverse_kinematics_result.txt");
	fopen_s(&fp, file_name, "w+");

	double *input = new double[1001 * 7];

	load_data("adams_end_ik_input.txt", 1001, 7, input);

	double pos_d[3], ori_d[3];
	for (uint indx = 0; indx < 1001; indx++) {
		pos_d[0] = input[indx * 7 + 1];
		pos_d[1] = input[indx * 7 + 2];
		pos_d[2] = input[indx * 7 + 3];
		ori_d[0] = input[indx * 7 + 4];
		ori_d[1] = input[indx * 7 + 5];
		ori_d[2] = input[indx * 7 + 6];

		kinematics_analysis();

		inverse_kinematics_analysis(pos_d, ori_d);

		save_data();

		printf("Time : %.3f[s]\n", t_current);

		t_current += h;
	}

	delete[] input;
	fclose(fp);
}

void RobotArm::kinematics_analysis()
{
	for (uint indx = 0; indx < num_body; indx++) {
		// Orientation
		double *Aijpp_ptr = body[indx].Aijpp;
		*(Aijpp_ptr++) = cos(body[indx].qi);	*(Aijpp_ptr++) = -sin(body[indx].qi);	*(Aijpp_ptr++) = 0;
		*(Aijpp_ptr++) = sin(body[indx].qi);	*(Aijpp_ptr++) = cos(body[indx].qi);	*(Aijpp_ptr++) = 0;
		*(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 0;						*(Aijpp_ptr++) = 1;
		memset(body[indx].Hi, 0, sizeof(double) * 3);
		memset(body[indx].Ai, 0, sizeof(double) * 9);
		if (indx == 0) {
			double A0_C01[9] = { 0, };
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					for (uint k = 0; k < 3; k++) {
						A0_C01[i * 3 + j] += body[indx].A0[i * 3 + k] * body[indx].C01[k * 3 + j];
					}
				}
			}
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					body[indx].Hi[i] += A0_C01[i * 3 + j] * body[indx].u_vec[j];
					for (uint k = 0; k < 3; k++) {
						body[indx].Ai[i * 3 + j] += A0_C01[i * 3 + k] * body[indx].Aijpp[k * 3 + j];
					}
				}
			}
		}
		else {
			double Ai_Cij[9] = { 0, };
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					for (uint k = 0; k < 3; k++) {
						Ai_Cij[i * 3 + j] += body[indx - 1].Ai[i * 3 + k] * body[indx - 1].Cij[k * 3 + j];
					}
				}
			}
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					body[indx].Hi[i] += Ai_Cij[i * 3 + j] * body[indx].u_vec[j];
					for (uint k = 0; k < 3; k++) {
						body[indx].Ai[i * 3 + j] += Ai_Cij[i * 3 + k] * body[indx].Aijpp[k * 3 + j];
					}
				}
			}
		}
		// Position
		if (indx == 0) {
			double s01[3] = { 0, };
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					s01[i] += body[indx].A0[i * 3 + j] * body[indx].s01p[j];
				}
				body[indx].ri[i] = body[0].r0[i] + s01[i];
			}
		}
		else {
			memset(body[indx - 1].sij, 0, sizeof(double) * 3);
			for (uint i = 0; i < 3; i++) {
				for (uint j = 0; j < 3; j++) {
					body[indx - 1].sij[i] += body[indx - 1].Ai[i * 3 + j] * body[indx - 1].sijp[j];
				}
				body[indx].ri[i] = body[indx - 1].ri[i] + body[indx - 1].sij[i];
			}
		}
		memset(body[indx].rhoi, 0, sizeof(double) * 3);
		for (uint i = 0; i < 3; i++) {
			for (uint j = 0; j < 3; j++) {
				body[indx].rhoi[i] += body[indx].Ai[i * 3 + j] * body[indx].rhoip[j];
			}
			body[indx].ric[i] = body[indx].ri[i] + body[indx].rhoi[i];
		}
	}
	// end point
	for (uint i = 0; i < 3; i++) {
		body[5].sij[i] = 0;
		for (uint j = 0; j < 3; j++) {
			body[5].sij[i] += body[5].Ai[i * 3 + j] * body[5].sijp[j];
		}
		body[5].re[i] = body[5].ri[i] + body[5].sij[i];
	}
	body[5].roll = atan2(body[5].Ai[2 * 3 + 1], body[5].Ai[2 * 3 + 2]);
	body[5].pitch = atan2(-body[5].Ai[2 * 3 + 0], sqrt(pow(body[5].Ai[2 * 3 + 1], 2) + pow(body[5].Ai[2 * 3 + 2], 2)));
	body[5].yaw = atan2(body[5].Ai[1 * 3 + 0], body[5].Ai[0 * 3 + 0]);
}

void RobotArm::inverse_kinematics_analysis(double des_pos[3], double des_ang[3]) {
	for (uint i = 0; i < 3; i++) {
		PH_pos[i] = body[5].re[i] - des_pos[i];
	}
	PH_ori[0] = body[5].roll - des_ang[0];
	PH_ori[1] = body[5].pitch + des_ang[1];
	PH_ori[2] = body[5].yaw - des_ang[2];

	for (uint i = 0; i < 3; i++) {
		PH[i] = PH_pos[i];
		PH[i + 3] = PH_ori[i];
	}

	calJacobian(des_pos);

	double *U, *s, *V;
	U = new double[6 * 6];
	s = new double[6];
	V = new double[6 * 6];

	numeric->svdcmp(J, 6, 6, U, s, V);

	double lamda = 0.0001;
	int m_size = 6;
	memset(JD, 0, sizeof(double) * m_size*m_size);
	double temp[36] = { 0, };
	for (uint i = 0; i < 6; i++) {
		for (uint j = 0; j < 6; j++) {
			for (uint k = 0; k < 6; k++) {
				temp[j * 6 + k] = V[j * 6 + i] * U[k * 6 + i];
			}
		}
		for (uint j = 0; j < 6; j++) {
			for (uint k = 0; k < 6; k++) {
				JD[j * 6 + k] += (s[i] / (pow(s[i], 2) + pow(lamda, 2)))*temp[j * 6 + k];
			}
		}
	}

	delete[] s;
	delete[] U;
	delete[] V;

	memset(delta_q, 0, sizeof(double) * 6);
	for (uint i = 0; i < 6; i++) {
		for (uint j = 0; j < 6; j++) {
			delta_q[i] += JD[i * 6 + j] * PH[j];
		}
	}

	for (uint i = 0; i < 6; i++) {
		body[i].qi += -delta_q[i];
	}

	//double err_max = 0;
	//for (uint i = 0; i < 6; i++) {
	//	err_max = abs(PH[i]) > err_max ? abs(PH[i]) : err_max;
	//}
	//int NRcount = 0;

	//while (err_max >= 1e-4 && NRcount < 5) {
	//	NRcount++;

	//	calJacobian(des_pos);

	//	double *U, *s, *V;
	//	U = new double[36];
	//	s = new double[6];
	//	V = new double[36];

	//	numeric->svdcmp(J, 6, 6, U, s, V);

	//	double lamda = 0.0001;
	//	int m_size = 6;
	//	memset(JD, 0, sizeof(double) * m_size*m_size);
	//	double temp[36] = { 0, };
	//	for (uint i = 0; i < 6; i++) {
	//		for (uint j = 0; j < 6; j++) {
	//			for (uint k = 0; k < 6; k++) {
	//				temp[j * 6 + k] = V[j*6+i] * U[k * 6 + i];
	//			}
	//		}
	//		for (uint j = 0; j < 6; j++) {
	//			for (uint k = 0; k < 6; k++) {
	//				JD[j * 6 + k] += (s[i] / (pow(s[i], 2) + pow(lamda, 2)))*temp[j * 6 + k];
	//			}
	//		}
	//	}

	//	delete[] s;
	//	delete[] U;
	//	delete[] V;

	//	memset(delta_q, 0, sizeof(double) * 6);
	//	for (uint i = 0; i < 6; i++) {
	//		for (uint j = 0; j < 6; j++) {
	//			delta_q[i] += JD[i * 6 + j] * PH[j];
	//		}
	//	}

	//	for (uint i = 0; i < 6; i++) {
	//		body[i].qi += -delta_q[i];
	//	}

	//	kinematics_analysis();

	//	for (uint i = 0; i < 3; i++) {
	//		PH_pos[i] = body[5].re[i] - des_pos[i];
	//	}
	//	PH_ori[0] = body[5].roll - des_ang[0];
	//	PH_ori[1] = body[5].pitch + des_ang[1];
	//	PH_ori[2] = body[5].yaw - des_ang[2];

	//	for (uint i = 0; i < 3; i++) {
	//		PH[i] = PH_pos[i];
	//		PH[i + 3] = PH_ori[i];
	//	}

	//	double err_max = 0;
	//	for (uint i = 0; i < 6; i++) {
	//		err_max = abs(PH[i]) > err_max ? abs(PH[i]) : err_max;
	//	}
	//}
}

void RobotArm::calJacobian(double des_pos[3])
{
	double Jv[3 * 6], Jw[3 * 6];
	for (uint indx = 0; indx < 6; indx++) {
		for (uint i = 0; i < 3; i++) {
			body[indx].oi[i] = des_pos[i] - body[indx].ri[i];
		}
		tilde(body[indx].Hi, body[indx].zit);
		for (uint i = 0; i < 3; i++) {
			body[indx].Jvi[i] = 0;
			for (uint j = 0; j < 3; j++) {
				body[indx].Jvi[i] += body[indx].zit[i * 3 + j] * body[indx].oi[j];
			}
		}
		for (uint j = 0; j < 3; j++) {
			Jv[j * 6 + indx] = body[indx].Jvi[j];
			Jw[j * 6 + indx] = body[indx].Hi[j];
		}
	}
	memcpy(J, Jv, sizeof(double) * 3 * 6);
	memcpy(J + 3 * 6, Jw, sizeof(double) * 3 * 6);
}

void RobotArm::save_data() {
	fprintf_s(fp, "%.7f\t", t_current);
	for (uint i = 0; i < num_body; i++) {
		fprintf_s(fp, "%.7f\t", body[i].qi);
	}
	fprintf_s(fp, "%.7f\t%.7f\t%.7f\t", body[5].re[0], body[5].re[1], body[5].re[2]);
	fprintf_s(fp, "%.7f\t%.7f\t%.7f", body[5].roll, -body[5].pitch, body[5].yaw);
	fprintf_s(fp, "\n");
}
