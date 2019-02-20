#include <iostream>
#include "robotarm.h"

int main() {
	uint num_body = 6;
	uint dof = 6;

	RobotArm *robot = new RobotArm(num_body, dof);
	//robot->run_kinematics();
	robot->run_inverse_kinematics();
	delete robot;

	return 0;
}