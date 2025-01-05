// implementation file
// Function definitions go here
#include "platform.h"
#include <iostream>
#include <vector>
#include <math.h>

// BLA::Matrix<6,3,double> platform::getTest() const {
// 	return test;
// }

// _____________________________________________________________
// ======================== Quaternion =========================
BLA::Matrix<4,1,double> platform::quaternionQ(BLA::Matrix<3,1,double> n, double w) {
    // x, y, z 	- axis of rotation (unit vector)
    // w 		- amount to rotate by (degrees)
	double a = w * M_PI / 180.0;	// Convert to radians
	double w_prime = cos(a/2);
	double x_prime = n.storage[0]*sin(a/2);
	double y_prime = n.storage[1]*sin(a/2);
	double z_prime = n.storage[2]*sin(a/2);
	BLA::Matrix<4,1,double> Q = {x_prime, y_prime, z_prime, w_prime};
	return Q;
}

// _____________________________________________________________
// ========================= Rotating ==========================
BLA::Matrix<3,1,double> platform::rotate(BLA::Matrix<3,1,double> P, BLA::Matrix<3,1,double> G, BLA::Matrix<3,1,double> n, double w){
	BLA::Matrix<4,1,double> Q = quaternionQ(n, w);
	double Q0 = Q.storage[0];
	double Q1 = Q.storage[1];
	double Q2 = Q.storage[2];
	double Q3 = Q.storage[3];

	BLA::Matrix<3,3,double> Q_XYZxyz = {pow(Q0, 2) - pow(Q1, 2) - pow(Q2, 2) + pow(Q3, 2), 2*(Q0*Q1 + Q2*Q3), 2*(Q0*Q2 - Q1*Q3),
	                                    2*(Q0*Q1 - Q2*Q3), -pow(Q0, 2) + pow(Q1, 2) - pow(Q2, 2) + pow(Q3, 2), 2*(Q1*Q2 + Q0*Q3),
										2*(Q0*Q2 + Q1*Q3), 2*(Q1*Q2 - Q0*Q3), -pow(Q0, 2) - pow(Q1, 2) + pow(Q2, 2) + pow(Q3, 2)};

	BLA::Matrix<3,1,double> V = P - G;
	return Q_XYZxyz * V + G;
}

	// _____________________________________________________________
// ========================= Rotating ==========================
void platform::rotateAll(BLA::Matrix<3,1,double> n, double theta) {
	p1t = rotate(p1t0, top_center_origin, n, theta);
	p2t = rotate(p2t0, top_center_origin, n, theta);
	p3t = rotate(p3t0, top_center_origin, n, theta);
	p4t = rotate(p4t0, top_center_origin, n, theta);
	p5t = rotate(p5t0, top_center_origin, n, theta);
	p6t = rotate(p6t0, top_center_origin, n, theta);
}

double** platform::rotateAndReturn(BLA::Matrix<3,1,double> n, double theta) {
	p1t = rotate(p1t0, top_center_origin, n, theta);
	p2t = rotate(p2t0, top_center_origin, n, theta);
	p3t = rotate(p3t0, top_center_origin, n, theta);
	p4t = rotate(p4t0, top_center_origin, n, theta);
	p5t = rotate(p5t0, top_center_origin, n, theta);
	p6t = rotate(p6t0, top_center_origin, n, theta);

	return getTop();
}

double* platform::getServos(BLA::Matrix<3,1,double> n, double theta) {
	rotateAll(n, theta);

	static double servoAngles[6];
	servoAngles[0] = -1*asin((((pow((p1t.storage[0]-p1b.storage[0]), 2) + pow((p1t.storage[1]-p1b.storage[1]), 2) + pow((p1t.storage[2]-p1b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt(pow((p1t.storage[1]-p1b.storage[1]), 2) + pow((p1t.storage[2]-p1b.storage[2]), 2))) - atan((p1t.storage[1]-p1b.storage[1])/(p1t.storage[2]-p1b.storage[2]));
	servoAngles[1] =    asin((((pow((p2t.storage[0]-p2b.storage[0]), 2) + pow((p2t.storage[1]-p2b.storage[1]), 2) + pow((p2t.storage[2]-p2b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt(pow((p2t.storage[1]-p2b.storage[1]), 2) + pow((p2t.storage[2]-p2b.storage[2]), 2))) - atan((p2t.storage[1]-p2b.storage[1])/(p2t.storage[2]-p2b.storage[2]));
	servoAngles[2] =    asin((((pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2) + pow((p3t.storage[2]-p3b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2)) + pow((p3t.storage[2]-p3b.storage[2]), 2))) - atan(sqrt(pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2))/(p3t.storage[2]-p3b.storage[2]));
	servoAngles[3] = -1*asin((((pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2) + pow((p4t.storage[2]-p4b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2)) + pow((p4t.storage[2]-p4b.storage[2]), 2))) + atan(sqrt(pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2))/(p4t.storage[2]-p4b.storage[2]));
	servoAngles[4] =    asin((((pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2) + pow((p5t.storage[2]-p5b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2)) + pow((p5t.storage[2]-p5b.storage[2]), 2))) - atan(sqrt(pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2))/(p5t.storage[2]-p5b.storage[2]));
	servoAngles[5] = -1*asin((((pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2) + pow((p6t.storage[2]-p6b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2)) + pow((p6t.storage[2]-p6b.storage[2]), 2))) + atan(sqrt(pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2))/(p6t.storage[2]-p6b.storage[2]));

	// Calculate the servo angles
	// double servoAngle0 = -1*asin((((pow((p1t.storage[0]-p1b.storage[0]), 2) + pow((p1t.storage[1]-p1b.storage[1]), 2) + pow((p1t.storage[2]-p1b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt(pow((p1t.storage[1]-p1b.storage[1]), 2) + pow((p1t.storage[2]-p1b.storage[2]), 2))) - atan((p1t.storage[1]-p1b.storage[1])/(p1t.storage[2]-p1b.storage[2]));
	// double servoAngle1 =    asin((((pow((p2t.storage[0]-p2b.storage[0]), 2) + pow((p2t.storage[1]-p2b.storage[1]), 2) + pow((p2t.storage[2]-p2b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt(pow((p2t.storage[1]-p2b.storage[1]), 2) + pow((p2t.storage[2]-p2b.storage[2]), 2))) - atan((p2t.storage[1]-p2b.storage[1])/(p2t.storage[2]-p2b.storage[2]));
	// double servoAngle2 =    asin((((pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2) + pow((p3t.storage[2]-p3b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2)) + pow((p3t.storage[2]-p3b.storage[2]), 2))) - atan(sqrt(pow((p3t.storage[0]-p3b.storage[0]), 2) + pow((p3t.storage[1]-p3b.storage[1]), 2))/(p3t.storage[2]-p3b.storage[2]));
	// double servoAngle3 = -1*asin((((pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2) + pow((p4t.storage[2]-p4b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2)) + pow((p4t.storage[2]-p4b.storage[2]), 2))) + atan(sqrt(pow((p4t.storage[0]-p4b.storage[0]), 2) + pow((p4t.storage[1]-p4b.storage[1]), 2))/(p4t.storage[2]-p4b.storage[2]));
	// double servoAngle4 =    asin((((pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2) + pow((p5t.storage[2]-p5b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2)) + pow((p5t.storage[2]-p5b.storage[2]), 2))) - atan(sqrt(pow((p5t.storage[0]-p5b.storage[0]), 2) + pow((p5t.storage[1]-p5b.storage[1]), 2))/(p5t.storage[2]-p5b.storage[2]));
	// double servoAngle5 = -1*asin((((pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2) + pow((p6t.storage[2]-p6b.storage[2]), 2)) + pow(arm_len, 2) - pow(rod_len, 2))/(2*arm_len))/sqrt((pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2)) + pow((p6t.storage[2]-p6b.storage[2]), 2))) + atan(sqrt(pow((p6t.storage[0]-p6b.storage[0]), 2) + pow((p6t.storage[1]-p6b.storage[1]), 2))/(p6t.storage[2]-p6b.storage[2]));

	// // double* servoAngles = new double[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	// double* servoAngles = new double[6]{servoAngle0, servoAngle1, servoAngle2, servoAngle3, servoAngle4, servoAngle5};

	return servoAngles;
}


double platform::close2zero(double val) {
	if (val < 0.0001 && val > -0.0001) {
		return 0;
	}
	return val;
}

void platform::printTop() {
	std::cout << "theta: " << theta_top << std::endl;
	std::cout << "M_PI: " << M_PI << std::endl;
	std::cout << "sin(M_PI/2): " << sin(M_PI/2.0) << std::endl;
	std::cout << "sin(M_PI/3.0+theta_top): " << sin(M_PI/3.0+theta_top) << std::endl;
	std::cout << "close2zero(sin(M_PI/3.0+theta_top)): " << close2zero(sin(M_PI/3.0+theta_top)) << std::endl;
	std::cout << "hyp_top * sin(M_PI/3.0+theta_top): " << hyp_top * sin(M_PI/3.0+theta_top) << std::endl;
	std::cout << close2zero(p1t_flat.storage[0]) << ", " << close2zero(p1t_flat.storage[1]) << ", " << close2zero(p1t_flat.storage[2]) << std::endl;
	std::cout << close2zero(p2t_flat.storage[0]) << ", " << close2zero(p2t_flat.storage[1]) << ", " << close2zero(p2t_flat.storage[2]) << std::endl;
	std::cout << close2zero(p3t_flat.storage[0]) << ", " << close2zero(p3t_flat.storage[1]) << ", " << close2zero(p3t_flat.storage[2]) << std::endl;
	std::cout << close2zero(p4t_flat.storage[0]) << ", " << close2zero(p4t_flat.storage[1]) << ", " << close2zero(p4t_flat.storage[2]) << std::endl;
	std::cout << close2zero(p5t_flat.storage[0]) << ", " << close2zero(p5t_flat.storage[1]) << ", " << close2zero(p5t_flat.storage[2]) << std::endl;
	std::cout << close2zero(p6t_flat.storage[0]) << ", " << close2zero(p6t_flat.storage[1]) << ", " << close2zero(p6t_flat.storage[2]) << std::endl;

	// return ;
}

double** platform::getTop() {

	// Create the empty array
	double** top = new double*[6];
	for (int i = 0; i < 6; ++i) {
		top[i] = new double[3]{0.0f, 0.0f, 0.0f};
		// top[i] = new double[3]{double(i+1)*1.0f, double(i+1)*2.0f, double(i+1)*3.0f};
	}

	// Calculate the top points
	top[0][0] = p1t.storage[0];
	top[0][1] = p1t.storage[1];
	top[0][2] = p1t.storage[2];
	
	top[1][0] = p2t.storage[0];
	top[1][1] = p2t.storage[1];
	top[1][2] = p2t.storage[2];

	top[2][0] = p3t.storage[0];
	top[2][1] = p3t.storage[1];
	top[2][2] = p3t.storage[2];

	top[3][0] = p4t.storage[0];
	top[3][1] = p4t.storage[1];
	top[3][2] = p4t.storage[2];

	top[4][0] = p5t.storage[0];
	top[4][1] = p5t.storage[1];
	top[4][2] = p5t.storage[2];

	top[5][0] = p6t.storage[0];
	top[5][1] = p6t.storage[1];
	top[5][2] = p6t.storage[2];

	// Return the array
	return top;
}