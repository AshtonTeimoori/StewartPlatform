#include <iostream>
#include "include/platform.h"
// #include "include/matplotlib-cpp/matplotlibcpp.h"

int main() {
	std::cout << "Testing the platform header" << std::endl;
	platform p = platform();

	double** top = p.getTop();

	std::cout << "\nPoints:\t\tx,\t\ty,\t\tz" << std::endl;
	for (int i = 0; i < 6; ++i) {
		std::cout << "Point " << i << ":\t" << top[i][0] << ",\t" << top[i][1] << ",\t" << top[i][2] << std::endl;
	}

	BLA::Matrix<3,1,double> n = {-0.88f, -0.47f, 0.0f};
	double theta = 0.0;

	top = p.rotateAndReturn(n, theta);

	std::cout << "\nPoints:\t\tx,\t\ty,\t\tz" << std::endl;
	for (int i = 0; i < 6; ++i) {
		std::cout << "Point " << i << ":\t" << top[i][0] << ",\t" << top[i][1] << ",\t" << top[i][2] << std::endl;
	}

	std::cout << "\nServos" << std::endl;
	double* servo_angles = p.getServos(n, theta);
	for (int i = 0; i < 6; ++i) {
		std::cout << "Servo " << i << ":\t" << servo_angles[i] * 180.0 / M_PI << std::endl;
	}
	return 0;
}