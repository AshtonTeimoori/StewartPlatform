#pragma once

#define _USE_MATH_DEFINES

#include "BasicLinearAlgebra/BasicLinearAlgebra.h"

#include <math.h>
// #include <vector.h>

// Specifications file
// Class declaration goes here
class platform {
	// =============================================================
	//                            PRIVATE
	// =============================================================
	private:

	// All global variable initializations can be considered ICs
	BLA::Matrix<3,1,double> origin = {0.0, 0.0, 0.0};     // Origin (Everything references this)

	// Base Geometry
	double rod_end_adjustment = 0.0932335;                // in (small distance from center of servo and rotation of rod end)
	double opp_base = 1.338583;                           // in (the distance from the center of the edge to the servo rotation)
	double adj_base = 3.807087 + rod_end_adjustment;      // in (The perpendicular distance from center to edge)
	double hyp_base = sqrt(pow(opp_base, 2) + pow(adj_base, 2));
	double theta_base = atan(opp_base/adj_base);
	
	// Base points go CCW (if you clump the two servos closest together, the left servo is the first point)
	BLA::Matrix<3,1,double> p1b = {hyp_base*cos(-theta_base), hyp_base*sin(-theta_base), 0};
	BLA::Matrix<3,1,double> p2b = {hyp_base*cos(theta_base), hyp_base*sin(theta_base), 0};
	BLA::Matrix<3,1,double> p3b = {hyp_base*cos(2*M_PI/3-theta_base), hyp_base*sin(2*M_PI/3-theta_base), 0};
	BLA::Matrix<3,1,double> p4b = {hyp_base*cos(2*M_PI/3+theta_base), hyp_base*sin(2*M_PI/3+theta_base), 0};
	BLA::Matrix<3,1,double> p5b = {hyp_base*cos(4*M_PI/3-theta_base), hyp_base*sin(4*M_PI/3-theta_base), 0};
	BLA::Matrix<3,1,double> p6b = {hyp_base*cos(4*M_PI/3+theta_base), hyp_base*sin(4*M_PI/3+theta_base), 0};
	// BLA::Matrix<3,1,double> base_points = {p1b, p2b, p3b, p4b, p5b, p6b};

	// Platform Geometry
	double opp_top = 0.55787;   					// in (the distance from the center of the edge to the rod attachment)
	double adj_top = 3.717599 + rod_end_adjustment; // in (The purpendicular distance from center to edge)
	double hyp_top = sqrt(pow(opp_top, 2) + pow(adj_top, 2));
	double theta_top = atan(opp_top/adj_top);
	
	// Geometry wihtout height adjustment p(#)t_flat
	BLA::Matrix<3,1,double> p1t_flat = {hyp_top * cos(theta_top-M_PI/3.0),  hyp_top * sin(theta_top-M_PI/3.0),  0};
	BLA::Matrix<3,1,double> p2t_flat = {hyp_top * cos(M_PI/3.0-theta_top),  hyp_top * sin(M_PI/3.0-theta_top),  0};
	BLA::Matrix<3,1,double> p3t_flat = {hyp_top * cos(M_PI/3.0+theta_top),  hyp_top * sin(M_PI/3.0+theta_top),  0};
	BLA::Matrix<3,1,double> p4t_flat = {hyp_top * cos(M_PI-theta_top),      hyp_top * sin(M_PI-theta_top),      0};
	BLA::Matrix<3,1,double> p5t_flat = {hyp_top * cos(M_PI+theta_top),      hyp_top * sin(M_PI+theta_top),      0};
	BLA::Matrix<3,1,double> p6t_flat = {hyp_top * cos(-theta_top-M_PI/3.0), hyp_top * sin(-theta_top-M_PI/3.0), 0};

	// Other Important Parameters
	double arm_len = 1.220472;         			// Servo arm length
	double rod_len = 4.471177;         			// Rod length
	BLA::Matrix<6,3,double> rod_vec;
	BLA::Matrix<6,3,double> servo_vec;
	// servo_vec = np.zeros([6, 3])
	BLA::Matrix<3,1,double> height = {0, 0, 3.625};
	BLA::Matrix<3,1,double> top_center_origin = origin + height;
	BLA::Matrix<3,1,double> top_center = origin + height;

	// Top Orientation
	BLA::Matrix<3,1,double> x_unit = {1, 0, 0};
	BLA::Matrix<3,1,double> y_unit = {0, 1, 0};
	BLA::Matrix<3,1,double> z_unit = {0, 0, 1};
	// BLA::Matrix<3,1,double> top_origin = top_center_origin + array([x_unit, y_unit, z_unit]);
	// BLA::Matrix<3,1,double> top_orientation = np.copy(top_origin);
	BLA::Matrix<3,1,double> p1t = p1t_flat + height;
	BLA::Matrix<3,1,double> p2t = p2t_flat + height;
	BLA::Matrix<3,1,double> p3t = p3t_flat + height;
	BLA::Matrix<3,1,double> p4t = p4t_flat + height;
	BLA::Matrix<3,1,double> p5t = p5t_flat + height;
	BLA::Matrix<3,1,double> p6t = p6t_flat + height;

	BLA::Matrix<3,1,double> p1t0 = p1t_flat + height;
	BLA::Matrix<3,1,double> p2t0 = p2t_flat + height;
	BLA::Matrix<3,1,double> p3t0 = p3t_flat + height;
	BLA::Matrix<3,1,double> p4t0 = p4t_flat + height;
	BLA::Matrix<3,1,double> p5t0 = p5t_flat + height;
	BLA::Matrix<3,1,double> p6t0 = p6t_flat + height;

	// // Servo Angles
	// BLA::Matrix<6,1,double> servoAngle;

	// Servo Arm Orientation
	BLA::Matrix<3,1,double> arm1_abs = {0,                      -1*arm_len,             0};
	BLA::Matrix<3,1,double> arm2_abs = {0,                      arm_len,                0};
	BLA::Matrix<3,1,double> arm3_abs = {arm_len*cos(M_PI/6),    arm_len*sin(M_PI/6),    0};
	BLA::Matrix<3,1,double> arm4_abs = {-1*arm_len*cos(M_PI/6), -1*arm_len*sin(M_PI/6), 0};
	BLA::Matrix<3,1,double> arm5_abs = {-1*arm_len*cos(M_PI/6), arm_len*sin(M_PI/6),    0};
	BLA::Matrix<3,1,double> arm6_abs = {arm_len*cos(M_PI/6 ),   -1*arm_len*sin(M_PI/6), 0};

	BLA::Matrix<3,1,double> arm1_rel = p1b + arm1_abs;
	BLA::Matrix<3,1,double> arm2_rel = p2b + arm2_abs;
	BLA::Matrix<3,1,double> arm3_rel = p3b + arm3_abs;
	BLA::Matrix<3,1,double> arm4_rel = p4b + arm4_abs;
	BLA::Matrix<3,1,double> arm5_rel = p5b + arm5_abs;
	BLA::Matrix<3,1,double> arm6_rel = p6b + arm6_abs;
	
	


	// PRIVATE FUNCTIONS
	// ======================== Quaternion =========================
	BLA::Matrix<4,1,double> quaternionQ(BLA::Matrix<3,1,double> n, double w);

	// ========================= Rotating ==========================
	BLA::Matrix<3,1,double> rotate(BLA::Matrix<3,1,double> P, BLA::Matrix<3,1,double> G, BLA::Matrix<3,1,double> n, double w);
	void rotateAll(BLA::Matrix<3,1,double> n, double theta);

	// ======================== For Outputs ========================
	double close2zero(double val);


	// =============================================================
	//                            PUBLIC
	// =============================================================
	public:

	// PUBLIC FUNCTIONS
	void printTop();
	
	// _____________________________________________________________
	// ========================= Rotating ==========================
	double** rotateAndReturn(BLA::Matrix<3,1,double> n, double theta);

	// ========================= Servo Angles ==========================
	double* getServos(BLA::Matrix<3,1,double> n, double theta);

	double** getTop();

};
