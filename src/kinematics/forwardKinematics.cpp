#include <math.h> 
#include <stdio.h> 
#include "../robotConstants.h"
#include "kinematicTypes.h"

vec3P forwardKinematics(double t1, double t2, double t3, double r3){
	
	double x = L01 * cos(t1) + L12 * (cos(t1)*cos(t2)) + cos(t3)*(cos(t1)*cos(t2))*(r3 + L23) - sin(t3)*(cos(t1)*sin(t2))*(r3 + L23);
	double y = L01 * sin(t1) + L12 * (cos(t2)*sin(t1)) + cos(t3)*(cos(t2)*sin(t1))*(r3 + L23) + sin(t3)*(-sin(t1)*sin(t2))*(r3 + L23);
	double z = L12 * sin(t2) + cos(t2)*sin(t3)*(r3 + L23) + cos(t3)*sin(t2)*(r3 + L23);

	return vec3P(x, y, z);

}
