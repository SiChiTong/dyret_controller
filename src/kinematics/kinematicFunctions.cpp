#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "kinematicFunctions.h"
#include "kinematicTypes.h"
#include "forwardKinematics.h"
#include "interpolation.h"

#include "dyret_common/Pose.h"

vec3P add(vec3P firstOperand, vec3P secondOperand) {
	double x = firstOperand.x() + secondOperand.x();
	double y = firstOperand.y() + secondOperand.y();
	double z = firstOperand.z() + secondOperand.z();

	vec3P vectorToReturn = vec3P(x, y, z);

	vectorToReturn.groundPoint = false;

	return vectorToReturn;
}

std::vector<vec3P> add(std::vector<vec3P> firstOperand, vec3P secondOperand) {

  std::vector<vec3P> vecToReturn(firstOperand.size());

  for (int i = 0; i < firstOperand.size(); i++){
    vecToReturn[i] = vec3P(firstOperand[i].x() + secondOperand.x(),
                           firstOperand[i].y() + secondOperand.y(),
                           firstOperand[i].z() + secondOperand.z());
  }

  return vecToReturn;
}

vec3P getWagPoint(double amplitude_x, double amplitude_y, double period, double time, double offset) {
	// Time given in seconds

	const double phaseOffset = 0.43f; // Offset to correct for front/back to side movement

	double calculatedPoint_x = amplitude_x/2 * tanh(3*sin( (2*M_PI*(time+(offset*period))) / period));
	double calculatedPoint_y = amplitude_y/2 * tanh(3*sin( (2*M_PI*(time+((offset+phaseOffset)*(period/2)))) / (period/2)));

	vec3P pointToReturn = vec3P(calculatedPoint_x, calculatedPoint_y, 0.0f);
	pointToReturn.groundPoint = false;

	return pointToReturn;
}

void printPose(std::vector<vec3P> givenPoseArray){
	fprintf(stderr, "\n");
	for (int i = 0; i < givenPoseArray.size(); i++) {
		fprintf(stderr, "%.2f, %.2f, %.2f\n", givenPoseArray[i].x(), givenPoseArray[i].y(), givenPoseArray[i].z());
	}
}