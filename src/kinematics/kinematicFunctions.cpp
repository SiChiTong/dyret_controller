#include <iostream>
#include <vector>

#include "ros/ros.h"

#include "kinematicFunctions.h"
#include "kinematicTypes.h"
#include "forwardKinematics.h"
#include "interpolation.h"

#include "dyret_common/calculate_inverse_kinematics.h"
#include "dyret_common/Pose.h"


double dyn2rad(int angleInDynamixel) {
	int normalizedInput = fmod(angleInDynamixel,4095);
	if (normalizedInput < 0) normalizedInput = normalizedInput + 4095;

	double angleToReturn = (double) ((normalizedInput-2048.0) / 2048.0) * M_PI;
	return angleToReturn;
}

int rad2dyn(double angleInRad) {

	// Normalize to -2pi -> 2pi:
	double normalizedInput = (fmod(angleInRad+M_PI, 2*M_PI)-M_PI);
	if (normalizedInput < 0) normalizedInput = normalizedInput + 2*M_PI;
	int returnVariable = ((normalizedInput / (2 * M_PI)) * 4095.0) + (2048);
	return returnVariable;
}

double round(double originalNumber, int decimals) {
	double num = originalNumber * pow(10.0, (decimals));
	long long int rounded = (long long int)((double)num + 0.5);
	double numberToReturn = ((double)rounded) / pow(10.0, (decimals));
	return numberToReturn;
}

double rad2deg(double angleInRad) {
	return (angleInRad/(2.0*M_PI))*360.0;
}

double deg2rad(double angleInDeg) {
	return (angleInDeg / 360.0) * 2.0 * M_PI;
}

vec3P calculateLocalPosition(int givenLegId, vec3P givenLegGlobalPosition) {
	// Leg0/3: Lx = -Gx, Ly =  Gz, Lz = -Gy
	// Leg1/2: Lx =  Gx, Ly = -Gz, Lz =  Gy

	double x, y, z;

	// Calculate local X:
	if (givenLegId == 1 || givenLegId == 2) {
		x = givenLegGlobalPosition.x();
	}
	else {
		x = -givenLegGlobalPosition.x();
	}

	// Calculate local Y:
	if (givenLegId == 1 || givenLegId == 2) {
		y = -givenLegGlobalPosition.z();
	}
	else {
		y = givenLegGlobalPosition.z();
	}

	z = -givenLegGlobalPosition.y();

	vec3P calculatedLocalPosition = vec3P(x, y, z);
	calculatedLocalPosition.groundPoint = givenLegGlobalPosition.groundPoint;

	return calculatedLocalPosition;
}

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

	double calculatedPoint_x = amplitude_x/2 * tanh(3*sin( (2*M_PI*(time+(offset*period))) / period));
	double calculatedPoint_y = amplitude_y/2 * tanh(3*sin( (2*M_PI*(time+((offset-0.75)*(period/2)))) / (period/2)));

	vec3P pointToReturn = vec3P(calculatedPoint_x, calculatedPoint_y, 0.0f);
	pointToReturn.groundPoint = false;

	return pointToReturn;
}

vec3P calculateGlobalPosition(int givenLegId, vec3P givenLegLocalPosition) {
	double x, y, z;
	
	// Leg0: Gx = -Lx, Gy = -Lz, Gz = Ly
	// Leg2: Gx =  Lx, Gy =  Lz, Gz = Ly

	// Calculate global X:
	if (givenLegId == 1 || givenLegId == 2) {
		x = givenLegLocalPosition.x();
	}
	else {
		x = -givenLegLocalPosition.x();
	}

	y = -givenLegLocalPosition.z();


	// Calculate global Z:
	if (givenLegId == 0 || givenLegId == 3) {
		z = givenLegLocalPosition.y();
	}
	else {
		z = -givenLegLocalPosition.y();
	}

	vec3P calculatedGlobalPosition = vec3P(x,y,z);

	calculatedGlobalPosition.groundPoint = givenLegLocalPosition.groundPoint;

	return calculatedGlobalPosition;
}
