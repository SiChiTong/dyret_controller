#pragma once

#include "../kinematics/kinematicTypes.h"

class GaitSegment {


public: 
	
	vec3P start, end;

	bool groundSegment;

	float maxSpeed;
	float length;
	float minTime;

};