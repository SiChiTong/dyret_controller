#pragma once

#include <math.h>

#include "../external/splineLibrary/vector.h"

struct vec3P {
	Vector3 points;

	bool groundPoint = false;

	vec3P() {}

	vec3P(float givenX, float givenY, float givenZ) {
		points = Vector3({ givenX, givenY, givenZ });
	}

	vec3P(float givenX, float givenY, float givenZ, bool givenGroundContact) {
		points = Vector3({ givenX, givenY, givenZ });
		groundPoint = givenGroundContact;
	}

	double x() { return points[0]; };
	double y() { return points[1]; };
	double z() { return points[2]; };
};

struct vec2A {
	double angles[2];
	bool valid = true;
};

struct vec3A {
	double angles[3];
	bool valid;
	vec3A() { 
		valid = true; 
	}
	vec3A(double angle1, double angle2, double angle3, bool givenValid) {
		angles[0] = angle1;
		angles[1] = angle2;
		angles[2] = angle3;
		valid = givenValid;
	}
};

struct gaitLine {
	vec3P start;
	vec3P end;

	bool groundLine;

	double lineLength; // Line length in mm
	double minTime;    // Min time in s
	double maxSpeed;   // Max speed in mm/s

	gaitLine() {
		groundLine = false;
		maxSpeed = NAN;
	}
};

struct gaitSegment {
	vec3P segmentStart;
	vec3P segmentEnd;

	double segmentLength;
	double minTime;
	double maxSpeed;

	bool groundContact;

	gaitSegment() {
		groundContact = false;
	}
};