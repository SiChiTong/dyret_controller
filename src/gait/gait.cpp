#include <stdio.h>

#include "../kinematics/kinematicFunctions.h"
#include "../kinematics/kinematicTypes.h"
#include "gait.h"

void Gait::markGroundSegments() {
	// Find bottom plane:
	float minPos = INFINITY;

	for (int i = 0; i < gaitPointVector.size(); i++) {
		if (gaitPointVector[i].z() < minPos) minPos = gaitPointVector[i].z();
	}

	for (int i = 0; i < gaitSegments.size(); i++) {
		if (gaitSegments[i].start.z() == minPos && gaitSegments[i].end.z() == minPos) {
			gaitSegments[i].groundSegment = true;
		}
	}

}

void Gait::enableWag(float givenWagPhase, float givenWagAmplitude_x, float givenWagAmplitude_y) {
	wagPhase       = givenWagPhase;
	wagAmplitude_x = givenWagAmplitude_x;
	wagAmplitude_y = givenWagAmplitude_y;

}

