#include <stdio.h>

#include "../kinematics/kinematicFunctions.h"
#include "../kinematics/kinematicTypes.h"
#include "gait.h"

void Gait::enableWag(float givenWagPhase, float givenWagAmplitude_x, float givenWagAmplitude_y) {
	wagPhase       = givenWagPhase;
	wagAmplitude_x = givenWagAmplitude_x;
	wagAmplitude_y = givenWagAmplitude_y;

}

