#pragma once

#include "kinematicTypes.h"

float getInterpolatedLength(vec3P startPosition, vec3P goalPosition);
float calculateIncrement(vec3P startPosition, vec3P goalPosition, int numberOfIncrements);
vec3P incInterpolation(vec3P currentPosition, vec3P goalPosition, double increment);
vec3P lineInterpolation(vec3P startPosition, vec3P goalPosition, float distance);
