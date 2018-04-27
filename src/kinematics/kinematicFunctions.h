#pragma once

#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "kinematicTypes.h"

double round(double originalNumber, int decimals);
//vec3P calculateLocalPosition(int givenLegId, vec3P givenLegGlobalPosition);
//vec3P calculateGlobalPosition(int givenLegId, vec3P givenLegLocalPosition);
vec3P getWagPoint(double amplitude_x, double amplitude_y, double period, double time, double offset);
vec3P add(vec3P firstOperand, vec3P secondOperand);
std::vector<vec3P> add(std::vector<vec3P> firstOperand, vec3P secondOperand);

