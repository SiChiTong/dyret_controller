#pragma once

#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "kinematicTypes.h"

vec3P getWagPoint(double amplitude_x, double amplitude_y, double period, double time, double offset);
vec3P add(vec3P firstOperand, vec3P secondOperand);
std::vector<vec3P> add(std::vector<vec3P> firstOperand, vec3P secondOperand);
void printPose(std::vector<vec3P>);

