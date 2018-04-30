#include <math.h> 
#include <stdio.h> 
#include "dyret_common/dimensions.h"
#include "kinematicTypes.h"

vec3P forwardKinematics(double t1, double t2, double t3, double r3, double femurActuatorLength, double tibiaActuatorLength){

  double x = -(L01 * sin(t1) + (L12 + femurActuatorLength) * (cos(t2)*sin(t1)) + cos(t3)*(cos(t2)*sin(t1))*(r3 + (L23 + tibiaActuatorLength)) + sin(t3)*(-sin(t1)*sin(t2))*(r3 + (L23 + tibiaActuatorLength)));
  double y = (L12 + femurActuatorLength) * sin(t2) + cos(t2)*sin(t3)*(r3 + (L23 + tibiaActuatorLength)) + cos(t3)*sin(t2)*(r3 + (L23 + tibiaActuatorLength));
  double z = -(L01 * cos(t1) + (L12 + femurActuatorLength) * (cos(t1)*cos(t2)) + cos(t3)*(cos(t1)*cos(t2))*(r3 + (L23 + tibiaActuatorLength)) - sin(t3)*(cos(t1)*sin(t2))*(r3 + (L23 + tibiaActuatorLength)));

  return vec3P(x, y, z);

}
