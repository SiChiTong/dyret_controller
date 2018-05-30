#ifndef DYRET_CONTROLLER_INVERSEKINEMATICS_H
#define DYRET_CONTROLLER_INVERSEKINEMATICS_H

#include <vector>

namespace inverseKinematics {

std::vector<double> calculateInverseKinematics(double x, double y, double z, int legId, double femurActuatorLength, double tibiaActuatorLength);

}
#endif
