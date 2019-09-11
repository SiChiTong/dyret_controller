
#include "inverseKinematics.h"

#include <math.h>
#include <vector>

#include <ros/console.h>

#include "dyret_common/dimensions.h"

#include "dyret_common/angleConv.h"

namespace inverseKinematics {

double femurActuatorLength, tibiaActuatorLength;

struct vec2A {
  double angles[2];
};

vec2A calculateT2(double x, double p1_x, double y, double p1_y, double z, double L12, double L13, double L23) {
  vec2A vectorToReturn;

  double L13xy = sqrt(pow((x - p1_x), 2) + pow((y - p1_y), 2));
  double L13z = z;

  double t2_bot = atan(L13z / L13xy);
  double t2_tot = acos((pow(L12, 2) + pow(L13, 2) - pow(L23, 2)) / (2 * L12 * L13));

  if ((fabs(x) < fabs(p1_x)) || (x < 0 && p1_x > 0) || (y < 0 && p1_y > 0) ||
      (y > 0 && p1_y > 0 && (fabs(y) < fabs(p1_y))) || (x > 0 && p1_x < 0 && (fabs(x) >= fabs(p1_x)) && y >= 0)) {
    vectorToReturn.angles[1] = M_PI - (t2_bot - t2_tot);
    vectorToReturn.angles[0] = M_PI - (t2_tot + t2_bot);
  } else {
    vectorToReturn.angles[0] = t2_bot - t2_tot;
    vectorToReturn.angles[1] = t2_tot + t2_bot;
  }

  return vectorToReturn;
}


double roundNum(double originalNumber, int decimals) {
  double num = originalNumber * pow(10.0, (decimals));
  long long int rounded = (long long int) ((double) num + 0.5);
  double numberToReturn = ((double) rounded) / pow(10.0, (decimals));
  return numberToReturn;
}

std::vector<double> calculateInverseKinematics(double x, double y, double z, int legId, double femurActuatorLength, double tibiaActuatorLength) {

  if (isnan(x) || isnan(y) || isnan(z)){
      ROS_ERROR("Inversekinematics: isnan true for (%.2f, %.2f, %.2f)", x, y, z);
  }

  if (femurActuatorLength < 0.0) femurActuatorLength = 0.0;
  if (tibiaActuatorLength < 0.0) tibiaActuatorLength = 0.0;

  std::vector<std::vector<double>> solutions(4);
  for (int i = 0; i < solutions.size(); i++) solutions[i].resize(3);

  double t1_1 = atan2(z, x);
  double t1_2 = M_PI + t1_1;

  double L12_corrected = L12 + femurActuatorLength;
  double L23_corrected = L23 + tibiaActuatorLength;

  solutions[0][0] = t1_1;
  solutions[1][0] = t1_1;
  solutions[2][0] = t1_2;
  solutions[3][0] = t1_2;

  double p1_x_1 = roundNum(cos(t1_1) * 72.0, 10);
  double p1_y_1 = roundNum(sin(t1_1) * 72.0, 10);
  double p1_z_1 = 0.0;

  double p1_x_2 = roundNum(cos(t1_2) * 72.0, 10);
  double p1_y_2 = roundNum(sin(t1_2) * 72.0, 10);
  double p1_z_2 = 0.0;

  double L13_1 = sqrt(pow((x - p1_x_1), 2) + pow((z - p1_y_1), 2) + pow((-y - p1_z_1), 2));
  double L13_2 = sqrt(pow((x - p1_x_2), 2) + pow((z - p1_y_2), 2) + pow((-y - p1_z_2), 2));

  // Calculate first two solutions
  solutions[0][2] = M_PI - acos((pow(L12_corrected, 2) + pow(L23_corrected, 2) - pow(L13_1, 2)) / (2 * L12_corrected * L23_corrected));
  solutions[1][2] = (2 * M_PI) - solutions[0][2];

  vec2A t2Angles = calculateT2(x, p1_x_1, z, p1_y_1, -y, L12_corrected, L13_1, L23_corrected);
  solutions[0][1] = t2Angles.angles[0];
  solutions[1][1] = t2Angles.angles[1];

  // Calculate second two solutions
  solutions[2][2] = M_PI - acos((pow(L12_corrected, 2) + pow(L23_corrected, 2) - pow(L13_2, 2)) / (2 * L12_corrected * L23_corrected));
  solutions[3][2] = (2 * M_PI) - solutions[2][2];

  t2Angles = calculateT2(x, p1_x_2, z, p1_y_2, -y, L12_corrected, L13_2, L23_corrected);
  solutions[2][1] = t2Angles.angles[0];
  solutions[3][1] = t2Angles.angles[1];

  // Delete invalid solutions
  if ((L13_1 > L12_corrected + L23_corrected) || (L13_1 + L12_corrected < L23_corrected)) {
    // Delete first two solutions
    ROS_ERROR("inverseKinematics: No valid inverse kinematics solutions for (%.2f, %.2f, %.2f)", x, y, z);
    return std::vector<double>();
    solutions.erase(solutions.begin());
    solutions.erase(solutions.begin());
  }

  solutions.erase(solutions.end());
  solutions.erase(solutions.end());

  for (int i = 0; i < solutions.size(); i++) {
    for (int j = 0; j < 3; j++) {
      solutions[i][j] = normalizeRad(solutions[i][j]);
    }
    solutions[i][0] = -(solutions[i][0] + M_PI / 2.0);
  }

  std::vector<double> solutionToReturn(3);

  for (int j = 0; j < 3; j++) { // For each joint in the leg
    if (legId == 0 || legId == 1) {
      solutionToReturn[j] = normalizeRad(solutions[1][j]);
    } else {
      solutionToReturn[j] = normalizeRad(solutions[0][j]);
    }
  }

  return solutionToReturn;
}

}