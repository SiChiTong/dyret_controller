#include <vector>

#include "kinematicFunctions.h"

// Reading
vec3P currentLegPos(int legId, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths);
std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::vector<double> legLengths);


// Open loop interpolation move of all four legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions,
                                  std::vector<vec3P> givenStartPositions,
                                  float givenProgress,
                                  ros::ServiceClient* givenPositionCommandService);

void moveAllLegsToGlobalPosition(std::vector<vec3P> givenPoints, ros::ServiceClient* givenPositionCommandService);

// Config
bool setServoSpeeds(double givenSpeed, ros::ServiceClient givenServoConfigService);
bool setServoPIDs(std::vector<double> givenPIDs, ros::ServiceClient givenServoConfigService);

// Utility
vec3P lockToZ(vec3P givenPosition, double givenZValue);
std::vector<vec3P> lockToZ(std::vector<vec3P> givenPositions, double givenZValue);

vec3P doLegLengthCorrection(vec3P givenLegPosition, int givenLegIndex);