#include <vector>

#include "kinematicFunctions.h"

// Reading
vec3P currentLegPos(int legId, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths);
std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::vector<double> legLengths);


// Open loop interpolation move of all four legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions,
                                  std::vector<vec3P> givenStartPositions,
                                  float givenProgress,
                                  ros::Publisher givenPositionCommand_pub);

void moveAllLegsToGlobalPosition(std::vector<vec3P> givenPoints, ros::Publisher givenPositionCommand_pub);

// Config
bool setServoSpeeds(double givenSpeed, ros::ServiceClient givenServoConfigService);
bool setServoPIDs(std::vector<int> givenPIDs, ros::ServiceClient givenServoConfigService);
bool setServoLog(bool enable, ros::ServiceClient givenServoConfigService);

// Utility
vec3P lockToZ(vec3P givenPosition, double givenZValue);
std::vector<vec3P> lockToZ(std::vector<vec3P> givenPositions, double givenZValue);
