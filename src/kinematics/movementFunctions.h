#include <vector>

#include "kinematicFunctions.h"

// Reading
bool legIsAtPos(int legId, vec3P givenGoalPosition, std::vector<double> servoAnglesInRad, double resolutionInMM, std::vector<double> legActuatorLengths);
bool legIsAtPos(int legId, vec3P givenGoalPosition, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths);
bool legIsAtHeight(int legId, double goalHeight, std::vector<double> servoAnglesInRad, double marginInMM,  std::vector<double> legActuatorLengths);
bool legsAreAtHeight(double goalHeight, std::vector<double> servoAnglesInRad, double marginInMM, std::vector<double> legActuatorLengths);
vec3P currentLegPos(int legId, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths);
std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::vector<double> legLengths);


// Writing
void moveLegToGlobal(int legId, vec3P givenPoint, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub);
void moveAllLegsToGlobal(vec3P givenPoint, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub);
void moveAllLegsToGlobal(std::vector<vec3P> givenPoints, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub);
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions, std::vector<vec3P> givenStartPositions, float givenProgress, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub);
bool interpolatingLegMoveClosedLoop(std::vector<vec3P> givenGoalPosition, double givenInterpolationIncrement, double givenGoalMargin, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths);
bool interpolatingLegMoveOpenLoop(int givenLegId, vec3P givenGoalPosition, vec3P givenStartPosition, double givenProgress, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub);
bool interpolatingLegMoveClosedLoop(int givenLegId, vec3P givenGoalPosition, double givenInterpolationIncrement, double givenGoalMargin, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths);
void moveLegToGlobalZ(int givenLegId, double givenHeight, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths);
void moveAllLegsToGlobalZ(double givenHeight, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths);

// Config
void setServoSpeeds(double givenSpeed, ros::Publisher givenServoConfigPublisher);
void setServoPIDs(std::vector<int> givenPIDs, ros::Publisher givenServoConfigPublisher);
void setServoTorques(int givenTorque, ros::Publisher givenServoConfigPublisher);
void setServoLog(bool enable, ros::Publisher givenServoConfigPublisher);

// Utility
vec3P lockToZ(vec3P givenPosition, double givenZValue);
std::vector<vec3P> lockToZ(std::vector<vec3P> givenPositions, double givenZValue);
