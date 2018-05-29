#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "dyret_common/Pose.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/angleConv.h"
#include "dyret_common/wait_for_ros.h"
#include "dyret_common/Configure.h"

#include "movementFunctions.h"
#include "interpolation.h"
#include "kinematicTypes.h"
#include "kinematicFunctions.h"
#include "forwardKinematics.h"
#include "inverseKinematics.h"

bool legIsAtPos(int legId, vec3P givenGoalPosition, std::vector<double> servoAnglesInRad, double resolutionInMM, std::vector<double> legActuatorLengths){

  vec3P currentGlobalLegPosition = forwardKinematics(servoAnglesInRad[3*legId], servoAnglesInRad[(3*legId)+1], servoAnglesInRad[(3*legId)+2], 0.0f, legActuatorLengths[0], legActuatorLengths[1]);

  for (int i = 0; i < 3; i++){
      if (fabs(givenGoalPosition.points[i] - currentGlobalLegPosition.points[i]) > resolutionInMM){
          return false;
      }
  }

  return true;
}

bool legIsAtPos(int legId, vec3P givenGoalPosition, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths){
  return legIsAtPos(legId, givenGoalPosition, servoAnglesInRad, 7.5, legActuatorLengths); // 7.5mm default
}

bool legIsAtHeight(int legId, double goalHeight, std::vector<double> servoAnglesInRad, double marginInMM, std::vector<double> legActuatorLengths){
  if (fabs(currentLegPos(legId, servoAnglesInRad,  legActuatorLengths).points[2] - goalHeight) > 5.0){ // Distance
     return false;
  }
  return true;
}

bool legsAreAtHeight(double goalHeight, std::vector<double> servoAnglesInRad, double marginInMM, std::vector<double> legActuatorLengths){

  for (int i = 0; i < 4; i++){
      if (legIsAtHeight(i, goalHeight, servoAnglesInRad, marginInMM, legActuatorLengths) == false){
          return false;
      }
  }

  return true;
}

vec3P currentLegPos(int legId, std::vector<double> servoAnglesInRad, std::vector<double> legActuatorLengths){
  vec3P position = forwardKinematics(servoAnglesInRad[3*legId], servoAnglesInRad[(3*legId)+1], servoAnglesInRad[(3*legId)+2], 0.0f, legActuatorLengths[0], legActuatorLengths[1]);

  return position;
}

std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::vector<double> legLengths){
  std::vector<vec3P> vectorToReturn(4);

  for (int i = 0; i < 4; i++){
      vectorToReturn[i] = currentLegPos(i, servoAnglesInRad, legLengths);
  }

  return vectorToReturn;
}

std::vector<double> getInverseSolution(int legId, vec3P givenPoint, double femurActuatorLength, double tibiaActuatorLength){

  std::vector<double> angleInRad = inverseKinematics::calculateInverseKinematics(givenPoint.x(), givenPoint.y(), givenPoint.z(), legId, femurActuatorLength, tibiaActuatorLength);

  return angleInRad;
}

void moveAllLegsToGlobal(vec3P givenPoint, double femurActuatorLength, double tibiaActuatorLength, ros::Publisher givenPoseCommand_pub){

  std::vector<int> servoIds(12);
  for (int i = 0; i < 12; i++) servoIds[i] = i;

  std::vector<float> anglesInRad;

  for (int i = 0; i < 4; i++){
      std::vector<double> inverseReturn = getInverseSolution(i, givenPoint, femurActuatorLength, tibiaActuatorLength);
      anglesInRad.insert(anglesInRad.end(), inverseReturn.begin(), inverseReturn.end());
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = anglesInRad;
  givenPoseCommand_pub.publish(poseMsg);

}

void moveAllLegsToGlobal(std::vector<vec3P> givenPoints, float femurActuatorLength, float tibiaActuatorLength, ros::Publisher givenPoseCommand_pub){

  std::vector<int> servoIds(12);
  for (int i = 0; i < 12; i++) servoIds[i] = i;

  std::vector<float> anglesInRad;

  for (int i = 0; i < 4; i++){
    std::vector<double> inverseReturn = inverseKinematics::calculateInverseKinematics(givenPoints[i].x(), givenPoints[i].y(), givenPoints[i].z(),i, femurActuatorLength, tibiaActuatorLength);
    fprintf(stderr, "%.2f, %.2f, %.2f\n", inverseReturn[0], inverseReturn[1], inverseReturn[2]);
    anglesInRad.insert(anglesInRad.end(), inverseReturn.begin(), inverseReturn.end());
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = anglesInRad;
  waitForRosInit(givenPoseCommand_pub, "pose_command");
  givenPoseCommand_pub.publish(poseMsg);

}

// Open loop interpolation move of 4 legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions,
                                  std::vector<vec3P> givenStartPositions,
                                  float givenProgress,
                                  float givenFemurLength,
                                  float givenTibiaLength,
                                  ros::Publisher givenPoseCommand_pub){

  // Check to see which legs have reached their positions
  bool reachedPositions = true;
  for (int i = 0; i < 4; i++){
      if (givenProgress <= getInterpolatedLength(givenStartPositions[i], givenGoalPositions[i])){
          reachedPositions = false;
          break; // break with reachedPositions = false
      }
  }

  if (reachedPositions == true) return true; // All legs have reached their positions

  int currentServoId = 0;
  std::vector<int> servoIds(12);
  std::vector<float> anglesInRad(12);

  for (int i = 0; i < 12; i++) servoIds[i] = i;

  for (int i = 0; i < 4; i++){
    vec3P globalLegPosition = lineInterpolation(givenStartPositions[i], givenGoalPositions[i], givenProgress);

    std::vector<double> inverseSolution = getInverseSolution(i, globalLegPosition, givenFemurLength, givenTibiaLength);

    for (int j = 0; j < 3; j++){ // For each joint in the leg
      anglesInRad[currentServoId++] = inverseSolution[j];
    }
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = anglesInRad;
  givenPoseCommand_pub.publish(poseMsg);

  return false;
}

// Closed loop interpolation move of four legs
bool interpolatingLegMoveClosedLoop(std::vector<vec3P> givenGoalPosition,
                                    double givenInterpolationIncrement,
                                    double givenGoalMargin,
                                    std::vector<double> servoAnglesInRad,
                                    float givenFemurLength,
                                    float givenTibiaLength,
                                    ros::Publisher givenPoseCommand_pub,
                                    std::vector<double> legActuatorLengths){

  // Check if we have arrived:
  bool finishedMoving = true;

  for (int j = 0; j < 4; j++){
      if (legIsAtPos(j, givenGoalPosition[j], servoAnglesInRad, givenGoalMargin, legActuatorLengths) == false){ // measureDistance
          finishedMoving = false;
          break;
      }
  }
  if (finishedMoving == true) return true;

  // Do interpolation:
  int currentServoId = 0;
  std::vector<float> anglesInRad(12);

  for (int j = 0; j < 4; j++){
    vec3P currentLegPosition = currentLegPos(j, servoAnglesInRad, legActuatorLengths);

    vec3P legPosition = incInterpolation(currentLegPosition, givenGoalPosition[j], givenInterpolationIncrement); // moveDistance

    for (int i = 0; i < 4; i++) {
      std::vector<double> inverseSolution = getInverseSolution(i, legPosition, givenFemurLength, givenTibiaLength);

      for (int j = 0; j < 3; j++) { // For each joint in the leg
        anglesInRad[currentServoId++] = inverseSolution[j];
      }
    }
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = anglesInRad;
  givenPoseCommand_pub.publish(poseMsg);

  return false;
}

// Open loop interpolation move of 1 leg
bool interpolatingLegMoveOpenLoop(int givenLegId,
                                  vec3P givenGoalPosition,
                                  vec3P givenStartPosition,
                                  double givenProgress,
                                  std::vector<double> servoAnglesInRad,
                                  float givenFemurLength,
                                  float givenTibiaLength,
                                  ros::Publisher givenPoseCommand_pub){

  if (givenProgress >= getInterpolatedLength(givenStartPosition, givenGoalPosition)){
    return true;
  }

  vec3P legPosition = lineInterpolation(givenStartPosition, givenGoalPosition, (float) givenProgress);

  std::vector<double> inverseSolution = getInverseSolution(givenLegId, legPosition, givenFemurLength, givenTibiaLength);

  std::vector<float> anglesInRad(12);

  for (int i = 0; i < anglesInRad.size(); i++) anglesInRad[i] = float(servoAnglesInRad[i]);

  for (int j = 0; j < 3; j++){ // For each joint in the leg
    anglesInRad[j + (givenLegId * 3.0f)] = inverseSolution[j];
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = anglesInRad;
  givenPoseCommand_pub.publish(poseMsg);

  return false;
}

void moveAllLegsToGlobalZ(double givenHeight,
                          std::vector<double> servoAnglesInRad,
                          float femurActuatorLength,
                          float tibiaActuatorLength,
                          ros::Publisher givenPoseCommand_pub,
                          std::vector<double> legActuatorLengths){
  std::vector<vec3P> legPositions(4);

  bool nonZero = false;
  for (int i = 0; i < servoAnglesInRad.size(); i++){
      if (servoAnglesInRad[i] != 0.0) nonZero = true;
  }

  if (nonZero == false) printf("All servo angles are zero!\n");

  for (int i = 0; i < 4; i++){
      legPositions[i] = currentLegPos(i, servoAnglesInRad, legActuatorLengths);
      legPositions[i].points[2] = givenHeight;
  }

  moveAllLegsToGlobal(legPositions, femurActuatorLength, tibiaActuatorLength, givenPoseCommand_pub);
}

bool setServoPIDs(std::vector<int> givenPIDs, ros::ServiceClient givenServoConfigService){
	/*dyret_common::ServoConfigArray msg;
  std::vector<dyret_common::ServoConfig> msgContents(12);

  for (int i = 0; i < 12; i++){
      msgContents[i].type = dyret_common::ServoConfig::TYPE_SET_PID;
      msgContents[i].parameters.resize(3);

      if (i == 0 || i == 3 || i == 6 || i == 9){
        // Coxa
        msgContents[i].parameters[0] = (double) givenPIDs[0];
        msgContents[i].parameters[1] = (double) givenPIDs[1];
        msgContents[i].parameters[2] = (double) givenPIDs[2];
      } else if (i == 1 || i == 4 || i == 7 || i == 10){
        // Femur
        msgContents[i].parameters[0] = (double) givenPIDs[3];
        msgContents[i].parameters[1] = (double) givenPIDs[4];
        msgContents[i].parameters[2] = (double) givenPIDs[5];
      } else {
        // Tibia
        msgContents[i].parameters[0] = (double) givenPIDs[6];
        msgContents[i].parameters[1] = (double) givenPIDs[7];
        msgContents[i].parameters[2] = (double) givenPIDs[8];
      }
  }

  msg.servoConfigs = msgContents;
  givenServoConfigPublisher.publish(msg);*/

	ROS_ERROR("Setting PID not yet implemented");

}

bool callServoConfigService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService){
  if (givenServoConfigService.call(givenCall))  {
    switch(givenCall.response.status){
      case dyret_common::Configure::Response::STATUS_NOERROR:
        ROS_INFO("Configure servo service returned no error");
        break;
      case dyret_common::Configure::Response::STATUS_STATE:
        ROS_ERROR("State error from configure servo response");
        break;
      case dyret_common::Configure::Response::STATUS_PARAMETER:
        ROS_ERROR("Parameter error from configure servo response");
        break;
      default:
        ROS_ERROR("Unknown error from configure servo response");
        break;
    }

    if (givenCall.response.status == givenCall.response.STATUS_NOERROR) return true;

  } else {
    ROS_ERROR("Failed to call servo config service");
    return false;
  }

}

// givenSpeed is a double 0->1
bool setServoSpeeds(double givenSpeed, ros::ServiceClient givenServoConfigService){
  dyret_common::Configure srv;

  srv.request.configuration.revolute.type = dyret_common::RevoluteConfig::TYPE_SET_SPEED;
  srv.request.configuration.revolute.ids.resize(12);
  srv.request.configuration.revolute.parameters.resize(12);

  for (int i = 0; i < 12; i++){
    srv.request.configuration.revolute.ids[i] = i;
    srv.request.configuration.revolute.parameters[i] = givenSpeed;
  }

  return callServoConfigService(srv, givenServoConfigService);

}

bool setServoLog(bool enable, ros::ServiceClient givenServoConfigService){
  dyret_common::Configure srv;

  ROS_ERROR("Set servo log not implemented!");

  return callServoConfigService(srv, givenServoConfigService);
}

vec3P lockToZ(vec3P givenPosition, double givenZValue){
  return vec3P(givenPosition.points[0], givenPosition.points[1], givenZValue);
}

std::vector<vec3P> lockToZ(std::vector<vec3P> givenPositions, double givenZValue){
  std::vector<vec3P> pointsToReturn(givenPositions.size());

  for (int i = 0; i < givenPositions.size(); i++){
      pointsToReturn[i] = lockToZ(givenPositions[i], givenZValue);
  }

  return pointsToReturn;
}
