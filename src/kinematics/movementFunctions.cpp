#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "dyret_common/CalculateInverseKinematics.h"
#include "dyret_common/Pose.h"
#include "dyret_common/ServoConfig.h"
#include "dyret_common/ServoConfigArray.h"

#include "dyret_utils/angleConv.h"

#include "movementFunctions.h"
#include "interpolation.h"
#include "kinematicTypes.h"
#include "kinematicFunctions.h"
#include "forwardKinematics.h"
#include "dyret_utils/wait_for_ros.h"

bool legIsAtPos(int legId, vec3P givenGoalPosition, std::vector<double> servoAnglesInRad, double resolutionInMM, std::vector<double> legActuatorLengths){

  vec3P currentLocalLegPosition = forwardKinematics(servoAnglesInRad[3*legId], servoAnglesInRad[(3*legId)+1], servoAnglesInRad[(3*legId)+2], 0.0f, legActuatorLengths[0], legActuatorLengths[1]);
  vec3P currentGlobalLegPosistion = calculateGlobalPosition(legId, currentLocalLegPosition);

  for (int i = 0; i < 3; i++){
      if (fabs(givenGoalPosition.points[i] - currentGlobalLegPosistion.points[i]) > resolutionInMM){
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
  vec3P localPosition = forwardKinematics(servoAnglesInRad[3*legId], servoAnglesInRad[(3*legId)+1], servoAnglesInRad[(3*legId)+2], 0.0f, legActuatorLengths[0], legActuatorLengths[1]);
  vec3P globalPosition = calculateGlobalPosition(legId, localPosition);

  return globalPosition;
}

std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::vector<double> legLengths){
  std::vector<vec3P> vectorToReturn(4);

  for (int i = 0; i < 4; i++){
      vectorToReturn[i] = currentLegPos(i, servoAnglesInRad, legLengths);
  }

  return vectorToReturn;
}

std::vector<double> getInverseSolution(int legId, vec3P givenPoint, ros::ServiceClient givenInverseKinematicsServiceClient){
  dyret_common::CalculateInverseKinematics srv;

  if (givenInverseKinematicsServiceClient.exists() == false){
      printf("Inverse kinematics service not online!");
  }

  vec3P localLegPosition = calculateLocalPosition(legId, givenPoint);

  srv.request.point.x = localLegPosition.x();
  srv.request.point.y = localLegPosition.y();
  srv.request.point.z = localLegPosition.z();

  if (givenInverseKinematicsServiceClient.call(srv)) {
       // Handle invertions (ids 1, 5, 8, 10):
       if (legId == 0 || legId == 3){
           for (int j = 0; j < srv.response.solutions.size(); j++){
               srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
           }
       }else if (legId == 1 || legId == 2){
           for (int j = 0; j < srv.response.solutions.size(); j++){
               srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
           }
       }
   }

   std::vector<double> anglesInRad(3);

   if (srv.response.solutions.size() == 0){
       fprintf(stderr, "Could not find an inverse solution to point (%.2f, %.2f, %.2f)\n", givenPoint.points[0], givenPoint.points[1], givenPoint.points[2]);
   }

   // Successful in getting solution
   for (int j = 0; j < 3; j++){ // For each joint in the leg
       if (legId == 0 || legId == 1){
           anglesInRad[j] = normalizeRad(srv.response.solutions[1].anglesInRad[j]);
       }else{
           anglesInRad[j] = normalizeRad(srv.response.solutions[0].anglesInRad[j]);
       }
   }

   return anglesInRad;
}

void moveLegToGlobal(int givenLegId, vec3P givenPoint, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub){

  std::vector<int> servoIds {givenLegId*3, (givenLegId*3)+1, (givenLegId*3)+2};
  std::vector<double> anglesInRad(3);

  anglesInRad = getInverseSolution(givenLegId, givenPoint, givenInverseKinematicsServiceClient);

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

}

void moveAllLegsToGlobal(vec3P givenPoint, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub){

  std::vector<int> servoIds(12);
  for (int i = 0; i < 12; i++) servoIds[i] = i;

  std::vector<double> anglesInRad;

  for (int i = 0; i < 4; i++){
      std::vector<double> inverseReturn = getInverseSolution(i, givenPoint, givenInverseKinematicsServiceClient);
      anglesInRad.insert(anglesInRad.end(), inverseReturn.begin(), inverseReturn.end());
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

}

void moveAllLegsToGlobal(std::vector<vec3P> givenPoints, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub){

  std::vector<int> servoIds(12);
  for (int i = 0; i < 12; i++) servoIds[i] = i;

  std::vector<double> anglesInRad;

  for (int i = 0; i < 4; i++){
      std::vector<double> inverseReturn = getInverseSolution(i, givenPoints[i], givenInverseKinematicsServiceClient);
      anglesInRad.insert(anglesInRad.end(), inverseReturn.begin(), inverseReturn.end());
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  waitForRosInit(givenDynCommands_pub, "dynCommands");
  givenDynCommands_pub.publish(poseMsg);

}

// Open loop interpolation move of 4 legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions, std::vector<vec3P> givenStartPositions, float givenProgress, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub){

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
  std::vector<double> anglesInRad(12);

  for (int i = 0; i < 12; i++) servoIds[i] = i;

  for (int i = 0; i < 4; i++){
    dyret_common::CalculateInverseKinematics srv;

    vec3P globalLegPosition = lineInterpolation(givenStartPositions[i], givenGoalPositions[i], givenProgress);

    vec3P localLegPosition = calculateLocalPosition(i, globalLegPosition);

    srv.request.point.x = localLegPosition.x();
    srv.request.point.y = localLegPosition.y();
    srv.request.point.z = localLegPosition.z();
    if (givenInverseKinematicsServiceClient.call(srv)) {
      // Handle invertions (ids 1, 5, 8, 10):
      if (i == 0 || i == 3){
        for (int j = 0; j < srv.response.solutions.size(); j++){
          srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
        }
      }else if (i == 1 || i == 2){
        for (int j = 0; j < srv.response.solutions.size(); j++){
          srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
        }
      }
    }

    // Successful in getting solution
    for (int j = 0; j < 3; j++){ // For each joint in the leg
      if (i == 0 || i == 1){
        anglesInRad[currentServoId++] = srv.response.solutions[1].anglesInRad[j];
      }else{
        anglesInRad[currentServoId++] = srv.response.solutions[0].anglesInRad[j];
      }
    }
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

  return false;
}

// Closed loop interpolation move of four legs
bool interpolatingLegMoveClosedLoop(std::vector<vec3P> givenGoalPosition, double givenInterpolationIncrement, double givenGoalMargin, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths){

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
  std::vector<int> servoIds(12);
  std::vector<double> anglesInRad(12);

  for (int j = 0; j < 12; j++) servoIds[j] = j;

  for (int j = 0; j < 4; j++){
	  dyret_common::CalculateInverseKinematics srv;

    vec3P currentLegPosition = currentLegPos(j, servoAnglesInRad, legActuatorLengths);

    vec3P globalLegPosition = incInterpolation(currentLegPosition, givenGoalPosition[j], givenInterpolationIncrement); // moveDistance

    vec3P localLegPosition = calculateLocalPosition(j, globalLegPosition);

    srv.request.point.x = localLegPosition.x();
    srv.request.point.y = localLegPosition.y();
    srv.request.point.z = localLegPosition.z();
    if (givenInverseKinematicsServiceClient.call(srv)) {
      // Handle invertions (ids 1, 5, 8, 10):
      if (j == 0 || j == 3){
        for (int j = 0; j < srv.response.solutions.size(); j++){
          srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
        }
      }else if (j == 1 || j == 2){
        for (int j = 0; j < srv.response.solutions.size(); j++){
          srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
        }
      }
    }

    // Successful in getting solution
    for (int k = 0; k < 3; k++){ // For each joint in the leg
      if (j == 0 || j == 1){
        anglesInRad[currentServoId++] = srv.response.solutions[1].anglesInRad[k];
      }else{
        anglesInRad[currentServoId++] = srv.response.solutions[0].anglesInRad[k];
      }
    }
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

  return false;
}

// Closed loop interpolation move of 1 leg
bool interpolatingLegMoveClosedLoop(int givenLegId, vec3P givenGoalPosition, double givenInterpolationIncrement, double givenGoalMargin, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths){
  vec3P legPos = currentLegPos(givenLegId, servoAnglesInRad, legActuatorLengths);

  vec3P globalLegPosition = incInterpolation(legPos, givenGoalPosition, givenInterpolationIncrement);

  dyret_common::CalculateInverseKinematics srv;

  vec3P localLegPosition = calculateLocalPosition(givenLegId, globalLegPosition);

  srv.request.point.x = localLegPosition.x();
  srv.request.point.y = localLegPosition.y();
  srv.request.point.z = localLegPosition.z();

  if (givenInverseKinematicsServiceClient.call(srv)) {
  // Handle invertions (ids 1, 5, 8, 10):
      if (givenLegId == 0 || givenLegId == 3){
          for (int j = 0; j < srv.response.solutions.size(); j++){
              srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
          }
      }else if (givenLegId == 1 || givenLegId == 2){
          for (int j = 0; j < srv.response.solutions.size(); j++){
              srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
          }
      }
  }

  std::vector<int> servoIds(3);
  std::vector<double> anglesInRad(3);

  // Successful in getting solution
  for (int j = 0; j < 3; j++){ // For each joint in the leg
      servoIds[j] = j + (givenLegId * 3); // Set ids
      if (givenLegId == 0 || givenLegId == 1){
          anglesInRad[j] = srv.response.solutions[1].anglesInRad[j];
      }else{
          anglesInRad[j] = srv.response.solutions[0].anglesInRad[j];
      }
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

  return true;
}

// Open loop interpolation move of 1 leg
bool interpolatingLegMoveOpenLoop(int givenLegId, vec3P givenGoalPosition, vec3P givenStartPosition, double givenProgress, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub){

  if (givenProgress >= getInterpolatedLength(givenStartPosition, givenGoalPosition)){
    return true;
  }

  vec3P globalLegPosition = lineInterpolation(givenStartPosition, givenGoalPosition, givenProgress);

  dyret_common::CalculateInverseKinematics srv;

  vec3P localLegPosition = calculateLocalPosition(givenLegId, globalLegPosition);

  srv.request.point.x = localLegPosition.x();
  srv.request.point.y = localLegPosition.y();
  srv.request.point.z = localLegPosition.z();

  if (givenInverseKinematicsServiceClient.call(srv)) {
  // Handle invertions (ids 1, 5, 8, 10):
      if (givenLegId == 0 || givenLegId == 3){
          for (int j = 0; j < srv.response.solutions.size(); j++){
              srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
          }
      }else if (givenLegId == 1 || givenLegId == 2){
          for (int j = 0; j < srv.response.solutions.size(); j++){
              srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
          }
      }
  }

  std::vector<int> servoIds(3);
  std::vector<double> anglesInRad(3);

  // Successful in getting solution
  for (int j = 0; j < 3; j++){ // For each joint in the leg
      servoIds[j] = j + (givenLegId * 3); // Set ids
      if (givenLegId == 0 || givenLegId == 1){
          anglesInRad[j] = srv.response.solutions[1].anglesInRad[j];
      }else{
          anglesInRad[j] = srv.response.solutions[0].anglesInRad[j];
      }
  }

  dyret_common::Pose poseMsg;
  poseMsg.id = servoIds;
  poseMsg.angle = anglesInRad;
  givenDynCommands_pub.publish(poseMsg);

  return false;
}

void moveAllLegsToGlobalZ(double givenHeight, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths){
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

  moveAllLegsToGlobal(legPositions, givenInverseKinematicsServiceClient, givenDynCommands_pub);
}

void moveLegToGlobalZ(int givenLegId, double givenHeight, std::vector<double> servoAnglesInRad, ros::ServiceClient givenInverseKinematicsServiceClient, ros::Publisher givenDynCommands_pub, std::vector<double> legActuatorLengths){
  vec3P legPos = currentLegPos(givenLegId, servoAnglesInRad, legActuatorLengths);
  vec3P newLegPosition = legPos;
  newLegPosition.points[2] = givenHeight;

  moveLegToGlobal(givenLegId, newLegPosition, givenInverseKinematicsServiceClient, givenDynCommands_pub);
}

void setServoPIDs(std::vector<int> givenPIDs, ros::Publisher givenServoConfigPublisher){
	dyret_common::ServoConfigArray msg;
  std::vector<dyret_common::ServoConfig> msgContents(12);

  for (int i = 0; i < 12; i++){
      msgContents[i].servoId = i;
      msgContents[i].configType = dyret_common::ServoConfig::t_setPID;
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
  givenServoConfigPublisher.publish(msg);

}

// givenSpeed is a double 0->1
void setServoSpeeds(double givenSpeed, ros::Publisher givenServoConfigPublisher){
  dyret_common::ServoConfigArray msg;
  std::vector<dyret_common::ServoConfig> msgContents(12);

  for (int i = 0; i < 12; i++){
      msgContents[i].servoId = i;
      msgContents[i].configType = dyret_common::ServoConfig::t_setSpeed;
      msgContents[i].parameters.resize(1);
      msgContents[i].parameters[0] = givenSpeed;
  }

  msg.servoConfigs = msgContents;
  waitForRosInit(givenServoConfigPublisher, "servoConfigPublisher");
  givenServoConfigPublisher.publish(msg);

}

void setServoLog(bool enable, ros::Publisher givenServoConfigPublisher){
  dyret_common::ServoConfigArray msg;
  std::vector<dyret_common::ServoConfig> msgContents(1);

  if (enable == true){
      msgContents[0].configType = dyret_common::ServoConfig::t_enableLog;
  } else {
      msgContents[0].configType = dyret_common::ServoConfig::t_disableLog;
  }

  msg.servoConfigs = msgContents;
  givenServoConfigPublisher.publish(msg);
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
