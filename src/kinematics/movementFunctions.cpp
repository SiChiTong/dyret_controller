#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "dyret_common/Pose.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/dimensions.h"
#include "dyret_common/angleConv.h"
#include "dyret_common/wait_for_ros.h"
#include "dyret_common/Configure.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/SendPositionCommand.h"

#include "movementFunctions.h"
#include "interpolation.h"
#include "kinematicTypes.h"
#include "kinematicFunctions.h"
#include "forwardKinematics.h"
#include "inverseKinematics.h"

vec3P doLegLengthCorrection(vec3P givenLegPosition, int givenLegIndex, std::array<double, 8> givenPrismaticCommands, std::array<double, 4> givenGroundHeights){
    // Find the angle between the legs
    double legLengthDifference = givenGroundHeights[0] - givenGroundHeights[2];
    double angle = atan2(legLengthDifference, frameLength);

    //fprintf(stderr, "Front: %.3f, Back: %.3f, diff: %.3f, angle: %.3f, groundHeight: %.3f\n", givenGroundHeights[0], givenGroundHeights[2], legLengthDifference, angle, givenGroundHeights[givenLegIndex]);

    // Translate point to origin
    double y = givenLegPosition.y();
    double z = givenLegPosition.z();
    double z_new = z - givenGroundHeights[givenLegIndex];

    //fprintf(stderr, "Original: (%.2f, %.2f) => Translated (%.2f, %.2f)\n", y, z, y, z_new);

    // Rotate point
    double y_m = y * cos(angle) - z_new * sin(angle);
    double z_m = z_new * cos(angle) + y * sin(angle);

    // Translate back
    double z_m_new = z_m + givenGroundHeights[givenLegIndex];

    //fprintf(stderr, "Rotated: (%.2f, %.2f) => Translated (%.2f, %.2f)\n\n", y_m, z_m, y_m, z_m_new);

    vec3P rotatedLegPosition = vec3P(givenLegPosition.x(), y_m, z_m_new);

    return rotatedLegPosition;
}

vec3P currentLegPos(int legId, std::vector<double> servoAnglesInRad, std::array<double, 8> legLengths){
  vec3P position = forwardKinematics(servoAnglesInRad[3*legId], servoAnglesInRad[(3*legId)+1], servoAnglesInRad[(3*legId)+2], 0.0f, legLengths[2*legId], legLengths[(2*legId)+1]);

  return position;
}

std::vector<vec3P> currentLegPositions(std::vector<double> servoAnglesInRad, std::array<double, 8> legLengths){
  std::vector<vec3P> vectorToReturn(4);

  for (int i = 0; i < 4; i++){
      vectorToReturn[i] = currentLegPos(i, servoAnglesInRad, legLengths);
  }

  return vectorToReturn;
}

void moveAllLegsToGlobalPosition(std::vector<vec3P> givenPoints, ros::ServiceClient* givenPositionCommandService){
  dyret_controller::SendPositionCommand srv;

  for (int i = 0; i < 4; i++){
    srv.request.positionCommand.legPosition[i].x = givenPoints[i].x();
    srv.request.positionCommand.legPosition[i].y = givenPoints[i].y();
    srv.request.positionCommand.legPosition[i].z = givenPoints[i].z();
  }

  givenPositionCommandService->call(srv);
}

// Open loop interpolation move of 4 legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions,
                                  std::vector<vec3P> givenStartPositions,
                                  float givenProgress,
                                  ros::ServiceClient* givenPositionCommandService){

  dyret_controller::SendPositionCommand srv;

  // Check to see which legs have reached their positions
  bool reachedPositions = true;
  for (int i = 0; i < 4; i++){
      if (givenProgress <= getInterpolatedLength(givenStartPositions[i], givenGoalPositions[i])){
          reachedPositions = false;
          break; // break with reachedPositions = false
      }
  }

  if (reachedPositions == true) return true; // All legs have reached their positions


  for (int i = 0; i < 4; i++){
    vec3P legPosition = lineInterpolation(givenStartPositions[i], givenGoalPositions[i], givenProgress);

    srv.request.positionCommand.legPosition[i].x = legPosition.x();
    srv.request.positionCommand.legPosition[i].y = legPosition.y();
    srv.request.positionCommand.legPosition[i].z = legPosition.z();
  }

  if (givenPositionCommandService->call(srv) == false){
    ROS_ERROR("Error while calling positionCommandService");
  }

  return false;
}

bool callConfigurationService(dyret_common::Configure givenCall, ros::ServiceClient givenServoConfigService){
  if (givenServoConfigService.call(givenCall))  {
    switch(givenCall.response.status){
      case dyret_common::Configure::Response::STATUS_NOERROR:
        ROS_INFO("Configuration service returned no error");
        break;
      case dyret_common::Configure::Response::STATUS_STATE:
        ROS_ERROR("State error from configuration service response");
        break;
      case dyret_common::Configure::Response::STATUS_PARAMETER:
        ROS_ERROR("Parameter error from configuration service response");
        break;
      default:
        ROS_ERROR("Unknown error from configuration service response");
        break;
    }

    if (givenCall.response.status == givenCall.response.STATUS_NOERROR) return true;

  } else {
    ROS_ERROR("Failed to call configuration service");
    return false;
  }

}

bool setServoPIDs(std::vector<double> givenPIDs, ros::ServiceClient givenConfigurationService){
	dyret_common::Configure msg;

  msg.request.configuration.revolute.ids = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // Set all servos
  msg.request.configuration.revolute.type = msg.request.configuration.revolute.TYPE_SET_PID;

  for (int i = 0; i < 12; i++){

      if (i == 0 || i == 3 || i == 6 || i == 9){
        // Coxa
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[0]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[1]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[2]);
      } else if (i == 1 || i == 4 || i == 7 || i == 10){
        // Femur
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[3]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[4]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[5]);
      } else {
        // Tibia
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[6]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[7]);
        msg.request.configuration.revolute.parameters.push_back(givenPIDs[8]);
      }
  }

  callConfigurationService(msg, givenConfigurationService);

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

  return callConfigurationService(srv, givenServoConfigService);
}

vec3P lockToZ(vec3P givenPosition, double givenZValue){
  return vec3P(givenPosition.points[0], givenPosition.points[1], givenZValue);
}

std::vector<vec3P> lockToZ(std::vector<vec3P> givenPositions, std::array<double, 4> givenZValues){
  std::vector<vec3P> pointsToReturn(givenPositions.size());

  for (int i = 0; i < givenPositions.size(); i++){
      pointsToReturn[i] = lockToZ(givenPositions[i], givenZValues[i]);
  }

  return pointsToReturn;
}
