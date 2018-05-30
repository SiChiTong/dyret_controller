#include <vector>
#include <math.h>

#include "ros/ros.h"

#include "dyret_common/Pose.h"
#include "dyret_common/Configuration.h"
#include "dyret_common/angleConv.h"
#include "dyret_common/wait_for_ros.h"
#include "dyret_common/Configure.h"

#include "dyret_controller/PositionCommand.h"

#include "movementFunctions.h"
#include "interpolation.h"
#include "kinematicTypes.h"
#include "kinematicFunctions.h"
#include "forwardKinematics.h"
#include "inverseKinematics.h"

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

void moveAllLegsToGlobalPosition(std::vector<vec3P> givenPoints, ros::Publisher givenPositionCommand_pub){
  dyret_controller::PositionCommand msg;

  for (int i = 0; i < msg.legPosition.size(); i++){
    msg.legPosition[i].x = givenPoints[i].x();
    msg.legPosition[i].y = givenPoints[i].y();
    msg.legPosition[i].z = givenPoints[i].z();
  }

  givenPositionCommand_pub.publish(msg);
}

// Open loop interpolation move of 4 legs
bool interpolatingLegMoveOpenLoop(std::vector<vec3P> givenGoalPositions,
                                  std::vector<vec3P> givenStartPositions,
                                  float givenProgress,
                                  ros::Publisher givenPositionCommand_pub){

  dyret_controller::PositionCommand msg;

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

    msg.legPosition[i].x = legPosition.x();
    msg.legPosition[i].y = legPosition.y();
    msg.legPosition[i].z = legPosition.z();
  }

  givenPositionCommand_pub.publish(msg);

  return false;
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
