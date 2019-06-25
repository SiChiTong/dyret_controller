#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/SendPositionCommand.h"

#include "kinematics/inverseKinematics.h"

ros::Publisher poseCommand_pub;

std::vector<float> currentLegAngles(12);

std::array<double, 8> prismaticPosition;

void sendPositionCommand(std::vector<double> legPositions){
    std::vector<float> legAngles;

    for (int i = 0; i < 4; i++){
        // Get inverse kinematics solution for current leg
        std::vector<double> inverseKinematicsReturn = inverseKinematics::calculateInverseKinematics(legPositions[i*3],
                                                                                                    legPositions[(i*3)+1],
                                                                                                    legPositions[(i*3)+2],
                                                                                                    i,
                                                                                                    prismaticPosition[i*2],
                                                                                                    prismaticPosition[(i*2)+1]);
        legAngles.insert(legAngles.end(), inverseKinematicsReturn.begin(), inverseKinematicsReturn.end());

    }

    std::ostringstream rosStringStream;

    for (int i = 0; i < 4; i++){
        rosStringStream << "\n\tId " << i << ": " << legPositions[i*3] << ", " << legPositions[(i*3)+1] << ", " << legPositions[(i*3)+2];
    }

    ROS_INFO("%s\nCalculated angles:\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t",
             rosStringStream.str().c_str(),
             legAngles[0],  legAngles[1],  legAngles[2],
             legAngles[3],  legAngles[4],  legAngles[5],
             legAngles[6],  legAngles[7],  legAngles[8],
             legAngles[9], legAngles[10], legAngles[11]);

    dyret_common::Pose poseMsg;
    poseMsg.header.stamp = ros::Time().now();
    poseMsg.revolute = legAngles;
    poseCommand_pub.publish(poseMsg);
}

bool positionCommandServiceCallback(dyret_controller::SendPositionCommand::Request  &req,
                                    dyret_controller::SendPositionCommand::Response &res){

    std::vector<double> legPositions = {req.positionCommand.legPosition[0].x, req.positionCommand.legPosition[0].y, req.positionCommand.legPosition[0].z,
                                        req.positionCommand.legPosition[1].x, req.positionCommand.legPosition[1].y, req.positionCommand.legPosition[1].z,
                                        req.positionCommand.legPosition[2].x, req.positionCommand.legPosition[2].y, req.positionCommand.legPosition[2].z,
                                        req.positionCommand.legPosition[3].x, req.positionCommand.legPosition[3].y, req.positionCommand.legPosition[3].z};

    sendPositionCommand(legPositions);

    return true;

}

void stateCallback(const dyret_common::State::ConstPtr& msg){
  for (int i = 0; i < msg->prismatic.size(); i++){
      if (msg->prismatic[i].position > 0.0) prismaticPosition[i] = msg->prismatic[i].position;
      else prismaticPosition[i] = 0.0;
  }

  if (msg->revolute.size() == 12){
    for (int i = 0; i < 12; i++){
      currentLegAngles[i] = msg->revolute[i].position;
    }
  }

}

int main(int argc, char **argv){

  ros::init(argc, argv, "inverseKinematicsAdapter");
  ros::NodeHandle n;

  poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 1);

  ros::Subscriber state_sub = n.subscribe("/dyret/state", 1, stateCallback);
  ros::ServiceServer gaitEvalService = n.advertiseService("/dyret/dyret_controller/positionCommandService", positionCommandServiceCallback);

  ros::spin();

  ros::shutdown();

  return 0;
}