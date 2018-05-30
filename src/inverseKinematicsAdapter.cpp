#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/PositionCommand.h"
#include "dyret_common/State.h"

#include "kinematics/inverseKinematics.h"

ros::Publisher poseCommand_pub;

double femurActuatorLength = 0.0;
double tibiaActuatorLength = 0.0;

void positionCommandCallback(const dyret_common::PositionCommand::ConstPtr& msg){
  std::vector<float> legAngles;

  for (int i = 0; i < 4; i++){
    // Get inverse kinematics solution for current leg
    std::vector<double> inverseKinematicsReturn = inverseKinematics::calculateInverseKinematics(msg->legPosition[i].x,
                                                                                                msg->legPosition[i].y,
                                                                                                msg->legPosition[i].z,
                                                                                                i,
                                                                                                femurActuatorLength,
                                                                                                tibiaActuatorLength);

    // Insert into solution array
    legAngles.insert(legAngles.end(), inverseKinematicsReturn.begin(), inverseKinematicsReturn.end());
  }

  dyret_common::Pose poseMsg;
  poseMsg.revolute = legAngles;
  poseCommand_pub.publish(poseMsg);

}

void stateCallback(const dyret_common::State::ConstPtr& msg){
  if (msg->prismatic.size() == 8){
    femurActuatorLength = (msg->prismatic[0].position + msg->prismatic[2].position + msg->prismatic[4].position + msg->prismatic[6].position) / 4.0f;
    tibiaActuatorLength = (msg->prismatic[1].position + msg->prismatic[3].position + msg->prismatic[5].position + msg->prismatic[7].position) / 4.0f;
  }

  if (femurActuatorLength < 0.0) femurActuatorLength = 0.0;
  if (tibiaActuatorLength < 0.0) tibiaActuatorLength = 0.0;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "inverseKinematicsAdapter");
  ros::NodeHandle n;

  poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 1);

  ros::Subscriber state_sub = n.subscribe("/dyret/state", 1, stateCallback);
  ros::Subscriber positionCommand_sub = n.subscribe("/dyret/dyret_controller/positionCommand", 1, positionCommandCallback);

  ros::spin();

  ros::shutdown();

  return 0;
}