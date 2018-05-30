#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "ros/console.h"

#include "dyret_common/Pose.h"
#include "dyret_common/State.h"

#include "dyret_controller/PositionCommand.h"

#include "kinematics/inverseKinematics.h"

ros::Publisher poseCommand_pub;

std::vector<float> currentLegAngles(12);

double femurActuatorLength = 0.0;
double tibiaActuatorLength = 0.0;

void positionCommandCallback(const dyret_controller::PositionCommand::ConstPtr& msg){

  std::vector<float> legAngles;

  for (int i = 0; i < 4; i++){
    // Get inverse kinematics solution for current leg
    std::vector<double> inverseKinematicsReturn = inverseKinematics::calculateInverseKinematics(msg->legPosition[i].x,
                                                                                                msg->legPosition[i].y,
                                                                                                msg->legPosition[i].z,
                                                                                                i,
                                                                                                femurActuatorLength,
                                                                                                tibiaActuatorLength);
    legAngles.insert(legAngles.end(), inverseKinematicsReturn.begin(), inverseKinematicsReturn.end());

  }

  std::ostringstream rosStringStream;

  for (int i = 0; i < 4; i++){
    rosStringStream << "\n\tId " << i << ": " << msg->legPosition[i].x << ", " << msg->legPosition[i].y << ", " << msg->legPosition[i].z;
  }

  ROS_INFO("%s\nCalculated angles:\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t",
           rosStringStream.str().c_str(),
           legAngles[0],  legAngles[1],  legAngles[2],
           legAngles[3],  legAngles[4],  legAngles[5],
           legAngles[6],  legAngles[7],  legAngles[8],
           legAngles[9], legAngles[10], legAngles[11]);

  dyret_common::Pose poseMsg;
  poseMsg.revolute = legAngles;
  poseCommand_pub.publish(poseMsg);

}

void stateCallback(const dyret_common::State::ConstPtr& msg){
  if (msg->prismatic.size() == 8){
    femurActuatorLength = (msg->prismatic[0].position + msg->prismatic[2].position + msg->prismatic[4].position + msg->prismatic[6].position) / 4.0f;
    tibiaActuatorLength = (msg->prismatic[1].position + msg->prismatic[3].position + msg->prismatic[5].position + msg->prismatic[7].position) / 4.0f;
  }

  if (msg->revolute.size() == 12){
    for (int i = 0; i < 12; i++){
      currentLegAngles[i] = msg->revolute[i].position;
    }

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