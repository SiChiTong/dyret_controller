#include <math.h> 
#include <vector>
#include <chrono>
#include <iostream>

#include "ros/ros.h"

#include "dyret_controller/DistAng.h"
#include "dyret_controller/Trajectory.h"

#include "dyret_common/wait_for_ros.h"

#include "states/trajectoryControllerState.h"

#include "dyret_controller/ActionMessage.h"

double accPos;
double accAngle;
std::vector<double> distanceGoal;
std::vector<double> angleGoal;
std::vector<double> timeoutInSec;
int currentSegment = 0;
bool directionForward;

bool sentMessage = false;
bool started;
ros::Time startTime;
bool newTrajectoryMessageReceived;

t_trajectoryControllerState currentState;

void gaitInferredPos_Callback(const dyret_controller::DistAng::ConstPtr& msg)
{
  if (msg->msgType == msg->t_measurementInferred){
      if (started == false){
          started = true;
          startTime = ros::Time::now();
          ROS_INFO("Set new startTime\n");
      }

      accPos   += msg->distance;
      accAngle += msg->angle;
  }

}

void trajectoryMessages_Callback(const dyret_controller::Trajectory::ConstPtr& msg)
{
  currentSegment = 0;

  if (msg->command == msg->t_executeTrajectory){

      distanceGoal.resize(msg->trajectorySegments.size());
      angleGoal.resize(msg->trajectorySegments.size());
      timeoutInSec.resize(msg->trajectorySegments.size());

      for (int i = 0; i < msg->trajectorySegments.size(); i++){
          distanceGoal[i] = msg->trajectorySegments[i].distance;
          angleGoal[i] = msg->trajectorySegments[i].angle;
          timeoutInSec[i] = msg->trajectorySegments[i].timeoutInSec;
      }
  } else if (msg->command == msg->t_resetPosition){
      ROS_INFO("Reset position from %.2f to 0.00\n", accPos);
      started = false;
      accPos = 0.0;
      distanceGoal.clear();
      accAngle = 0.0;
      angleGoal.clear();
  }

  sentMessage = false;
  newTrajectoryMessageReceived = true;

}

void sendRestPoseMessage(ros::Publisher givenActionMessages_pub){
	dyret_controller::ActionMessage actionMessage;
  actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
  actionMessage.actionType = dyret_controller::ActionMessage::t_restPose;
  actionMessage.speed = 0.0;
  actionMessage.direction = 0.0;
  givenActionMessages_pub.publish(actionMessage);
}

void sendContGaitMessage(double givenDirection, ros::Publisher givenActionMessages_pub){
  dyret_controller::ActionMessage actionMessage;
  actionMessage.configuration = dyret_controller::ActionMessage::t_mammal;
  actionMessage.actionType = dyret_controller::ActionMessage::t_contGait;
  actionMessage.speed = 0.0;
  actionMessage.direction = givenDirection;
  givenActionMessages_pub.publish(actionMessage);
}

int main(int argc, char **argv)
{

  currentState = WAIT;

  started = false;
  newTrajectoryMessageReceived = false;
  accPos = 0.0;

  ros::init(argc, argv, "trajectoryController");
  ros::NodeHandle n;

  ros::Subscriber gaitInferredPos_sub = n.subscribe("/dyret/dyret_controller/gaitInferredPos", 1000, gaitInferredPos_Callback);
  ros::Subscriber trajectoryMessages_sub = n.subscribe("/dyret/dyret_controller/trajectoryMessages", 1000, trajectoryMessages_Callback);
  ros::Publisher  actionMessages_pub = n.advertise<dyret_controller::ActionMessage>("/dyret/dyret_controller/actionMessages", 10);
  ros::AsyncSpinner spinner(4); // Use 4 threads

  sleep(1);

  spinner.start();

  ros::Rate loop_rate(20);

  while (ros::ok()){

    loop_rate.sleep();

    switch(currentState){
      case WAIT:
      {
        if ( (newTrajectoryMessageReceived == true) && (distanceGoal.size() != 0) ){
            currentSegment = 0;
            currentState   = SETUP_WALK;
            newTrajectoryMessageReceived = false;
            ROS_INFO("WAIT -> SETUP_WALK (s%u)\n", currentSegment);
        }

        break;
      }
      case SETUP_WALK:
      {
        if (distanceGoal.size() <= currentSegment){
            // No more segments to cover
            sendRestPoseMessage(actionMessages_pub);
            currentState = WAIT;
            ROS_INFO("SETUP_WALK -> WAIT (finished @ %u)\n", currentSegment);
        } else {

            // Send correct message:
            if (accPos < distanceGoal[currentSegment]){
                // Forward
                directionForward = true;
                sendContGaitMessage(0.0, actionMessages_pub);
            } else {
                // Reverse
                directionForward = false;
                sendContGaitMessage(M_PI, actionMessages_pub);
            }

            started = false;
            currentState = WALKING;
            if (directionForward == true) ROS_INFO("SETUP_WALK -> WALKING (F)\n"); else ROS_INFO("SETUP_WALK -> WALKING (R)\n");
        }

        break;
      }
      case WALKING:
      {
        if ( (directionForward ==  true && (distanceGoal[currentSegment] - accPos) < 0) ||
             (directionForward == false && (distanceGoal[currentSegment] - accPos) > 0) ){
                // Goal reached
                ROS_INFO("WALKING -> SETUP_WALK (seg%u done at %.2f of %.2f)\n", currentSegment, accPos, distanceGoal[currentSegment]);
                currentSegment++;
                currentState = SETUP_WALK;
        }

        float currentRelativeTime = (ros::Time::now() - startTime).sec;

        if ( (started == true) && (timeoutInSec[currentSegment] != 0.0) && (currentRelativeTime >= timeoutInSec[currentSegment]) ){
            // TIMEOUT
            ROS_INFO("WALKING -> SETUP_WALK (Timeout @ %.2f, seg%u)\n", currentRelativeTime, currentSegment);
            currentSegment++;
            currentState = SETUP_WALK;
        }

        //printf("Goal: %.2f, currentPos: %.2f, currentTime: %.2f, timeout: %.2f\n", distanceGoal[currentSegment], accPos, currentRelativeTime, timeoutInSec[currentSegment] );

        break;
      }
      default:
        ROS_INFO("INVALID CURRENTSTATE IN TRAJECTORYCONTROLLER\n");
        break;
    }
  }

  ros::waitForShutdown();

  return 0;
}
