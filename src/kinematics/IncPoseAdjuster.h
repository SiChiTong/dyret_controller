#include "ros/ros.h"
#include "../states/poseAdjustState.h"

#include "kinematicTypes.h"

class IncPoseAdjuster{

  bool closedLoop;
  float currentProgress;
  std::vector<vec3P> startPositions;

  std::vector<double>* servoAnglesInRad;

  ros::ServiceClient inverseKinematicsService_client;
  ros::Publisher dynCommands_pub;

  t_poseAdjustState currentPoseStates[4];
  std::vector<vec3P> goalPose;

  std::vector<vec3P> positionArray;
  std::vector<double> currentLean = {0.0, 0.0};

  bool reachedPose;
  vec3P tmpLegPoseVar;

  // Parameters for movement:
  const float stepHeight = 40.0;
  const float leanSpeed = 1.2;
  const float legMoveSpeed = 1.2;
  const float stepDownSpeed = 0.8;
  const float liftSpeed = 1.0;

  float groundHeight;

public:

  IncPoseAdjuster(bool givenClosedLoop,
                  std::vector<vec3P> givenGoalPose,
                  std::vector<double>* givenServoAnglesInRad,
                  ros::ServiceClient givenInverseKinematicsService_client,
                  ros::Publisher givenDynCommands_pub){

    currentProgress = 0.0;
    closedLoop = givenClosedLoop;
    positionArray.resize(4); // Set size 4
    servoAnglesInRad = givenServoAnglesInRad;
    goalPose = givenGoalPose;
    inverseKinematicsService_client = givenInverseKinematicsService_client;
    dynCommands_pub = givenDynCommands_pub;
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;

    reachedPose = false;
  }

  void setGoalPose(std::vector<vec3P> givenGoalPose){
    goalPose = givenGoalPose;
  }

  bool Spin();
  void reset(){
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;
    reachedPose = false;
  }

  void skip(){
    for (int i = 0; i < 4; i++) currentPoseStates[i] = FINISHED;
    reachedPose = true;
  }

  bool done(){
    return reachedPose;
  }
};
