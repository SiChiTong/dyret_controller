#include "ros/ros.h"
#include "../states/poseAdjustState.h"

#include "kinematicTypes.h"

class IncPoseAdjuster{

  float currentProgress;

  std::vector<double>* servoAnglesInRad;

  ros::ServiceClient* positionCommandService;

  t_poseAdjustState currentPoseStates[4];
  std::vector<vec3P> goalPose;

  std::vector<vec3P> positionArray;
  std::vector<double> currentLean = {0.0, 0.0};

  std::vector<double>* legActuatorLengths;

  static constexpr int stateTransitionDelay = 0;

  bool reachedPose;

  // Parameters for movement:
  const float stepHeight = 40.0;
  const float leanSpeed = 5;
  const float legMoveSpeed = 15.0;
  const float stepDownSpeed = 5.0;
  const float liftSpeed = 10.0;

  float groundHeight = -1;
  double medianlegHeight;

public:

  IncPoseAdjuster(std::vector<double>* givenServoAnglesInRad,
                  std::vector<double>* givenActuatorLengths,
                  ros::ServiceClient* givenPositionCommandService){
    currentProgress = 0.0;
    positionArray.resize(4); // Set size 4
    servoAnglesInRad = givenServoAnglesInRad;
    legActuatorLengths = givenActuatorLengths;
    positionCommandService = givenPositionCommandService;
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;
    currentProgress = 0.0;

    reachedPose = false;
  }

  void setPose(std::vector<vec3P> givenGoalPose){
    goalPose = givenGoalPose;
  }

  void setPose(std::vector<vec3P> givenGoalPose, float givenGroundHeight){
    groundHeight = givenGroundHeight;
    setPose(givenGoalPose);
  }

  bool Spin();
  void reset(){
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;
    currentProgress = 0.0;
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
