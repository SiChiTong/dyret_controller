#include "ros/ros.h"
#include "../states/poseAdjustState.h"

#include "kinematicTypes.h"

class IncPoseAdjuster{

  float currentProgress;

  std::vector<double>* servoAnglesInRad;

  ros::Publisher positionCommand_pub;

  t_poseAdjustState currentPoseStates[4];
  std::vector<vec3P> goalPose;

  std::vector<vec3P> positionArray;
  std::vector<double> currentLean = {0.0, 0.0};

  std::vector<double> legActuatorLengths = {0.0, 0.0};

  static constexpr int stateTransitionDelay = 0;

  bool reachedPose;

  // Parameters for movement:
  const float stepHeight = 40.0;
  const float leanSpeed = 1.0;
  const float legMoveSpeed = 3.0;
  const float stepDownSpeed = 0.5;
  const float liftSpeed = 2.0;

  float groundHeight;

public:

  IncPoseAdjuster(std::vector<vec3P> givenGoalPose,
                  std::vector<double>* givenServoAnglesInRad,
                  ros::Publisher givenPositionCommand_pub){
    currentProgress = 0.0;
    positionArray.resize(4); // Set size 4
    servoAnglesInRad = givenServoAnglesInRad;
    goalPose = givenGoalPose;
    positionCommand_pub = givenPositionCommand_pub;
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;
    currentProgress = 0.0;

    reachedPose = false;
  }

  void setPoseAndActuatorLengths(std::vector<vec3P> givenGoalPose, std::vector<double> givenActuatorLengths){
    legActuatorLengths = givenActuatorLengths;
    goalPose = givenGoalPose;
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
