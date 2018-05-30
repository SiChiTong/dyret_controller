#include "ros/ros.h"
#include "../states/poseAdjustState.h"

#include "kinematicTypes.h"

class IncPoseAdjuster{

  bool closedLoop;
  float currentProgress;
  std::vector<vec3P> startPositions;

  std::vector<double>* servoAnglesInRad;
  float* femurActuatorLength;
  float* tibiaActuatorLength;

  ros::Publisher poseCommand_pub;

  t_poseAdjustState currentPoseStates[4];
  std::vector<vec3P> goalPose;

  std::vector<vec3P> positionArray;
  std::vector<double> currentLean = {0.0, 0.0};

  std::vector<double> legActuatorLengths = {0.0, 0.0};

  static constexpr int stateTransitionDelay = 5;

  bool reachedPose;
  vec3P tmpLegPoseVar;

  // Parameters for movement:
  const float stepHeight = 40.0;
  const float leanSpeed = 0.2;
  const float legMoveSpeed = 0.2;
  const float stepDownSpeed = 0.1;
  const float liftSpeed = 1.0;

  float groundHeight;

public:

  IncPoseAdjuster(bool givenClosedLoop,
                  std::vector<vec3P> givenGoalPose,
                  std::vector<double>* givenServoAnglesInRad,
                  float* givenFemurActuatorLength,
                  float* givenTibiaActuatorLength,
                  ros::Publisher givenPoseCommand_pub){
    currentProgress = 0.0;
    closedLoop = givenClosedLoop;
    positionArray.resize(4); // Set size 4
    servoAnglesInRad = givenServoAnglesInRad;
    femurActuatorLength = givenFemurActuatorLength;
    tibiaActuatorLength = givenTibiaActuatorLength;
    goalPose = givenGoalPose;
    poseCommand_pub = givenPoseCommand_pub;
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;

    reachedPose = false;
  }

  void setPoseAndActuatorLengths(std::vector<vec3P> givenGoalPose, std::vector<double> givenActuatorLengths){
    legActuatorLengths = givenActuatorLengths;
    goalPose = givenGoalPose;
  }

  bool Spin();
  void reset(){
    for (int i = 0; i < 4; i++) currentPoseStates[i] = INIT_STEPDOWN;
    reachedPose = false;
    startPositions.clear();
  }

  void skip(){
    for (int i = 0; i < 4; i++) currentPoseStates[i] = FINISHED;
    reachedPose = true;
  }

  bool done(){
    return reachedPose;
  }
};
