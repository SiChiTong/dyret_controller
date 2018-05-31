#include <math.h> 
#include <vector>
#include <chrono>
#include <iostream>

#include "dyret_common/angleConv.h"

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <unistd.h>
#include <dynamic_reconfigure/server.h>

#include "dyret_common/State.h"
#include "dyret_common/Pose.h"

#include "gait/gait.h"
#include "gait/BSplineGait.h"

#include "kinematics/movementFunctions.h"
#include "kinematics/kinematicFunctions.h"
#include "kinematics/forwardKinematics.h"
#include "kinematics/inverseKinematics.h"
#include "kinematics/interpolation.h"

#include "kinematics/IncPoseAdjuster.h"

#include "dyret_common/wait_for_ros.h"
#include "dyret_common/timeHandling.h"

#include "dyret_common/Configure.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/ActionMessage.h"
#include "dyret_controller/DistAng.h"

#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GetGaitEvaluation.h"

#include "dyret_controller/gaitControllerParamsConfig.h"

#include <dyret_hardware/ActuatorBoardState.h>

using namespace std::chrono;

int currentAction;
dyret_controller::ActionMessage::ConstPtr lastActionMessage;
robo_cont::gaitControllerParamsConfig lastGaitControllerParamsConfigMessage;

bool movingForward;
double globalStepHeight;
double globalStepLength;
double globalSmoothing;
double globalGaitFrequency;
double globalGaitSpeed;
double globalWagAmplitude_x;
double globalWagAmplitude_y;
double globalWagPhaseOffset;
double globalLiftDuration;
const double groundHeightOffset = -430;
const double groundCorrectionFactor = 0.8;
float groundHeight = -430.0f;
std::vector<double> legActuatorLengths = {0.0, 0.0};

const float bSplineGaitWagOffset = 0.91;

const float spreadAmount  =  80.0; // was 50

std::vector<int> pidParameters;

std::vector<double> servoAnglesInRad(12);
std::vector<double> prismaticPositions(8);
float femurActuatorLength = 0.0;
float tibiaActuatorLength = 0.0;

void actuatorState_Callback(const dyret_hardware::ActuatorBoardState::ConstPtr& msg){

    if (msg->position.size() == 8) {
        float femurActuatorLength = (msg->position[0] + msg->position[2] + msg->position[4] + msg->position[6]) / 4.0f;
        float tibiaActuatorLength = (msg->position[1] + msg->position[3] + msg->position[5] + msg->position[7]) / 4.0f;

        legActuatorLengths[0] = femurActuatorLength;
        legActuatorLengths[1] = tibiaActuatorLength;

        groundHeight = groundHeightOffset - ((femurActuatorLength + tibiaActuatorLength) * groundCorrectionFactor);
    } else {
        ROS_WARN("Did not apply actuatorCommand to inverse kinematic");
    }

}

bool getGaitControllerStatusService(dyret_controller::GetGaitControllerStatus::Request  &req,
                                    dyret_controller::GetGaitControllerStatus::Response &res){
  res.gaitControllerStatus.actionType = currentAction;

  return true;
}

void actionMessagesCallback(const dyret_controller::ActionMessage::ConstPtr& msg){
  ROS_INFO("Switching currentAction from %u to %u", currentAction, msg->actionType);
  lastActionMessage = msg;
  currentAction = msg->actionType;

  ROS_INFO("Direction is %.2f (%.2f)", lastActionMessage->direction, msg->direction);
}

void servoStatesCallback(const dyret_common::State::ConstPtr& msg){
  for (int i = 0; i < 12; i++){
      servoAnglesInRad[i] = msg->revolute[i].position;
  }
  if (msg->revolute.size() == prismaticPositions.size()){
    for (int i = 0; i < prismaticPositions.size(); i++){
      prismaticPositions[i] = msg->prismatic[i].position;
    }
    femurActuatorLength = (prismaticPositions[0] + prismaticPositions[2] + prismaticPositions[4] + prismaticPositions[6]) / 4.0f;
    tibiaActuatorLength = (prismaticPositions[1] + prismaticPositions[3] + prismaticPositions[5] + prismaticPositions[7]) / 4.0f;
  }
}

void gaitControllerParamConfigCallback(robo_cont::gaitControllerParamsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: \n\t"
               "stepHeight: %.2f\n\t"
               "stepLength: %.2f\n\t"
               "smoothing: %.2f\n\t"
               "speed: %.2f\n\t"
               "wagAmplitude_x: %.2f\n\t"
               "wagAmplitude_y: %.2f\n\t"
               "wagPhaseOffset: %.2f\n\t"
               "liftDuration: %.2f",
            config.stepHeight,
            config.stepLength,
            config.smoothing,
            config.gaitFrequency,
            config.wagAmplitude_x,
            config.wagAmplitude_y,
            config.wagPhase,
            config.liftDuration);

  ROS_INFO("\n\tC: P%u I%u D%u\n\tF: P%u I%u D%u\n\tT: P%u I%u D%u\n\n",
            config.cP, config.cI, config.cD, config.fP, config.fI, config.fD, config.tP, config.tI, config.tD);

  lastGaitControllerParamsConfigMessage = config;

}

void startGaitRecording(ros::ServiceClient get_gait_evaluation_client){

  dyret_controller::GetGaitEvaluation srv;

  srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_start;

  if (get_gait_evaluation_client.call(srv)) {
    ROS_INFO("Called startGaitRecording service\n");
  }

}

void pauseGaitRecording(ros::ServiceClient get_gait_evaluation_client){

  dyret_controller::GetGaitEvaluation srv;

  srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_pause;

  if (get_gait_evaluation_client.call(srv)) {
    ROS_INFO("Called pauseGaitRecording service\n");
  }

}

std::vector<vec3P> getRestPose(){
    const float frontOffset   =  0.0;

    const std::vector<vec3P> restPose = {{ -spreadAmount, frontOffset, groundHeight },
                                         {  spreadAmount, frontOffset, groundHeight },
                                         {  spreadAmount, frontOffset, groundHeight },
                                         { -spreadAmount, frontOffset, groundHeight }};

    return restPose;
}

int main(int argc, char **argv)
{

  pidParameters.resize(3*3);

  ros::init(argc, argv, "gaitController");
  ros::NodeHandle n;

  // Initialize services
  ros::ServiceClient get_gait_evaluation_client = n.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
  ros::ServiceServer gaitControllerStatusService_server = n.advertiseService("get_gait_controller_status", getGaitControllerStatusService);
  // Initialize topics
  ros::Subscriber actionMessages_sub = n.subscribe("/dyret/dyret_controller/actionMessages", 100, actionMessagesCallback);
  ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);
  ros::Subscriber gaitInferredPos_sub = n.subscribe("/dyret/actuator_board/state", 1000, actuatorState_Callback);
  ros::Publisher  poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 3);
  ros::Publisher  gaitInferredPos_pub = n.advertise<dyret_controller::DistAng>("/dyret/dyret_controller/gaitInferredPos", 1000);
  ros::ServiceClient servoConfigClient = n.serviceClient<dyret_common::Configure>("/dyret/configuration");
  ros::Publisher positionCommand_pub = n.advertise<dyret_controller::PositionCommand>("/dyret/dyret_controller/positionCommand", 1);;

  waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
  waitForRosInit(actionMessages_sub, "/dyret/dyret_controller/actionMessages");
  waitForRosInit(servoStates_sub, "servoStates");

  // Initialize dynamic reconfiguration:
  dynamic_reconfigure::Server<robo_cont::gaitControllerParamsConfig> gaitControllerParamsConfigServer;
  dynamic_reconfigure::Server<robo_cont::gaitControllerParamsConfig>::CallbackType gaitControllerParamsConfigFunction;
  gaitControllerParamsConfigFunction = boost::bind(&gaitControllerParamConfigCallback, _1, _2);
  gaitControllerParamsConfigServer.setCallback(gaitControllerParamsConfigFunction);

  // Initialize bSplineGait
  globalStepLength     = 150.0;
  globalStepHeight     = 35.0;
  globalSmoothing      = 25.0;
  globalGaitFrequency  = 1.0;
  globalWagAmplitude_x = 40.0;
  globalWagAmplitude_y = 20.0;
  globalWagPhaseOffset = 0.0;
  globalLiftDuration   = 0.125;

  const float frontOffset   =   0.0f;
  const float leftOffset    =   0.0f;
  const float rearLegOffset = -30.0f;

  BSplineGait bSplineGait = BSplineGait(globalStepHeight,
                                        globalStepLength,
                                        globalSmoothing,
                                        groundHeight,
                                        spreadAmount,
                                        frontOffset,
                                        leftOffset,
                                        rearLegOffset,
                                        globalLiftDuration);

  bSplineGait.enableWag(bSplineGaitWagOffset, 40.0f, 0.0f);

  IncPoseAdjuster bSplineInitAdjuster(add(bSplineGait.getPosition(0.0, true), bSplineGait.getGaitWagPoint(0.0, true)),
                                      &servoAnglesInRad,
                                      positionCommand_pub);

  IncPoseAdjuster restPoseAdjuster(getRestPose(),
                                   &servoAnglesInRad,
                                   positionCommand_pub);

  int lastAction = dyret_controller::ActionMessage::t_idle;
  currentAction  = dyret_controller::ActionMessage::t_idle;

  setServoSpeeds(0.01, servoConfigClient);

  printf("P: %d, I: %d, D: %d\n", lastGaitControllerParamsConfigMessage.cP, lastGaitControllerParamsConfigMessage.cI, lastGaitControllerParamsConfigMessage.cD);
  pidParameters[0] = 10; // Set to 10 to stop coxa shaking before experiment begins
  pidParameters[1] = lastGaitControllerParamsConfigMessage.cI;
  pidParameters[2] = lastGaitControllerParamsConfigMessage.cD;
  pidParameters[3] = lastGaitControllerParamsConfigMessage.fP;
  pidParameters[4] = lastGaitControllerParamsConfigMessage.fI;
  pidParameters[5] = lastGaitControllerParamsConfigMessage.fD;
  pidParameters[6] = lastGaitControllerParamsConfigMessage.tP;
  pidParameters[7] = lastGaitControllerParamsConfigMessage.tI;
  pidParameters[8] = lastGaitControllerParamsConfigMessage.tD;
  setServoPIDs(pidParameters, servoConfigClient);

  std::vector<vec3P> restPose = getRestPose();
  moveAllLegsToGlobalPosition(getRestPose(), positionCommand_pub);

  restPoseAdjuster.skip();

  const double poseAdjustSpeed = 0.08;
  const double gaitServoSpeed = 0.0;

  ros::Rate loop_rate(3);

  long long int startTime;
  long long int lastTime = 0;
  bool activatedRecording = false;
  std::vector<vec3P> lastGlobalLegPositions;
  ros::Rate poseAdjusterRate(50);

  while ( ros::ok() ) {
      if (currentAction == dyret_controller::ActionMessage::t_sleep){

      } else if (currentAction == dyret_controller::ActionMessage::t_idle){
          //loop_rate.sleep();

          //moveAllLegsToGlobal(getRestPose(), ..., poseCommand_pub);

      }else if (currentAction == dyret_controller::ActionMessage::t_restPose){

          // Check for transition
          if (lastAction == dyret_controller::ActionMessage::t_contGait){

              setServoLog(false, servoConfigClient);
              pauseGaitRecording(get_gait_evaluation_client);

              activatedRecording = false;

              setServoSpeeds(poseAdjustSpeed, servoConfigClient);

              restPoseAdjuster.setPoseAndActuatorLengths(getRestPose(), legActuatorLengths);
              restPoseAdjuster.reset();
          }

          if (restPoseAdjuster.done() == false){
              restPoseAdjuster.Spin();
              poseAdjusterRate.sleep();

          } else {
              currentAction = dyret_controller::ActionMessage::t_idle;
          }

      }else if (currentAction == dyret_controller::ActionMessage::t_contGait){

          // Check for transition
          if (lastAction != dyret_controller::ActionMessage::t_contGait){
              setServoSpeeds(poseAdjustSpeed, servoConfigClient);
              bSplineInitAdjuster.reset();

              // PID:
              pidParameters[0] = lastGaitControllerParamsConfigMessage.cP;
              pidParameters[1] = lastGaitControllerParamsConfigMessage.cI;
              pidParameters[2] = lastGaitControllerParamsConfigMessage.cD;
              pidParameters[3] = lastGaitControllerParamsConfigMessage.fP;
              pidParameters[4] = lastGaitControllerParamsConfigMessage.fI;
              pidParameters[5] = lastGaitControllerParamsConfigMessage.fD;
              pidParameters[6] = lastGaitControllerParamsConfigMessage.tP;
              pidParameters[7] = lastGaitControllerParamsConfigMessage.tI;
              pidParameters[8] = lastGaitControllerParamsConfigMessage.tD;
              setServoPIDs(pidParameters, servoConfigClient);

              // Gait params:
              globalStepHeight     = lastGaitControllerParamsConfigMessage.stepHeight;
              globalStepLength     = lastGaitControllerParamsConfigMessage.stepLength;
              globalSmoothing      = lastGaitControllerParamsConfigMessage.smoothing;
              globalGaitFrequency  = lastGaitControllerParamsConfigMessage.gaitFrequency;
              globalGaitSpeed      = lastGaitControllerParamsConfigMessage.gaitSpeed;
              globalWagAmplitude_x = lastGaitControllerParamsConfigMessage.wagAmplitude_x;
              globalWagAmplitude_y = lastGaitControllerParamsConfigMessage.wagAmplitude_y;
              globalWagPhaseOffset = lastGaitControllerParamsConfigMessage.wagPhase;
              globalLiftDuration   = lastGaitControllerParamsConfigMessage.liftDuration;

              bSplineGait = BSplineGait(globalStepHeight,
                                        globalStepLength,
                                        globalSmoothing,
                                        groundHeight,
                                        spreadAmount,
                                        frontOffset,
                                        leftOffset,
                                        rearLegOffset,
                                        globalLiftDuration);

              bSplineGait.enableWag(bSplineGaitWagOffset+globalWagPhaseOffset, globalWagAmplitude_x, globalWagAmplitude_y);

              if ((std::isnan(globalGaitSpeed) == true) && (std::isnan(globalGaitFrequency) == true)){
                  ROS_ERROR("GlobalGaitSpeed or globalGaitFrequency has to be set\n");
              } else if (std::isnan(globalGaitFrequency) == true){
                  // Frequency has not been set => Calculate frequency from speed

                  ROS_FATAL("std::isnan(globalGaitFrequency) == true\n");
                  exit(-1);
              }

              if (fabs(lastActionMessage->direction - M_PI) < 0.1){
                movingForward = false;
              } else {
                movingForward = true;
              }

              vec3P wagPoint = bSplineGait.getGaitWagPoint(0.0, movingForward);

              std::vector<vec3P> initPose = lockToZ(add(bSplineGait.getPosition(0.0, movingForward), wagPoint), groundHeight);
              bSplineInitAdjuster.setPoseAndActuatorLengths(initPose, legActuatorLengths);

              lastTime = 0;

          }

          if (bSplineInitAdjuster.done() == false){
              if (bSplineInitAdjuster.Spin() == true){
                  setServoSpeeds(gaitServoSpeed, servoConfigClient);
              }
              poseAdjusterRate.sleep();

              startTime = std::chrono::duration_cast< std::chrono::milliseconds > (system_clock::now().time_since_epoch()).count();
          } else {

            if (activatedRecording == false){
                startGaitRecording(get_gait_evaluation_client);
                setServoLog(true, servoConfigClient);
                activatedRecording = true;
            }

            // Get leg positions:
            double currentRelativeTime = (std::chrono::duration_cast< milliseconds >
                (system_clock::now().time_since_epoch()).count()) - startTime; // Given in milliseconds
            std::vector<vec3P> globalLegPositions(4);

            globalLegPositions = bSplineGait.getPosition(currentRelativeTime * globalGaitFrequency, movingForward);

            // Calculate posChangeMsg:
            if (lastTime == 0){
                // Not initialized
                lastTime = getMs();
            } else {
                // Initialized:

                double secondsPassed = getMs() - lastTime;
                double distance = bSplineGait.getStepLength() * globalGaitFrequency * (secondsPassed / 1000.0f);

                dyret_controller::DistAng posChangeMsg;
                if (movingForward) posChangeMsg.distance = distance; else posChangeMsg.distance = -distance;

                posChangeMsg.msgType = posChangeMsg.t_measurementInferred;
                posChangeMsg.angle    = 0.0f;
                gaitInferredPos_pub.publish(posChangeMsg);
            }

            lastTime = getMs();

            dyret_common::Pose msg;

            std::vector<int> servoIds(12);
            for (int i = 0; i < servoIds.size(); i++) servoIds[i] = i;

            std::vector<float> anglesInRad;
            int currentActuatorIndex = 0;

            vec3P wag = bSplineGait.getGaitWagPoint(currentRelativeTime * globalGaitFrequency, movingForward);

            std::vector<vec3P> currentPositions = currentLegPositions(servoAnglesInRad, legActuatorLengths);

            // L0z, L1z, L2z, L3_z, wag_commanded_x, wag_commanded_y, L0_x, L1_x, L2_x, L3_x
/*            fprintf(wagLog, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                    currentPositions[0].points[2],
                    currentPositions[1].points[2],
                    currentPositions[2].points[2],
                    currentPositions[3].points[2],
                    wag.points[0],
                    wag.points[1],
                    currentPositions[0].points[0],
                    currentPositions[1].points[0],
                    currentPositions[2].points[0],
                    currentPositions[3].points[0]);
*/
            // Get IK solutions for each leg:
            for (int i = 0; i < 4; i++){ // For each leg

              vec3P legPosition = add(globalLegPositions[i], wag);

              std::vector<double> inverseReturn = inverseKinematics::calculateInverseKinematics(legPosition.x(),
                                                                                                legPosition.y(),
                                                                                                legPosition.z(),
                                                                                                i,
                                                                                                femurActuatorLength,
                                                                                                tibiaActuatorLength);
              for (int j = 0; j < 3; j++){
                anglesInRad.push_back(inverseReturn[j]);
              }

            }

/*        std::vector<vec3P> actual = currentLegPositions(servoAnglesInRad, legActuatorLengths);

        fprintf(gaitLogGlobal,"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
                currentRelativeTime,
                globalLegPositions[0].x(), globalLegPositions[0].y(), globalLegPositions[0].z(),
                actual[0].x(), actual[0].y(), actual[0].z(),
                globalLegPositions[1].x(), globalLegPositions[1].y(), globalLegPositions[1].z(),
                actual[1].x(), actual[1].y(), actual[1].z(),
                globalLegPositions[2].x(), globalLegPositions[2].y(), globalLegPositions[2].z(),
                actual[2].x(), actual[2].y(), actual[2].z(),
                globalLegPositions[3].x(), globalLegPositions[3].y(), globalLegPositions[3].z(),
                actual[3].x(), actual[3].y(), actual[3].z()
               );
*/

        if (servoIds.size() != 0){
            msg.revolute = anglesInRad;
            poseCommand_pub.publish(msg);
        } else {
            ROS_WARN("Did not send invalid dyn commands!\n");
        }
      }
    } else {
        // Undefined action!
    }

    lastAction = currentAction; // Save last action to handle transitions
    ros::spinOnce();

  }

/*
  fclose(wagLog);
  fclose(gaitLogGlobal);
*/

  ROS_INFO("Exiting gaitController");

  return 0;
}
