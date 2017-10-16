#include <math.h> 
#include <vector>
#include <chrono>
#include <iostream>

#include "dyret_utils/angleConv.h"

#include "ros/ros.h"
#include <ros/console.h>
#include "dyret_common/CalculateInverseKinematics.h"
#include "std_msgs/String.h"
#include <unistd.h>
#include <dynamic_reconfigure/server.h>

#include "dyret_common/ServoState.h"
#include "dyret_common/ServoStateArray.h"
#include "dyret_common/ServoConfig.h"
#include "dyret_common/ServoConfigArray.h"
#include "dyret_common/Pose.h"
#include "dyret_common/ActionMessage.h"
#include "dyret_common/DistAng.h"
#include "dyret_common/GetGaitEvaluation.h"

#include "dyret_controller/gaitControllerParamsConfig.h"
#include "dyret_common/GetGaitControllerStatus.h"

#include "gait/gait.h"
#include "gait/BSplineGait.h"

#include "kinematics/movementFunctions.h"
#include "kinematics/kinematicFunctions.h"
#include "kinematics/forwardKinematics.h"
#include "kinematics/interpolation.h"

#include "kinematics/IncPoseAdjuster.h"

#include "dyret_utils/wait_for_ros.h"
#include "dyret_utils/timeHandling.h"

using namespace std::chrono;

int currentAction;
dyret_common::ActionMessage::ConstPtr lastActionMessage;
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

std::vector<int> pidParameters;

std::vector<double> servoAnglesInRad(12);

bool getGaitControllerStatusService(dyret_common::GetGaitControllerStatus::Request  &req,
		dyret_common::GetGaitControllerStatus::Response &res){
  res.gaitControllerStatus.actionType = currentAction;

  return true;
}

void actionMessagesCallback(const dyret_common::ActionMessage::ConstPtr& msg){
  ROS_INFO("Switching currentAction from %u to %u", currentAction, msg->actionType);
  lastActionMessage = msg;
  currentAction = msg->actionType;

  ROS_INFO("Direction is %.2f (%.2f)", lastActionMessage->direction, msg->direction);
}

void servoStatesCallback(const dyret_common::ServoStateArray::ConstPtr& msg){
  for (int i = 0; i < 12; i++){
      if (i == 1 || i == 5 || i == 8 || i == 10){ // Invert
        servoAnglesInRad[i] = invRad(msg->servoStates[i].position);
      } else {
        servoAnglesInRad[i] = msg->servoStates[i].position;
      }
  }
}

void gaitControllerParamConfigCallback(robo_cont::gaitControllerParamsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: \n\tstepHeight: %.2f\n\tstepLength: %.2f\n\tsmoothing: %.2f\n\tspeed: %.2f\n\twagAmplitude_x: %.2f\n\twagAmplitude_y: %.2f\n\twagPhaseOffset: %.2f",
            config.stepHeight,
            config.stepLength,
            config.smoothing,
            config.gaitFrequency,
            config.wagAmplitude_x,
            config.wagAmplitude_y,
            config.wagPhase);
  ROS_INFO("\n\tC: P%u I%u D%u\n\tF: P%u I%u D%u\n\tT: P%u I%u D%u\n\n",
            config.cP, config.cI, config.cD, config.fP, config.fI, config.fD, config.tP, config.tI, config.tD);

  lastGaitControllerParamsConfigMessage = config;

}

void startGaitRecording(ros::ServiceClient get_gait_evaluation_client){

  dyret_common::GetGaitEvaluation srv;

  srv.request.givenCommand = dyret_common::GetGaitEvaluation::Request::t_start;

  if (get_gait_evaluation_client.call(srv)) {
    ROS_INFO("Called startGaitRecording service\n");
  }

}

void pauseGaitRecording(ros::ServiceClient get_gait_evaluation_client){

  dyret_common::GetGaitEvaluation srv;

  srv.request.givenCommand = dyret_common::GetGaitEvaluation::Request::t_pause;

  if (get_gait_evaluation_client.call(srv)) {
    ROS_INFO("Called pauseGaitRecording service\n");
  }

}

int main(int argc, char **argv)
{

  pidParameters.resize(3*3);

  ros::init(argc, argv, "gaitController");
  ros::NodeHandle n;
  sleep(1);

  // Initialize services
  ros::ServiceClient get_gait_evaluation_client = n.serviceClient<dyret_common::GetGaitEvaluation>("get_gait_evaluation");
  ros::ServiceClient inverseKinematicsService_client = n.serviceClient<dyret_common::CalculateInverseKinematics>("calculate_inverse_kinematics");
  ros::ServiceServer gaitControllerStatusService_server = n.advertiseService("get_gait_controller_status", getGaitControllerStatusService);
  // Initialize topics
  ros::Subscriber actionMessages_sub = n.subscribe("actionMessages", 100, actionMessagesCallback);
  ros::Subscriber servoStates_sub = n.subscribe("/dyret/servoStates", 1, servoStatesCallback);
  ros::Publisher  dynCommands_pub = n.advertise<dyret_common::Pose>("/dyret/dynCommands", 3);
  ros::Publisher  gaitInferredPos_pub = n.advertise<dyret_common::DistAng>("gaitInferredPos", 1000);
  ros::Publisher  servoConfig_pub = n.advertise<dyret_common::ServoConfigArray>("/dyret/servoConfigs", 10);

  sleep(1);
  //waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
  //waitForRosInit(inverseKinematicsService_client, "inverseKinematicsService");
  //waitForRosInit(actionMessages_sub, "actionMessages");
  //waitForRosInit(servoStates_sub, "servoStates");

  // Initialize dynamic reconfiguration:
  dynamic_reconfigure::Server<robo_cont::gaitControllerParamsConfig> gaitControllerParamsConfigServer;
  dynamic_reconfigure::Server<robo_cont::gaitControllerParamsConfig>::CallbackType gaitControllerParamsConfigFunction;
  gaitControllerParamsConfigFunction = boost::bind(&gaitControllerParamConfigCallback, _1, _2);
  gaitControllerParamsConfigServer.setCallback(gaitControllerParamsConfigFunction);

  // Initialize bSplineGait
  const float groundHeight = -450.0;
  //const float groundHeight = -562.0; // Tallest

  globalStepLength     = 150.0;
  globalStepHeight     = 35.0;
  globalSmoothing      = 25.0;
  globalGaitFrequency  = 1.0;
  globalWagAmplitude_x = 40.0;
  globalWagAmplitude_y = 20.0;
  globalWagPhaseOffset = 0.0;

	const float spreadAmount  =  35.0;
	const float frontOffset   =  0.0;
	const float leftOffset    =   0.0;
	const float rearLegOffset = -30.0;

	const std::vector<vec3P> restPose = {{ -spreadAmount, frontOffset, groundHeight },
	                                     {  spreadAmount, frontOffset, groundHeight },
	                                     {  spreadAmount, frontOffset, groundHeight },
	                                     { -spreadAmount, frontOffset, groundHeight }};

	BSplineGait bSplineGait = BSplineGait(globalStepHeight, globalStepLength, globalSmoothing, groundHeight, spreadAmount, frontOffset, leftOffset, rearLegOffset);
	bSplineGait.enableWag(0.83f, 40.0f, 0.0f);

  IncPoseAdjuster bSplineInitAdjuster(false, add(bSplineGait.getPosition(0.0, true), bSplineGait.getGaitWagPoint(0.0)), &servoAnglesInRad, inverseKinematicsService_client, dynCommands_pub);

  FILE * gaitLogGlobal;
  gaitLogGlobal = fopen("gaitLogGlobal.csv", "w");

  FILE * wagLog;
  wagLog = fopen("wagLog.csv", "w");
//  fprintf(wagLog,"L0z, L1z, L2z, L3_z, wag_commanded_x, wag_commanded_y, L0_x, L1_x, L2_x, L3_x\n");

  IncPoseAdjuster restPoseAdjuster(false, restPose, &servoAnglesInRad, inverseKinematicsService_client, dynCommands_pub);

  bool printedPos = false; // DEBUG

  int lastAction = dyret_common::ActionMessage::t_idle;
  currentAction  = dyret_common::ActionMessage::t_idle;

  setServoSpeeds(0.08, servoConfig_pub);
  sleep(1);
  moveAllLegsToGlobal(restPose, inverseKinematicsService_client, dynCommands_pub);
    ROS_ERROR("Moved all legs to global restpose");
  restPoseAdjuster.skip();

  const double poseAdjustSpeed = 0.08;
  const double gaitServoSpeed = 0.0; // max rpm

  ros::Rate loop_rate(3);

  long long startTime;
  bool activatedRecording = false;
  std::vector<vec3P> lastGlobalLegPositions;

  while ( ros::ok() ) {
      if (currentAction == dyret_common::ActionMessage::t_idle){
          //loop_rate.sleep();

          moveAllLegsToGlobal(restPose, inverseKinematicsService_client, dynCommands_pub);

      }else if (currentAction == dyret_common::ActionMessage::t_restPose){
          // Check for transition
          if (lastAction == dyret_common::ActionMessage::t_contGait){

              setServoLog(false, servoConfig_pub);
              pauseGaitRecording(get_gait_evaluation_client);

              activatedRecording = false;

              setServoSpeeds(poseAdjustSpeed, servoConfig_pub);
              restPoseAdjuster.reset();
          }

          if (restPoseAdjuster.done() == false){
              restPoseAdjuster.Spin();
          } else {
              currentAction = dyret_common::ActionMessage::t_idle;
          }

      }else if (currentAction == dyret_common::ActionMessage::t_contGait){

          // Check for transition
          if (lastAction != dyret_common::ActionMessage::t_contGait){
              setServoSpeeds(poseAdjustSpeed, servoConfig_pub);
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
              setServoPIDs(pidParameters, servoConfig_pub);

              // Gait params:
              globalStepHeight     = lastGaitControllerParamsConfigMessage.stepHeight;
              globalStepLength     = lastGaitControllerParamsConfigMessage.stepLength;
              globalSmoothing      = lastGaitControllerParamsConfigMessage.smoothing;
              globalGaitFrequency  = lastGaitControllerParamsConfigMessage.gaitFrequency;
              globalGaitSpeed      = lastGaitControllerParamsConfigMessage.gaitSpeed;
              globalWagAmplitude_x = lastGaitControllerParamsConfigMessage.wagAmplitude_x;
              globalWagAmplitude_y = lastGaitControllerParamsConfigMessage.wagAmplitude_y;
              globalWagPhaseOffset = lastGaitControllerParamsConfigMessage.wagPhase;

              bSplineGait = BSplineGait(globalStepHeight, globalStepLength, globalSmoothing, groundHeight, spreadAmount, frontOffset, leftOffset, rearLegOffset);
              bSplineGait.enableWag(0.83f+globalWagPhaseOffset, globalWagAmplitude_x, globalWagAmplitude_y);

              if ((std::isnan(globalGaitSpeed) == true) && (std::isnan(globalGaitFrequency) == true)){
                  ROS_ERROR("GlobalGaitSpeed or globalGaitFrequency has to be set\n");
              } else if (std::isnan(globalGaitFrequency) == true){
                  // Frequency has not been set => Calculate frequency from speed

                  globalGaitFrequency = globalGaitSpeed / ( (bSplineGait.getStepLength() / bSplineGait.getGndContactPercent() ) * (60.0 / 1000.0));

                  ROS_INFO("globalGaitFrequency set to %.2f\n", globalGaitFrequency);
              }

              float calculatedSpeed = (bSplineGait.getStepLength() / bSplineGait.getGndContactPercent()) * globalGaitFrequency * (60.0/1000.0);

              ROS_INFO("Calculated speed: %.2f\n", calculatedSpeed);

              dyret_common::DistAng posChangeMsg;
                              posChangeMsg.distance = calculatedSpeed;
                              posChangeMsg.msgType  = posChangeMsg.t_measurementCalculated;
                              posChangeMsg.angle    = 0.0f;
                              gaitInferredPos_pub.publish(posChangeMsg);

              if (fabs(lastActionMessage->direction - M_PI) < 0.1){
                movingForward = false;
              } else {
                movingForward = true;
              }

              vec3P wagPoint = bSplineGait.getGaitWagPoint(0.0);
              if (movingForward == false) wagPoint.points[0] = -wagPoint.points[0];

              std::vector<vec3P> initPose = lockToZ(add(bSplineGait.getPosition(0.0, movingForward), wagPoint), groundHeight);
              bSplineInitAdjuster.setGoalPose(initPose);

          }

          if (bSplineInitAdjuster.done() == false){
              if (bSplineInitAdjuster.Spin() == true){
                  setServoSpeeds(gaitServoSpeed, servoConfig_pub);
              }

              startTime = std::chrono::duration_cast< std::chrono::milliseconds > (system_clock::now().time_since_epoch()).count();
          } else {

            if (activatedRecording == false){
                startGaitRecording(get_gait_evaluation_client);
                setServoLog(true, servoConfig_pub);
                activatedRecording = true;
            }

            // Get leg positions:
            double currentRelativeTime = (std::chrono::duration_cast< milliseconds >
                (system_clock::now().time_since_epoch()).count()) - startTime; // Given in milliseconds
            std::vector<vec3P> globalLegPositions(4);

            globalLegPositions = bSplineGait.getPosition(currentRelativeTime * globalGaitFrequency, movingForward);

            if (lastGlobalLegPositions.size() == 0){
                // Not initialized
                lastGlobalLegPositions.resize(4);
            } else {
                // Initialized:

                double groundContactHeight = round(fmin(fmin(globalLegPositions[0].z(), globalLegPositions[1].z()),fmin(globalLegPositions[2].z(), globalLegPositions[3].z())));

                double distanceCovered = 0;
                int    pointCounter = 0;

                for (int i = 0; i < 4; i++){
                    // Forward motion is given in the global Y axis:
                    if ((abs(globalLegPositions[i].z() - groundContactHeight) < 0.1) && (abs(lastGlobalLegPositions[i].z() - groundContactHeight) < 0.1)){
                        // Will fail if the loop is not quick enough (< quarter walk period)

                        distanceCovered += globalLegPositions[i].y() - lastGlobalLegPositions[i].y();
                        pointCounter++;
                    }
                }

                dyret_common::DistAng posChangeMsg;
                posChangeMsg.distance = -(distanceCovered/pointCounter);
                posChangeMsg.msgType = posChangeMsg.t_measurementInferred;
                posChangeMsg.angle    = 0.0f;
                gaitInferredPos_pub.publish(posChangeMsg);
            }

            lastGlobalLegPositions = globalLegPositions;

            dyret_common::Pose msg;

            std::vector<int> servoIds(12);
            std::vector<double> anglesInRad(12);
            int currentActuatorIndex = 0;

            vec3P wag = bSplineGait.getGaitWagPoint(currentRelativeTime * globalGaitFrequency);

            if (movingForward == false){
                wag.points[0] = -wag.points[0];
                wag.points[1] = -wag.points[1];
            }

            std::vector<vec3P> currentPositions = currentLegPositions(servoAnglesInRad);

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
            	dyret_common::CalculateInverseKinematics srv;

                vec3P localLegPosition = calculateLocalPosition(i, add(globalLegPositions[i], wag));

                srv.request.point.x = localLegPosition.x();
                srv.request.point.y = localLegPosition.y();
                srv.request.point.z = localLegPosition.z();

                if (inverseKinematicsService_client.call(srv)) {
                    // Handle invertions (ids 1, 5, 8, 10):
                    if (i == 0 || i == 3){
                        for (int j = 0; j < srv.response.solutions.size(); j++){
                            srv.response.solutions[j].anglesInRad[1] = -srv.response.solutions[j].anglesInRad[1];
                        }
                    }else if (i == 1 || i == 2){
                        for (int j = 0; j < srv.response.solutions.size(); j++){
                            srv.response.solutions[j].anglesInRad[2] = -srv.response.solutions[j].anglesInRad[2];
                        }
                    }

              if (srv.response.solutions.size() != 0){
                if (servoIds.size() == 0) servoIds.resize(12);

                // Successful in getting solution
                for (int j = 0; j < 3; j++){ // For each joint in the leg
                  servoIds[currentActuatorIndex] = currentActuatorIndex; // Set ids
                  if (i == 0 || i == 1){
                      anglesInRad[currentActuatorIndex++] = normalizeRad(srv.response.solutions[1].anglesInRad[j]);
                  }else{
                      anglesInRad[currentActuatorIndex++] = normalizeRad(srv.response.solutions[0].anglesInRad[j]);
                  }
                }
              } else {
                servoIds.clear();
              }

            } else {
              ROS_ERROR("Failed to call inverse kinematics service");
            }
        }

        std::vector<vec3P> actual = currentLegPositions(servoAnglesInRad);

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

        if (servoIds.size() != 0){
            msg.id = servoIds;
            msg.angle = anglesInRad;
            dynCommands_pub.publish(msg);
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

  fclose(wagLog);
  fclose(gaitLogGlobal);

  ROS_INFO("Exiting gaitController");

  return 0;
}
