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

#include "gait/BSplineGait.h"
#include "gait/wagGenerator.h"

#include "kinematics/movementFunctions.h"
#include "kinematics/kinematicFunctions.h"
#include "kinematics/forwardKinematics.h"
#include "kinematics/inverseKinematics.h"
#include "kinematics/interpolation.h"

#include "kinematics/IncPoseAdjuster.h"

#include "dyret_common/wait_for_ros.h"

#include "dyret_common/Configure.h"

#include "dyret_controller/PositionCommand.h"
#include "dyret_controller/ActionMessage.h"
#include "dyret_controller/DistAngMeasurement.h"

#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GetGaitEvaluation.h"

#include "dyret_controller/gaitControllerParamsConfig.h"

#include <dyret_hardware/ActuatorBoardState.h>

using namespace std::chrono;

unsigned char currentAction;
dyret_controller::ActionMessage::ConstPtr lastActionMessage;
robo_cont::gaitControllerParamsConfig lastGaitControllerParamsConfigMessage;

bool initAdjustInSim = true;

bool movingForward;
double globalGaitFrequency;
double globalGaitSpeed;
double globalLiftDuration;
const double groundHeightOffset = -430;
const double groundCorrectionFactor = 0.8;
float groundHeight = -430.0f;
std::vector<double> legActuatorLengths;

const float bSplineGaitWagOffset = 0.91;

const float spreadAmount = 80.0; // was 50

std::vector<double> pidParameters;

std::vector<double> servoAnglesInRad(12);
std::vector<double> prismaticPositions(8);

bool getGaitControllerStatusService(dyret_controller::GetGaitControllerStatus::Request &req,
                                    dyret_controller::GetGaitControllerStatus::Response &res) {
    res.gaitControllerStatus.actionType = currentAction;

    return true;
}

void actionMessagesCallback(const dyret_controller::ActionMessage::ConstPtr &msg) {
    ROS_INFO("Switching currentAction from %u to %u", currentAction, msg->actionType);
    lastActionMessage = msg;
    currentAction = msg->actionType;

    ROS_INFO("Direction is %.2f (%.2f)", lastActionMessage->direction, msg->direction);
}

void servoStatesCallback(const dyret_common::State::ConstPtr &msg) {
    for (int i = 0; i < 12; i++) {
        servoAnglesInRad[i] = msg->revolute[i].position;
    }

    for (int i = 0; i < prismaticPositions.size(); i++) {
        prismaticPositions[i] = msg->prismatic[i].position;
    }
    legActuatorLengths[0] =
            (prismaticPositions[0] + prismaticPositions[2] + prismaticPositions[4] + prismaticPositions[6]) / 4.0f;
    legActuatorLengths[1] =
            (prismaticPositions[1] + prismaticPositions[3] + prismaticPositions[5] + prismaticPositions[7]) / 4.0f;

    groundHeight = (float) (groundHeightOffset - ((legActuatorLengths[0] + legActuatorLengths[1]) * groundCorrectionFactor));
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

    ROS_INFO("\n\tC: P%.2f I%.2f D%.2f\n\tF: P%.2f I%.2f D%.2f\n\tT: P%.2f I%.2f D%.2f\n\n",
             config.cP, config.cI, config.cD, config.fP, config.fI, config.fD, config.tP, config.tI, config.tD);

    lastGaitControllerParamsConfigMessage = config;

}

void startGaitRecording(ros::ServiceClient get_gait_evaluation_client) {

    dyret_controller::GetGaitEvaluation srv;

    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_start;

    if (get_gait_evaluation_client.call(srv)) {
        ROS_INFO("Called startGaitRecording service\n");
    }

}

void pauseGaitRecording(ros::ServiceClient get_gait_evaluation_client) {

    dyret_controller::GetGaitEvaluation srv;

    srv.request.givenCommand = dyret_controller::GetGaitEvaluation::Request::t_pause;

    if (get_gait_evaluation_client.call(srv)) {
        ROS_INFO("Called pauseGaitRecording service\n");
    }

}

std::vector<vec3P> getRestPose() {
    const float frontOffset = 0.0;

    const std::vector<vec3P> restPose = {{-spreadAmount, frontOffset, groundHeight},
                                         {spreadAmount,  frontOffset, groundHeight},
                                         {spreadAmount,  frontOffset, groundHeight},
                                         {-spreadAmount, frontOffset, groundHeight}};

    return restPose;
}

int main(int argc, char **argv) {

    pidParameters.resize(3 * 3);

    ros::init(argc, argv, "gaitController");
    ros::NodeHandle n;

    // Initialize services
    ros::ServiceClient get_gait_evaluation_client = n.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
    ros::ServiceServer gaitControllerStatusService_server = n.advertiseService("get_gait_controller_status", getGaitControllerStatusService);
    // Initialize topics
    ros::Subscriber actionMessages_sub = n.subscribe("/dyret/dyret_controller/actionMessages", 100, actionMessagesCallback);
    ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);
    ros::Publisher poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 3);
    ros::Publisher gaitInferredPos_pub = n.advertise<dyret_controller::DistAngMeasurement>("/dyret/dyret_controller/gaitInferredPos", 1000);
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

    legActuatorLengths = {0.0, 0.0};

    // Initialize bSplineGait
    globalGaitFrequency = 1.0;
    globalLiftDuration = 0.125;

    const float frontOffset = 0.0f;
    const float rearLegOffset = -30.0f;

    BSplineGait bSplineGait;
    WagGenerator wagGenerator;

    IncPoseAdjuster restPoseAdjuster(&servoAnglesInRad,
                                     &legActuatorLengths,
                                     positionCommand_pub);
    restPoseAdjuster.setPose(getRestPose());
    restPoseAdjuster.skip();

    int lastAction = dyret_controller::ActionMessage::t_idle;
    currentAction = dyret_controller::ActionMessage::t_idle;

    if (ros::Time::isSystemTime()) { // do not set servo speed in simulation
        setServoSpeeds(0.01, servoConfigClient);
    }

    if (ros::Time::isSystemTime()) { // do not set pid in simulation
        printf("P: %.2f, I: %.2f, D: %.2f\n", lastGaitControllerParamsConfigMessage.cP,
               lastGaitControllerParamsConfigMessage.cI, lastGaitControllerParamsConfigMessage.cD);
        pidParameters[0] = 2.0; // Set to 2 to stop coxa shaking before experiment begins
        pidParameters[1] = lastGaitControllerParamsConfigMessage.cI;
        pidParameters[2] = lastGaitControllerParamsConfigMessage.cD;
        pidParameters[3] = lastGaitControllerParamsConfigMessage.fP;
        pidParameters[4] = lastGaitControllerParamsConfigMessage.fI;
        pidParameters[5] = lastGaitControllerParamsConfigMessage.fD;
        pidParameters[6] = lastGaitControllerParamsConfigMessage.tP;
        pidParameters[7] = lastGaitControllerParamsConfigMessage.tI;
        pidParameters[8] = lastGaitControllerParamsConfigMessage.tD;
        setServoPIDs(pidParameters, servoConfigClient);
    }

    std::vector<vec3P> restPose = getRestPose();
    moveAllLegsToGlobalPosition(getRestPose(), positionCommand_pub);

    const double poseAdjustSpeed = 0.08;
    const double gaitServoSpeed = 0.0;

    ros::Rate loop_rate(5);

    ros::Time startTime;
    ros::Time lastTime;
    bool lastTimeValid = false;
    bool activatedRecording = false;
    ros::Rate poseAdjusterRate(50);
    ros::Rate gaitRate(100);

    IncPoseAdjuster gaitInitAdjuster(
            &servoAnglesInRad,
            &legActuatorLengths,
            positionCommand_pub);

    while (ros::ok()) {
        if (currentAction == dyret_controller::ActionMessage::t_sleep) {

        } else if (currentAction == dyret_controller::ActionMessage::t_idle) {
            loop_rate.sleep();

            // Check for transition
            if (lastAction == dyret_controller::ActionMessage::t_contGait) {
                if (ros::Time::isSystemTime()) setServoLog(false, servoConfigClient);
                pauseGaitRecording(get_gait_evaluation_client);

                activatedRecording = false;

                if (ros::Time::isSystemTime()) setServoSpeeds(poseAdjustSpeed, servoConfigClient);
            }

        } else if (currentAction == dyret_controller::ActionMessage::t_restPose) {

            // Check for transition from walking
            if (lastAction == dyret_controller::ActionMessage::t_contGait) {

                if (ros::Time::isSystemTime()) setServoLog(false, servoConfigClient);
                pauseGaitRecording(get_gait_evaluation_client);

                activatedRecording = false;

            }

            // Check for transition from anything
            if (lastAction != dyret_controller::ActionMessage::t_restPose){
                if (ros::Time::isSystemTime()) setServoSpeeds(poseAdjustSpeed, servoConfigClient);

                if (ros::Time::isSimTime() && !initAdjustInSim) {
                    moveAllLegsToGlobalPosition(getRestPose(), positionCommand_pub);
                    restPoseAdjuster.skip();
                    ros::Duration(1).sleep();
                    startTime = ros::Time::now();
                } else {
                    restPoseAdjuster.setPose(getRestPose());
                    restPoseAdjuster.reset();
                }
            }

            if (restPoseAdjuster.done() == false) {
                restPoseAdjuster.Spin();
                poseAdjusterRate.sleep();

            } else {
                currentAction = dyret_controller::ActionMessage::t_idle;
            }

        } else if (currentAction == dyret_controller::ActionMessage::t_contGait) {

            gaitRate.sleep();

            // Check for transition
            if (lastAction != dyret_controller::ActionMessage::t_contGait) {
                if (ros::Time::isSystemTime()) setServoSpeeds(poseAdjustSpeed, servoConfigClient);
                gaitInitAdjuster.reset();

                // PID:
                if (ros::Time::isSystemTime()) { // do not set pid in simulation
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
                }

                // Gait params:
                globalGaitFrequency = lastGaitControllerParamsConfigMessage.gaitFrequency;
                globalGaitSpeed = lastGaitControllerParamsConfigMessage.gaitSpeed;
                globalLiftDuration = lastGaitControllerParamsConfigMessage.liftDuration;

                bSplineGait.initGait(lastGaitControllerParamsConfigMessage.stepHeight,
                                     lastGaitControllerParamsConfigMessage.stepLength,
                                     lastGaitControllerParamsConfigMessage.smoothing,
                                     groundHeight,
                                     spreadAmount,
                                     frontOffset,
                                     rearLegOffset,
                                     globalLiftDuration);

                wagGenerator.enableWag(bSplineGaitWagOffset + lastGaitControllerParamsConfigMessage.wagPhase,
                                       lastGaitControllerParamsConfigMessage.wagAmplitude_x,
                                       lastGaitControllerParamsConfigMessage.wagAmplitude_y);

                gaitInitAdjuster.setPose(add(bSplineGait.getPosition(0.0, true), wagGenerator.getGaitWagPoint(0.0, true)));

                if ((std::isnan(globalGaitSpeed) == true) && (std::isnan(globalGaitFrequency) == true)) {
                    ROS_ERROR("GlobalGaitSpeed or globalGaitFrequency has to be set\n");
                } else if (std::isnan(globalGaitFrequency) == true) {
                    // Frequency has not been set => Calculate frequency from speed

                    ROS_FATAL("std::isnan(globalGaitFrequency) == true\n");
                    exit(-1);
                }

                if (fabs(lastActionMessage->direction - M_PI) < 0.1) {
                    movingForward = false;
                } else {
                    movingForward = true;
                }

                // Calculate the initial pose of the gait
                std::vector<vec3P> initGaitPose = lockToZ(add( bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward) ),
                                                          groundHeight);

                // Adjust to the start of the gait only in the real world or if the initAdjustInSim bool is set for testing
                if (ros::Time::isSimTime() && !initAdjustInSim) {
                    moveAllLegsToGlobalPosition(initGaitPose, positionCommand_pub);
                    gaitInitAdjuster.skip();
                    ros::Duration(1).sleep();
                    startTime = ros::Time::now();
                } else {
                    gaitInitAdjuster.setPose(initGaitPose);
                }

                lastTimeValid = false;

            }

            if (gaitInitAdjuster.done() == false) {

                if (ros::Time::isSystemTime() || initAdjustInSim) {
                    if (gaitInitAdjuster.Spin() == true) {
                        if (ros::Time::isSystemTime()) setServoSpeeds(gaitServoSpeed, servoConfigClient);
                    }
                }

                poseAdjusterRate.sleep();

                startTime = ros::Time::now();
            } else {

                if (activatedRecording == false) {
                    startGaitRecording(get_gait_evaluation_client);
                    if (ros::Time::isSystemTime()) setServoLog(true, servoConfigClient);
                    activatedRecording = true;
                }

                // Get leg positions:
                double currentRelativeTime =
                        (ros::Time::now() - startTime).toNSec() / 1000000.0; // Given in milliseconds
                std::vector<vec3P> globalLegPositions(4);

                globalLegPositions = bSplineGait.getPosition(currentRelativeTime * globalGaitFrequency, movingForward);

                // Calculate posChangeMsg:
                if (lastTimeValid == false) {
                    // Not initialized
                    lastTime = ros::Time::now();
                    lastTimeValid = true;
                } else {
                    // Initialized:

                    double secondsPassed = (ros::Time::now() - lastTime).toNSec() / 1000000000.0f;
                    double distance = bSplineGait.getStepLength() * globalGaitFrequency * (secondsPassed);

                    dyret_controller::DistAngMeasurement posChangeMsg;
                    if (movingForward) posChangeMsg.distance = distance; else posChangeMsg.distance = -distance;

                    posChangeMsg.msgType = posChangeMsg.t_measurementInferred;
                    posChangeMsg.angle = 0.0f;
                    gaitInferredPos_pub.publish(posChangeMsg);
                }

                lastTime = ros::Time::now();

                dyret_common::Pose msg;

                std::vector<int> servoIds(12);
                for (int i = 0; i < servoIds.size(); i++) servoIds[i] = i;

                std::vector<float> anglesInRad;

                vec3P wag = wagGenerator.getGaitWagPoint(currentRelativeTime * globalGaitFrequency, movingForward);

                std::vector<vec3P> currentPositions = currentLegPositions(servoAnglesInRad, legActuatorLengths);

                // Get IK solutions for each leg:
                for (int i = 0; i < 4; i++) { // For each leg

                    vec3P legPosition = add(globalLegPositions[i], wag);

                    std::vector<double> inverseReturn = inverseKinematics::calculateInverseKinematics(legPosition.x(),
                                                                                                      legPosition.y(),
                                                                                                      legPosition.z(),
                                                                                                      i,
                                                                                                      legActuatorLengths[0],
                                                                                                      legActuatorLengths[1]);
                    for (int j = 0; j < 3; j++) {
                        anglesInRad.push_back(inverseReturn[j]);
                    }

                }

                if (servoIds.size() != 0) {
                    msg.revolute = anglesInRad;
                    poseCommand_pub.publish(msg);
                } else {
                    ROS_WARN("Did not send invalid dyn commands!\n");
                }
            }
        } else {
            ROS_FATAL("Undefined action!");
            exit(-1);
        }

        lastAction = currentAction; // Save last action to handle transitions
        ros::spinOnce();

    }

    ROS_INFO("Exiting gaitController");

    return 0;
}
