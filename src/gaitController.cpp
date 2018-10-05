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
#include "dyret_controller/GaitConfiguration.h"

#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GetGaitEvaluation.h"

#include <dyret_hardware/ActuatorBoardState.h>

using namespace std::chrono;

unsigned char currentAction;
dyret_controller::ActionMessage::ConstPtr lastActionMessage;

std::string gaitType;
std::map<std::string, float> gaitConfiguration;

// This decides if the leg adjustment done in hardware is done in sim as well
bool initAdjustInSim = true;

bool movingForward;
double globalGaitFrequency;
double frequencyFactor;
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

void gaitConfigurationCallback(const dyret_controller::GaitConfiguration::ConstPtr &msg) {

    ROS_INFO("Got gait configuration message for gait type %s", msg->gaitName.c_str());
    gaitType = msg->gaitName;

    gaitConfiguration.clear();
    assert(msg->parameterName.size() == msg->parameterValue.size());
    for (size_t i = 0; i < msg->parameterName.size(); ++i)
        gaitConfiguration[msg->parameterName[i]] = msg->parameterValue[i];

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

void sendGaitInferredPos(ros::Publisher givenGaitInferredPos_pub, float givenDistance){
    dyret_controller::DistAngMeasurement posChangeMsg;
    posChangeMsg.msgType = posChangeMsg.t_measurementInferred;
    posChangeMsg.absoluteMeasurement = true;
    posChangeMsg.distance = givenDistance;
    posChangeMsg.angle = 0.0f;

    givenGaitInferredPos_pub.publish(posChangeMsg);
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
    ros::Subscriber gaitConfiguration_sub = n.subscribe("/dyret/dyret_controller/gaitConfiguration", 1, gaitConfigurationCallback);
    ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);
    ros::Publisher poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 3);
    ros::Publisher gaitInferredPos_pub = n.advertise<dyret_controller::DistAngMeasurement>("/dyret/dyret_controller/gaitInferredPos", 1000);
    ros::ServiceClient servoConfigClient = n.serviceClient<dyret_common::Configure>("/dyret/configuration");
    ros::Publisher positionCommand_pub = n.advertise<dyret_controller::PositionCommand>("/dyret/dyret_controller/positionCommand", 1);;

    waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
    waitForRosInit(actionMessages_sub, "/dyret/dyret_controller/actionMessages");
    waitForRosInit(servoStates_sub, "servoStates");

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

    std::vector<vec3P> restPose = getRestPose();
    moveAllLegsToGlobalPosition(getRestPose(), positionCommand_pub);

    const double poseAdjustSpeed = 0.08;
    const double gaitServoSpeed = 0.0;

    ros::Rate loop_rate(5);

    ros::Time startTime;
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

                if (fabs(lastActionMessage->direction - M_PI) < 0.1) {
                    movingForward = false;
                } else {
                    movingForward = true;
                }


                if (gaitType == "highLevelSplineGait") {
                    // Gait params:
                    frequencyFactor = gaitConfiguration.at("frequencyFactor");
                    globalLiftDuration = gaitConfiguration.at("liftDuration");
                    fprintf(stderr, "1\n");
                    bSplineGait.initHighLevelGait(gaitConfiguration.at("stepHeight"),
                                                  gaitConfiguration.at("stepLength"),
                                                  gaitConfiguration.at("smoothing"),
                                                  groundHeight,
                                                  spreadAmount,
                                                  frontOffset,
                                                  rearLegOffset,
                                                  globalLiftDuration);

                    wagGenerator.enableWag(bSplineGaitWagOffset + gaitConfiguration.at("wagPhase"),
                                           gaitConfiguration.at("wagAmplitude_x"),
                                           gaitConfiguration.at("wagAmplitude_y"));

                } else if (gaitType == "lowLevelSplineGait"){

                    frequencyFactor = gaitConfiguration.at("frequencyFactor");
                    globalLiftDuration = gaitConfiguration.at("liftDuration");

                    bSplineGait.initLowLevelGait(gaitConfiguration, groundHeight);
                    if (movingForward) bSplineGait.writeGaitToFile();

                    wagGenerator.enableWag(0.0, 0.0, 0.0);

                } else {
                    ROS_FATAL("Unknown gait specified: %s!", gaitType.c_str());
                    exit(-1);
                }

                // Limit frequency so speed is below 10m/min:
                double maxFrequency = ((10.0/60.0)*1000.0) / bSplineGait.getStepLength();
                globalGaitFrequency = maxFrequency * frequencyFactor;

                gaitInitAdjuster.setPose(add(bSplineGait.getPosition(0.0, true), wagGenerator.getGaitWagPoint(0.0, true)));

                // Calculate the initial pose of the gait
                std::vector<vec3P> initialGaitPose = lockToZ(add( bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward) ),
                                                          groundHeight);

                // Adjust to the start of the gait only in the real world or if the initAdjustInSim bool is set for testing
                if (ros::Time::isSimTime() && !initAdjustInSim) {
                    moveAllLegsToGlobalPosition(initialGaitPose, positionCommand_pub);
                    gaitInitAdjuster.skip();
                    ros::Duration(1).sleep();
                    startTime = ros::Time::now();
                } else {
                    gaitInitAdjuster.setPose(initialGaitPose);
                }

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

                double secondsPassed = (ros::Time::now() - startTime).toNSec() / 1000000000.0f;
                float distance = (float) (bSplineGait.getStepLength() * globalGaitFrequency * (secondsPassed));

                dyret_controller::DistAngMeasurement posChangeMsg;
                if (movingForward) posChangeMsg.distance = distance; else posChangeMsg.distance = -distance;

                posChangeMsg.msgType = posChangeMsg.t_measurementInferred;
                posChangeMsg.angle = 0.0f;
                posChangeMsg.absoluteMeasurement = true;
                gaitInferredPos_pub.publish(posChangeMsg);

                // Generate and send pose message
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
