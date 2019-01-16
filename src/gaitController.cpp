#include <math.h>
#include <vector>
#include <chrono>
#include <iostream>

#include "dyret_common/angleConv.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
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
#include "dyret_controller/DistAngMeasurement.h"
#include "dyret_controller/ConfigureGait.h"

#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/SendPositionCommand.h"
#include "dyret_controller/GetInferredPosition.h"
#include "dyret_controller/GaitControllerCommandService.h"

#include "dyret_controller/LoggerCommand.h"

#include <dyret_hardware/ActuatorBoardState.h>

#include <std_srvs/Empty.h>

using namespace std::chrono;

unsigned char currentAction;

// Config:
const double poseAdjustSpeed = 0.0f;
const double gaitSpeed = 0.0f;
const float frontOffset = 0.0f;
const float rearLegOffset = -30.0f;

bool walking = false;

std::string gaitType;
std::map<std::string, float> gaitConfiguration;

// This decides if the leg adjustment done in hardware is done in sim as well
bool initAdjustInSim = true;

double currentInferredPosition = 0.0;
bool movingForward = true;
double globalGaitFrequency;
double globalLiftDuration;
const double groundHeightOffset = -430;
const double groundCorrectionFactor = 0.8;
float groundHeight = -430.0f;
std::vector<double> legActuatorLengths;

float receivedFemurLength = 0.0;
float receivedTibiaLength = 0.0;

const float bSplineGaitWagOffset = 0.91;

const float spreadAmount = 80.0; // was 50

bool activatedRecording;

std::vector<vec3P> startGaitPose;

std::vector<double> pidParameters;

std::vector<double> servoAnglesInRad(12);
std::vector<double> prismaticPositions(8);

// Public stuff
ros::ServiceClient get_gait_evaluation_client;
ros::ServiceClient servoConfigClient;
BSplineGait bSplineGait;
WagGenerator wagGenerator;
ros::Publisher poseCommand_pub;
ros::Time startTime;
ros::ServiceClient positionCommand_pub;
ros::ServiceClient loggerCommandService_client;

IncPoseAdjuster gaitInitAdjuster(
        &servoAnglesInRad,
        &legActuatorLengths,
        &positionCommand_pub);



bool getGaitControllerStatusService(dyret_controller::GetGaitControllerStatus::Request &req,
                                    dyret_controller::GetGaitControllerStatus::Response &res) {
    res.gaitControllerStatus.actionType = currentAction;

    return true;
}

void setLegLengths(float femurLengths, float tibiaLengths) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    msg.prismatic.resize(2);

    msg.prismatic[0] = femurLengths;
    msg.prismatic[1] = tibiaLengths;

    poseCommand_pub.publish(msg);
}

bool legsAreLength(float femurLengths, float tibiaLengths) {
    return ((fabs(femurLengths - legActuatorLengths[0]) < 1.0f) && (fabs(tibiaLengths - legActuatorLengths[1]) < 1.0f));
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

std::vector<vec3P> getRestPose() {
    const float frontOffset = 0.0;

    const std::vector<vec3P> restPose = {{-spreadAmount, frontOffset, groundHeight},
                                         {spreadAmount,  frontOffset, groundHeight},
                                         {spreadAmount,  frontOffset, groundHeight},
                                         {-spreadAmount, frontOffset, groundHeight}};

    return restPose;
}

bool inferredPositionCallback(dyret_controller::GetInferredPosition::Request  &req,
                              dyret_controller::GetInferredPosition::Response &res){

    res.currentInferredPosition.angle = 0.0;
    res.currentInferredPosition.msgType = res.currentInferredPosition.t_measurementInferred;
    res.currentInferredPosition.distance = (float) currentInferredPosition;
    res.currentInferredPosition.absoluteMeasurement = true;

    return true;
}

ros::Time lastUpdate;
void spinGaitOnce(){

    // Activate gait recording if it has not been done, is done the first time it is run
    if (activatedRecording == false) {
        activatedRecording = true;

        startTime = ros::Time::now();
        lastUpdate = ros::Time::now();
        currentInferredPosition = 0.0;
    }

    if ((ros::Time::now() - lastUpdate).toSec() >= 0.02){ //
        lastUpdate = ros::Time::now();

        // Get leg positions:
        double currentRelativeTime =
                (ros::Time::now() - startTime).toNSec() / 1000000.0; // Given in milliseconds
        std::vector<vec3P> globalLegPositions(4);

        globalLegPositions = bSplineGait.getPosition(currentRelativeTime * globalGaitFrequency, movingForward);

        // Set currentInferredPosition:
        double secondsPassed = (ros::Time::now() - startTime).toNSec() / 1000000000.0f;
        float distance = (float) (bSplineGait.getStepLength() * globalGaitFrequency * (secondsPassed));

        currentInferredPosition = distance;

        // Generate and send pose message
        dyret_common::Pose msg;
        msg.header.stamp = ros::Time().now();

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

            msg.prismatic.resize(2);
            msg.prismatic[0] = receivedFemurLength;
            msg.prismatic[1] = receivedTibiaLength;
            poseCommand_pub.publish(msg);
        } else {
            ROS_WARN("Did not send invalid dyn commands!\n");
        }
    }
}

bool adjustPose(std::vector<vec3P> givenPose){
    ros::Rate poseAdjusterRate(50);

    IncPoseAdjuster poseAdjuster(&servoAnglesInRad,
                                 &legActuatorLengths,
                                 &positionCommand_pub);

    poseAdjuster.setPose(givenPose);

    if (ros::Time::isSystemTime()){
        setServoSpeeds(poseAdjustSpeed, servoConfigClient);
        ROS_INFO("Set servo speed to %.2f", poseAdjustSpeed);
    }

    usleep(1000);

    while (!poseAdjuster.done()) {
        poseAdjuster.Spin();
        poseAdjusterRate.sleep();
    }

    return true;
}

bool adjustRestPose(){
    return adjustPose(getRestPose());
}

bool adjustGaitPose(){
    return adjustPose(startGaitPose);
}

float getMapValue(std::map<std::string, float> givenMap, std::string givenKey){
    if (givenMap.find(givenKey) == givenMap.end()) {
        ROS_ERROR("Could not find key %s", givenKey.c_str());
    }

    return givenMap.at(givenKey);
}

bool gaitConfigurationCallback(dyret_controller::ConfigureGait::Request  &req,
                               dyret_controller::ConfigureGait::Response &res) {

    ROS_INFO("Got gait configuration message for gait type %s", req.gaitConfiguration.gaitName.c_str());

    // Set leg lengths and wait until they reach the correct length
    float femurLength = req.gaitConfiguration.femurLength;
    float tibiaLength = req.gaitConfiguration.tibiaLength;

    receivedFemurLength = femurLength;
    receivedTibiaLength = tibiaLength;

    if (req.gaitConfiguration.prepareForGait && (femurLength >= 0 && tibiaLength >= 0)) {
        ROS_INFO("Setting leg lengths to %.2f and %.2f", femurLength, req.gaitConfiguration.tibiaLength);
        setLegLengths(femurLength, req.gaitConfiguration.tibiaLength);
        if (ros::Time::isSimTime()) ros::Duration(2).sleep();
    }

    // Reset vars between runs
    activatedRecording = false;
    currentInferredPosition = 0.0;

    gaitType = req.gaitConfiguration.gaitName;

    gaitConfiguration.clear();
    assert(req.gaitConfiguration.gaitParameterName.size() == req.gaitConfiguration.gaitParameterValue.size());
    for (size_t i = 0; i < req.gaitConfiguration.gaitParameterName.size(); ++i)
        gaitConfiguration[req.gaitConfiguration.gaitParameterName[i]] = req.gaitConfiguration.gaitParameterValue[i];

    gaitInitAdjuster.reset();

    if (gaitType == "highLevelSplineGait") {
        // Gait params:
        globalLiftDuration = getMapValue(gaitConfiguration, "liftDuration");

        bSplineGait.initHighLevelGait(getMapValue(gaitConfiguration, "stepHeight"),
                                      getMapValue(gaitConfiguration, "stepLength"),
                                      getMapValue(gaitConfiguration, "smoothing"),
                                      groundHeight,
                                      spreadAmount,
                                      frontOffset,
                                      rearLegOffset,
                                      globalLiftDuration);

        wagGenerator.enableWag(bSplineGaitWagOffset + getMapValue(gaitConfiguration, "wagPhase"),
                               getMapValue(gaitConfiguration, "wagAmplitude_x"),
                               getMapValue(gaitConfiguration, "wagAmplitude_y"));

    } else if (gaitType == "lowLevelSplineGait"){

        globalGaitFrequency = getMapValue(gaitConfiguration, "frequency");
        globalLiftDuration = getMapValue(gaitConfiguration, "liftDuration");

        bSplineGait.initLowLevelGait(gaitConfiguration, groundHeight);
        //if (movingForward) bSplineGait.writeGaitToFile();

        //wagGenerator.enableWag(0.0, 0.0, 0.0);
        wagGenerator.enableWag(bSplineGaitWagOffset + getMapValue(gaitConfiguration, "wagPhase"),
                               getMapValue(gaitConfiguration, "wagAmplitude_x"),
                               getMapValue(gaitConfiguration, "wagAmplitude_y"));

    } else {
        ROS_FATAL("Unknown gait specified: %s!", gaitType.c_str());
        exit(-1);
    }

    // Set initial pose for adjustment
    movingForward = req.gaitConfiguration.directionForward;

    startGaitPose = lockToZ(add(bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward)),
                            groundHeight);

    // Limit frequency so speed is below 10m/min:
    globalGaitFrequency = getMapValue(gaitConfiguration, "frequency");

    if (ros::Time::isSystemTime()){
        setServoSpeeds(poseAdjustSpeed, servoConfigClient);
        ROS_INFO("Set servo speed to %.2f", poseAdjustSpeed);
    }

    // Adjust to the start of the gait only in simulation. NOTE: THIS ASSUMES FORWARD MOVEMENT
    if (ros::Time::isSimTime() && !initAdjustInSim) {
        // Calculate the initial pose of the gait
        std::vector<vec3P> initialGaitPose = lockToZ(add( bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward) ),
                                                     groundHeight);

        moveAllLegsToGlobalPosition(initialGaitPose, &positionCommand_pub);
        gaitInitAdjuster.skip();
        sleep(1);
        startTime = ros::Time::now();
    }

    // Adjust gait pose
    if (req.gaitConfiguration.prepareForGait) {
        adjustGaitPose();
    }

    ros::Time legLengthAdjustmentStart = ros::Time::now();
    if (req.gaitConfiguration.prepareForGait && (femurLength >= 0 && tibiaLength >= 0)) {
        moveAllLegsToGlobalPosition(startGaitPose, &positionCommand_pub); // Continuously send pose to adjust to leg length change
        setLegLengths(femurLength, tibiaLength);
        ros::spinOnce();

        if (!ros::Time::isSimTime()){
            while (!legsAreLength(femurLength, tibiaLength)) {
                ros::spinOnce();
                usleep(10000);
                setLegLengths(femurLength, tibiaLength);

                int secPassed = ros::Time::now().sec - legLengthAdjustmentStart.sec;

                if (((ros::Time::isSystemTime()) && (secPassed > 90)) || (ros::Time::isSimTime() && (secPassed > 5))) {
                    ROS_ERROR("Timed out waiting for legs to be at length at %ds", secPassed);
                    return false;
                }
                secPassed += 1;
            }
            ROS_INFO("Leg lengths achieved");
        }
    }

    if (ros::Time::isSystemTime()){
        setServoSpeeds(gaitSpeed, servoConfigClient);
        ROS_INFO("Set servo speed to %.2f", gaitSpeed);
    }

    return true;
}

bool gaitControllerCommandCallback(dyret_controller::GaitControllerCommandService::Request  &req,
                                   dyret_controller::GaitControllerCommandService::Response &res) {
    if (req.gaitControllerCommand.gaitControllerCommand == req.gaitControllerCommand.t_spinOnce){
        spinGaitOnce();
    } else if (req.gaitControllerCommand.gaitControllerCommand == req.gaitControllerCommand.t_adjustRestPose){
        ROS_INFO("Adjusting to restpose");
        return adjustRestPose();
    } else if (req.gaitControllerCommand.gaitControllerCommand == req.gaitControllerCommand.t_adjustGaitPose) {
        ROS_INFO("Adjusting to gaitpose");
        return adjustGaitPose();
    } else if (req.gaitControllerCommand.gaitControllerCommand == req.gaitControllerCommand.t_startWalking) {
        ROS_INFO("Starting to walk");
        walking = true;
    } else if (req.gaitControllerCommand.gaitControllerCommand == req.gaitControllerCommand.t_stopWalking) {
        ROS_INFO("Stopping");
        walking = false;
    } else {
        ROS_ERROR("Unknown gaitControllerCommand");
    }

    return true;
}

bool restPoseServiceCallback(std_srvs::Empty::Request  &req,
                             std_srvs::Empty::Response &res) {
    ROS_INFO("Adjusting to restpose");
    return adjustRestPose();
}

int main(int argc, char **argv) {

    pidParameters.resize(3 * 3);

    ros::init(argc, argv, "gaitController");
    ros::NodeHandle n;

    // Initialize services
    ros::ServiceServer gaitConfigurationServer = n.advertiseService("/dyret/dyret_controller/gaitConfigurationService", gaitConfigurationCallback);
    ros::ServiceServer gaitControllerStatusService_server = n.advertiseService("get_gait_controller_status", getGaitControllerStatusService);
    ros::ServiceServer inferredPositionServer = n.advertiseService("/dyret/dyret_controller/getInferredPosition", inferredPositionCallback);
    ros::ServiceServer gaitControllerAction = n.advertiseService("/dyret/dyret_controller/gaitControllerCommandService", gaitControllerCommandCallback);
    ros::ServiceServer restPoseService = n.advertiseService("/dyret/dyret_controller/restPoseService", restPoseServiceCallback);

    loggerCommandService_client = n.serviceClient<dyret_controller::LoggerCommand>("/dyret/dyret_logger/loggerCommand");

    get_gait_evaluation_client = n.serviceClient<dyret_controller::GetGaitEvaluation>("get_gait_evaluation");
    servoConfigClient = n.serviceClient<dyret_common::Configure>("/dyret/configuration");
    positionCommand_pub = n.serviceClient<dyret_controller::SendPositionCommand>("/dyret/dyret_controller/positionCommandService");

    ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);

    poseCommand_pub = n.advertise<dyret_common::Pose>("/dyret/command", 3);

    waitForRosInit(get_gait_evaluation_client, "get_gait_evaluation");
    waitForRosInit(servoStates_sub, "servoStates");

    legActuatorLengths = {0.0, 0.0};

    // Initialize bSplineGait
    globalGaitFrequency = 1.0;
    globalLiftDuration = 0.125;

    /*if (ros::Time::isSystemTime()) { // do not set servo speed in simulation
        setServoSpeeds(0.01, servoConfigClient);
    }

    std::vector<vec3P> restPose = getRestPose();
    moveAllLegsToGlobalPosition(getRestPose(), &positionCommand_pub);*/

    ros::Rate loop_rate(5);
    ros::Rate gaitRate(50);

    activatedRecording = false;
    ros::Rate poseAdjusterRate(50);

    while (ros::ok()) {
        if (walking) {
            spinGaitOnce();
        }

        ros::spinOnce();

    }

    ROS_INFO("Exiting gaitController");

    return 0;
}
