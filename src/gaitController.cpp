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
const float rearLegOffset = 0.0f; // was -30

bool walking = false;

std::string gaitType;
std::map<std::string, float> gaitConfiguration;

// This decides if the leg adjustment done in hardware is done in sim as well
bool initAdjustInSim = true;

double currentInferredPosition = 0.0;
bool movingForward = true;
double globalGaitFrequency;
double globalLiftDuration;

const double groundHeightOffset = -487.5f; // Ground height at femurlength  0, tibialength   0
const double groundHeightMax = -637.5f;    // Ground height at femurlength 50, tibialength 100
const double groundCorrectionFactor = -(groundHeightMax - groundHeightOffset) / 150.0f;

double gaitTimingOffset = 0.0; // Offset to allow for lining up different frequency gaits
double globalTime;
double globalTimeAtChange = 0.0;

const float bSplineGaitWagOffset = 0.91;

const float spreadAmount = 80.0; // was 50

bool activatedRecording;

std::vector<vec3P> startGaitPose;

std::vector<double> pidParameters;

std::vector<double> servoAnglesInRad(12);

std::array<double, 4> groundHeights = {groundHeightOffset, groundHeightOffset, groundHeightOffset, groundHeightOffset};

std::array<double, 8> prismaticPositions;
std::array<double, 8> prismaticErrors;
std::array<double, 8> prismaticCommands;

// Public stuff
ros::ServiceClient get_gait_evaluation_client;
ros::ServiceClient servoConfigClient;
BSplineGait bSplineGait;
BSplineGait bSplineGait_rear;
WagGenerator wagGenerator;
ros::Publisher poseCommand_pub;
ros::Time startTime;
ros::ServiceClient positionCommand_pub;
ros::ServiceClient loggerCommandService_client;

IncPoseAdjuster gaitInitAdjuster(
        &servoAnglesInRad,
        &prismaticCommands,
        &positionCommand_pub);



bool getGaitControllerStatusService(dyret_controller::GetGaitControllerStatus::Request &req,
                                    dyret_controller::GetGaitControllerStatus::Response &res) {
    res.gaitControllerStatus.actionType = currentAction;

    return true;
}

void setLegLengths(std::vector<float> femurLengths, std::vector<float> tibiaLengths) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    if (femurLengths.size() == 1 && tibiaLengths.size() == 1){
        msg.prismatic.resize(2);
        msg.prismatic[0] = femurLengths[0];
        msg.prismatic[1] = tibiaLengths[0];
    }

    if (femurLengths.size() == 2 && tibiaLengths.size() == 2){
        msg.prismatic.resize(8);
        msg.prismatic[0] = femurLengths[0];
        msg.prismatic[1] = tibiaLengths[0];
        msg.prismatic[2] = femurLengths[0];
        msg.prismatic[3] = tibiaLengths[0];
        msg.prismatic[4] = femurLengths[1];
        msg.prismatic[5] = tibiaLengths[1];
        msg.prismatic[6] = femurLengths[1];
        msg.prismatic[7] = tibiaLengths[1];
    }

    poseCommand_pub.publish(msg);
}

void setLegLengths(std::array<double, 8> givenPrismaticCommands) {
    dyret_common::Pose msg;

    msg.header.stamp = ros::Time().now();

    msg.prismatic.resize(givenPrismaticCommands.size());

    for (int i = 0; i < givenPrismaticCommands.size(); i++){
        msg.prismatic[i] = (float) givenPrismaticCommands[i];
    }

    poseCommand_pub.publish(msg);
}

bool allLegsAreCloserThan(float givenMargin){

    // Sleep and spin to receive a new message with the error
    ros::Duration(0.15).sleep();
    ros::spinOnce();

    for (int i = 0; i < prismaticErrors.size(); i++){
        if (fabs(prismaticErrors[i]) > givenMargin) return false;
    }

    return true;
}

float getGroundHeight(float givenFemurLength, float givenTibiaLength){

    return (float) (groundHeightOffset - ((givenFemurLength + givenTibiaLength) * groundCorrectionFactor));

}

void servoStatesCallback(const dyret_common::State::ConstPtr &msg) {
    for (int i = 0; i < 12; i++) {
        servoAnglesInRad[i] = msg->revolute[i].position;
    }

    for (int i = 0; i < prismaticPositions.size(); i++) {
        prismaticPositions[i] = msg->prismatic[i].position;
    }

    for (int i = 0; i < 4; i++){
        groundHeights[i] = getGroundHeight(prismaticPositions[i*2], prismaticPositions[(i*2)+1]);
    }

    for (int i = 0; i < prismaticErrors.size(); i++){
        prismaticErrors[i] = msg->prismatic[i].error;
    }

}

std::vector<vec3P> getRestPose() {
    const float frontOffset = 0.0;

    const std::vector<vec3P> restPose = {{-spreadAmount, frontOffset, (float) groundHeights[0]},
                                         {spreadAmount,  frontOffset, (float) groundHeights[1]},
                                         {spreadAmount,  frontOffset, (float) groundHeights[2]},
                                         {-spreadAmount, frontOffset, (float) groundHeights[3]}};

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

        // Update groundheights:
        bSplineGait.setGroundHeights(groundHeights);

        lastUpdate = ros::Time::now();

        // Get leg positions:
        double currentRelativeTime =
                (ros::Time::now() - startTime).toNSec() / 1000000.0; // Given in milliseconds
        std::vector<vec3P> globalLegPositions(4);

        globalTime = (currentRelativeTime * globalGaitFrequency) - gaitTimingOffset;

        if (globalTimeAtChange != 0.0){ // There has been a gait reconfiguration
            //fprintf(stderr, "Global time: %.2f, globalTimeAtChange: %.2f\n", globalTime, globalTimeAtChange);
            //fprintf(stderr, "Old gaitTimingOffset: %.2f\n", gaitTimingOffset);

            gaitTimingOffset = globalTime + gaitTimingOffset - globalTimeAtChange;

            //fprintf(stderr, "New gaitTimingOffset: %.2f\n", gaitTimingOffset);

            globalTime = (currentRelativeTime * globalGaitFrequency) - gaitTimingOffset;

            globalTimeAtChange = 0.0;
        }

        globalLegPositions = bSplineGait.getPosition(globalTime, movingForward);

        if (gaitType == "lowLevelAdvancedSplineGait") {
            std::vector<vec3P> globalLegPositionsRear = bSplineGait_rear.getPosition(globalTime, movingForward);
            globalLegPositions[2] = globalLegPositionsRear[2];
            globalLegPositions[3] = globalLegPositionsRear[3];
        }

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

        vec3P wag = wagGenerator.getGaitWagPoint(globalTime, movingForward);

        std::vector<vec3P> currentPositions = currentLegPositions(servoAnglesInRad, prismaticPositions);

        bool validSolution = true;

        // Get IK solutions for each leg:
        for (int i = 0; i < 4; i++) { // For each leg

            vec3P legPosition = add(globalLegPositions[i], wag);

            // Only do leg length correction for lowLevelAdvancedSplineGait, which is the only gait with support for multiple leg lengths
            if (gaitType == "lowLevelAdvancedSplineGait") {
                legPosition = doLegLengthCorrection(legPosition, i, prismaticCommands, groundHeights);
            }

            std::vector<double> inverseReturn = inverseKinematics::calculateInverseKinematics(legPosition.x(),
                                                                                              legPosition.y(),
                                                                                              legPosition.z(),
                                                                                              i,
                                                                                              prismaticPositions[i*2],
                                                                                              prismaticPositions[(i*2)+1]);

            if (inverseReturn.size() == 0){
                ROS_ERROR("Not sending invalid kinematics return");
                validSolution = false;
                break;
            }

            for (int j = 0; j < 3; j++) {
                anglesInRad.push_back(inverseReturn[j]);
            }

        }

        if (validSolution) {

            if (servoIds.size() != 0) {
                // Set revolute joints positions:
                msg.revolute = anglesInRad;

                // Set prismamtic joints positions:
                msg.prismatic.resize(prismaticCommands.size());
                for (int i = 0; i < prismaticCommands.size(); i++) {
                    msg.prismatic[i] = prismaticCommands[i];
                }

                poseCommand_pub.publish(msg);
            } else {
                ROS_WARN("Did not send invalid dyn commands!\n");
            }
        }
    }
}

bool adjustPose(std::vector<vec3P> givenPose, std::array<double, 4> givenGroundHeight){
    ros::Rate poseAdjusterRate(50);

    // Wait for leg length to be close enough so we dont get any IK errors

    do {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    } while(!allLegsAreCloserThan(10.0f));

    IncPoseAdjuster poseAdjuster(&servoAnglesInRad,
                                 &prismaticPositions,
                                 &positionCommand_pub);

    poseAdjuster.setPose(givenPose, givenGroundHeight);


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
    return adjustPose(getRestPose(), groundHeights);
}

bool adjustGaitPose(std::array<double, 4> givenGroundHeight){
    return adjustPose(startGaitPose, givenGroundHeight);
}

bool adjustGaitPose(){
    return adjustPose(startGaitPose, groundHeights);
}

float getMapValue(std::map<std::string, float> givenMap, std::string givenKey){
    if (givenMap.find(givenKey) == givenMap.end()) {
        ROS_ERROR("Could not find key %s", givenKey.c_str());
    }

    return givenMap.at(givenKey);
}

bool gaitConfigurationCallback(dyret_controller::ConfigureGait::Request  &req,
                               dyret_controller::ConfigureGait::Response &res) {

    ROS_INFO("Got gait configuration message for gait type %s", req.gaitConfiguration.gaitType.c_str());

    if (req.gaitConfiguration.liveUpdate) {
        globalTimeAtChange = globalTime;
    } else {
        gaitTimingOffset = 0.0;
    }

    //if (req.gaitConfiguration.liveUpdate) fprintf(stderr, "Live update\n"); else fprintf(stderr, "Offline update\n");

    if (req.gaitConfiguration.femurLengths.size() == 1 && req.gaitConfiguration.tibiaLengths.size() == 1) {
        prismaticCommands = {req.gaitConfiguration.femurLengths[0], req.gaitConfiguration.tibiaLengths[0],
                             req.gaitConfiguration.femurLengths[0], req.gaitConfiguration.tibiaLengths[0],
                             req.gaitConfiguration.femurLengths[0], req.gaitConfiguration.tibiaLengths[0],
                             req.gaitConfiguration.femurLengths[0], req.gaitConfiguration.tibiaLengths[0]};
    } else if (req.gaitConfiguration.femurLengths.size() == 4 && req.gaitConfiguration.tibiaLengths.size() == 4) {
        prismaticCommands = {req.gaitConfiguration.femurLengths[0], req.gaitConfiguration.tibiaLengths[0],
                             req.gaitConfiguration.femurLengths[1], req.gaitConfiguration.tibiaLengths[1],
                             req.gaitConfiguration.femurLengths[2], req.gaitConfiguration.tibiaLengths[2],
                             req.gaitConfiguration.femurLengths[3], req.gaitConfiguration.tibiaLengths[3]};
    } else {
        ROS_ERROR("Unsupported leg length vector size in gait configuration message");
        prismaticCommands = {0, 0, 0, 0, 0, 0, 0, 0};
    }

    if (req.gaitConfiguration.liveUpdate == false) {

        // Set leg lengths and wait until they reach the correct length
        if (req.gaitConfiguration.prepareForGait) {
            setLegLengths(prismaticCommands);
            if (ros::Time::isSimTime()) ros::Duration(2).sleep();
        }

        // Reset vars between runs
        activatedRecording = false;
        currentInferredPosition = 0.0;

        gaitType = req.gaitConfiguration.gaitType;
    }

    std::array<double,4> tmpGroundHeights;

    for (int i = 0; i < 4; i++) {
        tmpGroundHeights[i] = getGroundHeight(prismaticCommands[i * 2], prismaticCommands[(i * 2) + 1]);
    }

    gaitConfiguration.clear();
    assert(req.gaitConfiguration.gaitParameterName.size() == req.gaitConfiguration.gaitParameterValue.size());
    for (size_t i = 0; i < req.gaitConfiguration.gaitParameterName.size(); ++i)
        gaitConfiguration[req.gaitConfiguration.gaitParameterName[i]] = req.gaitConfiguration.gaitParameterValue[i];

    if (req.gaitConfiguration.liveUpdate == false) gaitInitAdjuster.reset();

    if (gaitType == "highLevelSplineGait") {
        globalLiftDuration = getMapValue(gaitConfiguration, "liftDuration");

        bSplineGait.initHighLevelGait(getMapValue(gaitConfiguration, "stepHeight"),
                                      getMapValue(gaitConfiguration, "stepLength"),
                                      getMapValue(gaitConfiguration, "smoothing"),
                                      tmpGroundHeights,
                                      spreadAmount,
                                      frontOffset,
                                      rearLegOffset,
                                      globalLiftDuration);

        wagGenerator.enableWag(bSplineGaitWagOffset + getMapValue(gaitConfiguration, "wagPhase"),
                               getMapValue(gaitConfiguration, "wagAmplitude_x"),
                               getMapValue(gaitConfiguration, "wagAmplitude_y"));

        if (!req.gaitConfiguration.logFilePath.empty()) {
          bSplineGait.writeGaitToFile(req.gaitConfiguration.logFilePath);
        }

    } else if (gaitType == "lowLevelSplineGait") {

        globalGaitFrequency = getMapValue(gaitConfiguration, "frequency");
        globalLiftDuration = getMapValue(gaitConfiguration, "liftDuration");

        bSplineGait.initLowLevelGait(gaitConfiguration, tmpGroundHeights);

        wagGenerator.enableWag(bSplineGaitWagOffset + getMapValue(gaitConfiguration, "wagPhase"),
                               getMapValue(gaitConfiguration, "wagAmplitude_x"),
                               getMapValue(gaitConfiguration, "wagAmplitude_y"));



        if (!req.gaitConfiguration.logFilePath.empty()) {
            bSplineGait.writeGaitToFile(req.gaitConfiguration.logFilePath);
            wagGenerator.writeWagLog(req.gaitConfiguration.logFilePath);
        }

    } else if (gaitType == "lowLevelAdvancedSplineGait"){

        globalGaitFrequency = getMapValue(gaitConfiguration, "frequency");
        globalLiftDuration = getMapValue(gaitConfiguration, "liftDuration");

        // Convert to separate phenotype maps:
        std::map<std::string, float> frontGaitConfiguration = {{"wagPhase",         getMapValue(gaitConfiguration, "wagPhase")},
                                                               {"wagAmplitude_x",   getMapValue(gaitConfiguration, "wagAmplitude_x")},
                                                               {"wagAmplitude_y",   getMapValue(gaitConfiguration, "wagAmplitude_y")},
                                                               {"liftDuration",     getMapValue(gaitConfiguration, "liftDuration")},
                                                               {"difficultyFactor", getMapValue(gaitConfiguration, "difficultyFactor")},
                                                               {"p0_x",             0.0f},
                                                               {"p0_y",             getMapValue(gaitConfiguration, "p0_y")},
                                                               {"p1_x",             0.0f},
                                                               {"p1_y",             getMapValue(gaitConfiguration, "p1_y")},
                                                               {"p2_x",             getMapValue(gaitConfiguration, "p2_x_front")},
                                                               {"p2_y",             getMapValue(gaitConfiguration, "p2_y_front")},
                                                               {"p2_z",             getMapValue(gaitConfiguration, "p2_z_front")},
                                                               {"p3_x",             getMapValue(gaitConfiguration, "p3_x_front")},
                                                               {"p3_y",             getMapValue(gaitConfiguration, "p3_y_front")},
                                                               {"p3_z",             getMapValue(gaitConfiguration, "p3_z_front")},
                                                               {"p4_x",             getMapValue(gaitConfiguration, "p4_x_front")},
                                                               {"p4_y",             getMapValue(gaitConfiguration, "p4_y_front")},
                                                               {"p4_z",             getMapValue(gaitConfiguration, "p4_z_front")}};

        std::map<std::string, float> rearGaitConfiguration = {{"wagPhase",         getMapValue(gaitConfiguration, "wagPhase")},
                                                              {"wagAmplitude_x",   getMapValue(gaitConfiguration, "wagAmplitude_x")},
                                                              {"wagAmplitude_y",   getMapValue(gaitConfiguration, "wagAmplitude_y")},
                                                              {"liftDuration",     getMapValue(gaitConfiguration, "liftDuration")},
                                                              {"difficultyFactor", getMapValue(gaitConfiguration, "difficultyFactor")},
                                                              {"p0_x",             0.0f},
                                                              {"p0_y",             getMapValue(gaitConfiguration, "p0_y")},
                                                              {"p1_x",             0.0f},
                                                              {"p1_y",             getMapValue(gaitConfiguration, "p1_y")},
                                                              {"p2_x",             getMapValue(gaitConfiguration, "p2_x_rear")},
                                                              {"p2_y",             getMapValue(gaitConfiguration, "p2_y_rear")},
                                                              {"p2_z",             getMapValue(gaitConfiguration, "p2_z_rear")},
                                                              {"p3_x",             getMapValue(gaitConfiguration, "p3_x_rear")},
                                                              {"p3_y",             getMapValue(gaitConfiguration, "p3_y_rear")},
                                                              {"p3_z",             getMapValue(gaitConfiguration, "p3_z_rear")},
                                                              {"p4_x",             getMapValue(gaitConfiguration, "p4_x_rear")},
                                                              {"p4_y",             getMapValue(gaitConfiguration, "p4_y_rear")},
                                                              {"p4_z",             getMapValue(gaitConfiguration, "p4_z_rear")}};

        bSplineGait.initLowLevelGait(frontGaitConfiguration, tmpGroundHeights);
        bSplineGait_rear.initLowLevelGait(rearGaitConfiguration, tmpGroundHeights);

        wagGenerator.enableWag(bSplineGaitWagOffset + getMapValue(gaitConfiguration, "wagPhase"),
                               getMapValue(gaitConfiguration, "wagAmplitude_x"),
                               getMapValue(gaitConfiguration, "wagAmplitude_y"));

        /*if (!req.gaitConfiguration.logFilePath.empty()) {
            bSplineGait.writeGaitToFile(req.gaitConfiguration.logFilePath);
        }*/

    } else {
        ROS_FATAL("Unknown gait specified: %s!", gaitType.c_str());
        exit(-1);
    }

    globalGaitFrequency = getMapValue(gaitConfiguration, "frequency");

    if (req.gaitConfiguration.liveUpdate == false) {
        // Set initial pose for adjustment
        movingForward = req.gaitConfiguration.directionForward;

        // Calculate startpose for all four legs or two at a time
        startGaitPose = lockToZ(
                add(bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward)),
                tmpGroundHeights);

        if (gaitType == "lowLevelAdvancedSplineGait") {
            std::vector <vec3P> startGaitPoseRear = lockToZ(add(bSplineGait_rear.getPosition(0.0, movingForward),
                                                                wagGenerator.getGaitWagPoint(0.0, movingForward)),
                                                            tmpGroundHeights);;
            startGaitPose[2] = startGaitPoseRear[2];
            startGaitPose[3] = startGaitPoseRear[3];
        }

        if (ros::Time::isSystemTime()) {
            setServoSpeeds(poseAdjustSpeed, servoConfigClient);
            ROS_INFO("Set servo speed to %.2f", poseAdjustSpeed);
        }

        // Adjust to the start of the gait only in simulation. NOTE: THIS ASSUMES FORWARD MOVEMENT
        if (ros::Time::isSimTime() && !initAdjustInSim) {
            // Calculate the initial pose of the gait
            std::vector <vec3P> initialGaitPose = lockToZ(
                    add(bSplineGait.getPosition(0.0, movingForward), wagGenerator.getGaitWagPoint(0.0, movingForward)),
                    tmpGroundHeights);

            if (gaitType == "lowLevelAdvancedSplineGait") {
                std::vector <vec3P> initialGaitPoseRear = lockToZ(add(bSplineGait_rear.getPosition(0.0, movingForward),
                                                                      wagGenerator.getGaitWagPoint(0.0, movingForward)),
                                                                  tmpGroundHeights);
                initialGaitPose[2] = initialGaitPoseRear[2];
                initialGaitPose[3] = initialGaitPoseRear[3];
            }

            moveAllLegsToGlobalPosition(initialGaitPose, &positionCommand_pub);
            gaitInitAdjuster.skip();
            sleep(1);
            startTime = ros::Time::now();
        }

        // Adjust gait pose
        if (req.gaitConfiguration.prepareForGait) {
            adjustGaitPose(tmpGroundHeights);
        }

        ros::Time legLengthAdjustmentStart = ros::Time::now();
        if (req.gaitConfiguration.prepareForGait) {
            moveAllLegsToGlobalPosition(startGaitPose,
                                        &positionCommand_pub); // Continuously send pose to adjust to leg length change

            setLegLengths(prismaticCommands);

            ros::spinOnce();

            if (!ros::Time::isSimTime()) {

                while (!allLegsAreCloserThan(1.0f)) {

                    ros::spinOnce();
                    usleep(10000);
                    //setLegLengths(prismaticCommands);

                    moveAllLegsToGlobalPosition(startGaitPose,
                                                &positionCommand_pub); // Continuously send pose to adjust to leg length change

                    int secPassed = ros::Time::now().sec - legLengthAdjustmentStart.sec;

                    if (((ros::Time::isSystemTime()) && (secPassed > 90)) ||
                        (ros::Time::isSimTime() && (secPassed > 5))) {
                        ROS_ERROR("Timed out waiting for legs to be at length at %ds", secPassed);
                        return false;
                    }
                }
                ROS_INFO("Leg lengths achieved");
            }

            sleep(1); // Wait one second for leg length to finish
        }

        if (ros::Time::isSystemTime()) {
            setServoSpeeds(gaitSpeed, servoConfigClient);
            ROS_INFO("Set servo speed to %.2f", gaitSpeed);
        }
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
