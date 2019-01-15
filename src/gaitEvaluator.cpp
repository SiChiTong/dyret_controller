#include <sys/stat.h>

#include <ctime>
#include <chrono>
#include <signal.h>
#include <math.h>
#include <numeric>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "dyret_common/State.h"

#include "dyret_common/wait_for_ros.h"

#include "dyret_controller/DistAngMeasurement.h"
#include "dyret_controller/GetGaitControllerStatus.h"
#include "dyret_controller/GetGaitEvaluation.h"
#include "dyret_controller/GetInferredPosition.h"

bool enableCapture;
bool enableLogging = false;
bool firstLine = true;

std::vector<std::vector<double>> imuData;
std::vector<std::vector<double>> currentData;

ros::Time startTime;
double accTime;
double linAcc_z;
std::vector<double> startPosition;
std::vector<double> currentPosition;
std::vector<double> lastSavedPosition;

ros::ServiceClient inferredPositionClient;

FILE * fitnessLogFile;

// This function makes sure the given key exists in the map before inserting it
void setMapValue(std::map<std::string, double> &givenMap, std::string givenKey, double givenValue){
  if (givenMap.find(givenKey) != givenMap.end()){
    givenMap[givenKey] = givenValue;
  } else {
    ROS_FATAL("GivenKey \"%s\" not found in map!", givenKey.c_str());
    ros::shutdown();
    exit(-1);
  }
}

float getInferredPosition(){
    dyret_controller::GetInferredPosition srv;

    if (!inferredPositionClient.call(srv)){
        printf("Error while calling GetInferredPosition service\n");
        ROS_ERROR("Error while calling GetInferredPosition service");

        return 0.0;
    }

    return srv.response.currentInferredPosition.distance;
}

std::map<std::string, double> getResults(std::map<std::string, double> resultMap, bool useCurrentTime = false){

    double timePassed = accTime;

    if (useCurrentTime){
        timePassed = (ros::Time::now() - startTime).sec;
    }

    // Calculate IMU fitness values:
    std::vector<float> sums(imuData.size());
    std::vector<float> means(imuData.size());
    std::vector<float> sq_sums(imuData.size());
    std::vector<float> SDs(imuData.size());
    //std::vector<float> SDs_z(imuData.size());

    for (int i = 0; i < imuData.size(); i++){
        sums[i] = std::accumulate(imuData[i].begin(), imuData[i].end(), 0.0);
        means[i] = sums[i] / imuData[i].size();
        sq_sums[i] = std::inner_product(imuData[i].begin(), imuData[i].end(), imuData[i].begin(), 0.0);
        SDs[i] = std::sqrt(sq_sums[i] / imuData[i].size() - means[i] * means[i]);
        //SDs_z[i] = std::sqrt(sq_sums[i] / imuData[i].size());
    }

    // Calculate servo fitness value:
    std::vector<float> currentSums(currentData.size());
    std::vector<float> currentMeans(currentData.size());
    float averagePowerDraw;
    float ampHours;
    float powerFitness;

    for (int i = 0; i < currentData.size(); i++){
        currentSums[i] = std::accumulate(currentData[i].begin(), currentData[i].end(), 0.0f);
        currentMeans[i] = currentSums[i] / currentData[i].size();
    }
    averagePowerDraw = std::accumulate(currentMeans.begin(), currentMeans.end(), 0.0f);
    if (averagePowerDraw != 0) {
        ampHours = averagePowerDraw * ((((float) timePassed) / 60.0f) / 60.0f);
        powerFitness = (fabs(getInferredPosition()) / 1000.0f) / ampHours; // m / Ah
    } else{
        powerFitness = 0.0;
    }

    // Calculate speed fitness value:
    float calculatedInferredSpeed = (float) (fabs(getInferredPosition()) / 1000.0f) / (((float) timePassed) / 60.0f); // speed in m/min

    float sensorPoseDist = (float) sqrt(pow(startPosition[0] - lastSavedPosition[0],2) + pow(startPosition[1] - lastSavedPosition[1],2) + pow(startPosition[2] - lastSavedPosition[2],2));
    float sensorPoseDistForward = (float) -(startPosition[1] - lastSavedPosition[1]);

    float sensorPoseSpeed = sensorPoseDist / (((float) timePassed) / 60.0f);
    float sensorPoseSpeedForward = sensorPoseDistForward / (((float) timePassed) / 60.0f);

    ROS_INFO("Origin: %.2f, %.2f, %.2f\n", startPosition[0], startPosition[1], startPosition[2]);
    ROS_INFO("End:    %.2f, %.2f, %.2f\n", lastSavedPosition[0], lastSavedPosition[1], startPosition[2]);

    // Calculate mocap fitness value
    ROS_INFO("Distance: %.2f (S: %.2f)\n", sensorPoseDist, sensorPoseSpeed);

    // Assign fitness values:
    setMapValue(resultMap, "inferredSpeed", calculatedInferredSpeed);
    setMapValue(resultMap, "angVel", -(SDs[0] + SDs[1] + SDs[2]));
    setMapValue(resultMap, "linAcc", -(SDs[3] + SDs[4] + SDs[5]));
    setMapValue(resultMap, "combAngStab", - (SDs[6] + SDs[7]));
    setMapValue(resultMap, "power", powerFitness);
    setMapValue(resultMap, "sensorSpeed", sensorPoseSpeed);
    setMapValue(resultMap, "combImuStab", resultMap["combAngStab"] + (resultMap["linAcc"] / 50.0f));
    setMapValue(resultMap, "linAcc_z", (float) linAcc_z);
    setMapValue(resultMap, "sensorSpeedForward", sensorPoseSpeedForward);

    return resultMap;
}

std::string getDateString() {
    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    std::stringstream ss;

    ss << now->tm_year + 1900
       << std::setw(2) << std::setfill('0') << now->tm_mon + 1
       << std::setw(2) << std::setfill('0') << now->tm_mday
       << std::setw(2) << std::setfill('0') << now->tm_hour
       << std::setw(2) << std::setfill('0') << now->tm_min
       << std::setw(2) << std::setfill('0') << now->tm_sec;

    return ss.str();

}

void printMap(std::map<std::string, double> givenMap, std::string givenLeadingString, FILE * givenDestination){

    int i = 0;
    for(auto elem : givenMap){

        if (isinf(elem.second)){
            fprintf(givenDestination, (givenLeadingString + "\"%s\": \"INF\"").c_str(), elem.first.c_str());
        } else if (isnan(elem.second)) {
            fprintf(givenDestination, (givenLeadingString + "\"%s\": \"NAN\"").c_str(), elem.first.c_str());
        } else {
            fprintf(givenDestination, (givenLeadingString + "\"%s\": %f").c_str(), elem.first.c_str(), elem.second);
        }

        if (i != givenMap.size()-1) fprintf(givenDestination, ",\n"); else fprintf(givenDestination, "\n");
        i++;
    }
}

bool getGaitEvaluationService(dyret_controller::GetGaitEvaluation::Request  &req,
                              dyret_controller::GetGaitEvaluation::Response &res){

  std::map<std::string, double> results = {{"inferredSpeed", 0.0},
                                           {"angVel", 0.0},
                                           {"linAcc", 0.0},
                                           {"combAngStab", 0.0},
                                           {"power", 0.0},
                                           {"sensorSpeed", 0.0},
                                           {"combImuStab", 0.0},
                                           {"linAcc_z", 0.0},
                                           {"sensorSpeedForward",  0.0}};

  switch (req.givenCommand){
    case (dyret_controller::GetGaitEvaluationRequest::t_getDescriptors):
    {

    }
    case (dyret_controller::GetGaitEvaluationRequest::t_start):
    {
        ROS_INFO("req.t_start received\n");
        enableCapture = true;
        startTime = ros::Time::now();

        for (int i = 0; i < startPosition.size(); i++) startPosition[i] = currentPosition[i];

        if (enableLogging){
            fitnessLogFile = fopen(std::string("/home/tonnesfn/catkin_ws/customLogs/fitnessEvals/" + getDateString() + "_fitness.csv").c_str(), "w");
            fprintf(fitnessLogFile, "{\n  \"measurements\": [\n");
            firstLine = true;
        }


        break;
    }
    case (dyret_controller::GetGaitEvaluationRequest::t_pause):
        ROS_INFO("req.t_pause received\n");
        enableCapture = false;
        accTime += (ros::Time::now() - startTime).sec;

        break;
    case (dyret_controller::GetGaitEvaluationRequest::t_resetStatistics):
      ROS_INFO("req.t_resetStatistics received\n");
      enableCapture = false;

      accTime = 0.0;

      for (int i = 0; i < imuData.size(); i++) imuData[i].clear();
      for (int i = 0; i < currentData.size(); i++) currentData[i].clear();

      break;
    case (dyret_controller::GetGaitEvaluationRequest::t_getResults):
      ROS_INFO("req.t_getResults received\n");

      results = getResults(results);

      if (enableLogging){
          fprintf(fitnessLogFile, "    }\n  ]\n}");
          fclose(fitnessLogFile);
      }

      break;
    case (dyret_controller::GetGaitEvaluationRequest::t_disableLogging):
        ROS_INFO("Disabled logging");
        enableLogging = false;
        break;
    case (dyret_controller::GetGaitEvaluationRequest::t_enableLogging):
        ROS_INFO("Enabledlogging");
        enableLogging = true;
        break;
  };

  for(auto elem : results){
    res.descriptors.push_back(elem.first);
    res.results.push_back(elem.second);
  }

  return true;

}

void sensorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  currentPosition[0] = msg->pose.position.x;
  currentPosition[1] = msg->pose.position.y;
  currentPosition[2] = msg->pose.position.z;

  if (enableCapture){
      lastSavedPosition[0] = msg->pose.position.x;
      lastSavedPosition[1] = msg->pose.position.y;
      lastSavedPosition[2] = msg->pose.position.z;
  }
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  if (enableCapture) {

    // Angular velocity
    imuData[0].push_back(msg->angular_velocity.x);
    imuData[1].push_back(msg->angular_velocity.y);
    imuData[2].push_back(msg->angular_velocity.z);

    // Linear acceleration
    imuData[3].push_back(msg->linear_acceleration.x);
    imuData[4].push_back(msg->linear_acceleration.y);
    imuData[5].push_back(msg->linear_acceleration.z);

    tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    imuData[6].push_back(roll);
    imuData[7].push_back(pitch);
    imuData[8].push_back(fabs(yaw));
  }

  linAcc_z = msg->linear_acceleration.z;
}

void servoStatesCallback(const dyret_common::State::ConstPtr& msg){
  if (enableCapture){
      for(int i = 0; i < 12; i++){
          currentData[i].push_back(msg->revolute[i].current);
      }
  }
}

void logCallback(const ros::TimerEvent&){
    if (enableLogging && enableCapture){
        std::map<std::string, double> results = {{"inferredSpeed", 0.0},
                                                 {"angVel", 0.0},
                                                 {"linAcc", 0.0},
                                                 {"combAngStab", 0.0},
                                                 {"power", 0.0},
                                                 {"sensorSpeed", 0.0},
                                                 {"combImuStab", 0.0},
                                                 {"linAcc_z", 0.0},
                                                 {"sensorSpeedForward",  0.0}};
        results = getResults(results, true);

        if (firstLine){
            firstLine = false;
            fprintf(fitnessLogFile, "    {\n");
        } else {
            fprintf(fitnessLogFile, "    },\n    {\n");
        }

        fprintf(fitnessLogFile, "      \"msec\": %lu,\n", ros::Time::now().toNSec() / 1000);
        printMap(results, "      ", fitnessLogFile);
    }
}

int main(int argc, char **argv) {
  lastSavedPosition.resize(3); // 3D mocap
  currentPosition.resize(3);
  startPosition.resize(3);     // 3D mocap
  imuData.resize(9);           // 9 axes
  currentData.resize(12);      // 12 servos

  enableCapture = false;
  linAcc_z = 9.81;

  ros::init(argc, argv, "gaitEvaluator");
  ros::NodeHandle n;

  ros::Subscriber imuData_sub = n.subscribe("/dyret/sensor/imu", 100, imuDataCallback);
  ros::ServiceServer gaitEvalService = n.advertiseService("get_gait_evaluation", getGaitEvaluationService);
  ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);
  ros::Subscriber sensorPose_sub = n.subscribe("/dyret/sensor/pose", 5, sensorPoseCallback);
  inferredPositionClient = n.serviceClient<dyret_controller::GetInferredPosition>("/dyret/dyret_controller/getInferredPosition");

  ros::Timer timer = n.createTimer(ros::Duration(0.1), logCallback);

  while(ros::ok()) ros::spin();

}
