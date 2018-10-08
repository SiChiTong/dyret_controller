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

bool enableCapture;

std::vector<std::vector<double>> imuData;
std::vector<std::vector<double>> currentData;

ros::Time startTime;
double accPos;
double accTime;
double linAcc_z;
std::vector<double> startPosition;
std::vector<double> currentPosition;
std::vector<double> lastSavedPosition;

bool getGaitEvaluationService(dyret_controller::GetGaitEvaluation::Request  &req,
                              dyret_controller::GetGaitEvaluation::Response &res){

  std::vector<float> results;
  std::vector<std::string> descriptors = {"inferredSpeed",
                                          "angVel",
                                          "linAcc",
                                          "combAngStab",
                                          "power",
                                          "sensorSpeed",
                                          "combImuStab",
                                          "linAcc_z"};

  switch (req.givenCommand){
    case (dyret_controller::GetGaitEvaluationRequest::t_getDescriptors):
    {
        results.resize(0);
    }
    case (dyret_controller::GetGaitEvaluationRequest::t_start):
    {
        ROS_INFO("req.t_start received\n");
        enableCapture = true;
        startTime = ros::Time::now();

        for (int i = 0; i < startPosition.size(); i++) startPosition[i] = currentPosition[i];

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
      accPos = 0.0;

      for (int i = 0; i < imuData.size(); i++) imuData[i].clear();
      for (int i = 0; i < currentData.size(); i++) currentData[i].clear();

      break;
    case (dyret_controller::GetGaitEvaluationRequest::t_getResults):
      ROS_INFO("req.t_getResults received\n");


      results.resize(8); // See descriptors above for contents

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
      ampHours = averagePowerDraw * ((((float) accTime) / 60.0f) / 60.0f);
      powerFitness = (fabs(accPos) / 1000.0f) / ampHours; // m / Ah

      // Calculate speed fitness value:
      float calculatedInferredSpeed = (fabs(accPos) / 1000.0f) / (((float) accTime) / 60.0f); // speed in m/min

      double sensorPoseDist = sqrt(pow(startPosition[0] - lastSavedPosition[0],2) + pow(startPosition[1] - lastSavedPosition[1],2) + pow(startPosition[2] - lastSavedPosition[2],2));
      float sensorPoseSpeed = sensorPoseDist / (((float) accTime) / 60.0f);

      ROS_INFO("Origin: %.2f, %.2f, %.2f\n", startPosition[0], startPosition[1], startPosition[2]);
      ROS_INFO("End:    %.2f, %.2f, %.2f\n", lastSavedPosition[0], lastSavedPosition[1], startPosition[2]);

      // Calculate mocap fitness value
      ROS_INFO("Distance: %.2f (S: %.2f)\n", sensorPoseDist, sensorPoseSpeed);

      // Assign fitness values:
      results[0] = calculatedInferredSpeed;            // Inferred speed in m/min
      results[1] = - (SDs[0] + SDs[1] + SDs[2]);       // angVel
      results[2] = - (SDs[3] + SDs[4] + SDs[5]);       // linAcc
      results[3] = - (SDs[6] + SDs[7] + SDs[8]);       // Combined angle stability (roll + pitch)
      results[4] = powerFitness;                       // Power efficiency
      results[5] = sensorPoseSpeed;                    // Sensor speed in m/min
      results[6] = results[3] + (results[2] / 50.0f);  // Combined IMU stability
      results[7] = (float) linAcc_z; // For topple detection

      break;
  };

  res.results = results;
  res.descriptors = descriptors;

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

void gaitInferredPos_Callback(const dyret_controller::DistAngMeasurement::ConstPtr& msg)
{
  if (msg->msgType == msg->t_measurementInferred){
      accPos = msg->distance;
  }
}

int main(int argc, char **argv) {
  lastSavedPosition.resize(3); // 3D mocap
  currentPosition.resize(3);
  startPosition.resize(3); // 3D mocap
  imuData.resize(9);      // 9 axes
  currentData.resize(12); // 12 servos
  accPos = 0.0;

  enableCapture = false;
  linAcc_z = 9.81;

  ros::init(argc, argv, "gaitEvaluator");
  ros::NodeHandle n;

  ros::Subscriber imuData_sub = n.subscribe("/dyret/sensor/imu", 100, imuDataCallback);
  ros::Subscriber gaitInferredPos_sub = n.subscribe("/dyret/dyret_controller/gaitInferredPos", 1000, gaitInferredPos_Callback);
  ros::ServiceServer gaitEvalService = n.advertiseService("get_gait_evaluation", getGaitEvaluationService);
  ros::Subscriber servoStates_sub = n.subscribe("/dyret/state", 1, servoStatesCallback);
  ros::Subscriber sensorPose_sub = n.subscribe("/dyret/sensor/pose", 5, sensorPoseCallback);

  sleep(1);

  while(ros::ok()) ros::spin();

}
