#include <ctime>
#include <chrono>
#include <signal.h>
#include <math.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Pose2D.h"

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "dyret_common/ServoState.h"
#include "dyret_common/ServoStateArray.h"
#include "dyret_common/GetGaitEvaluation.h"
#include "dyret_common/GetGaitControllerStatus.h"
#include "dyret_common/DistAng.h"

#include "dyret_utils/wait_for_ros.h"
#include "dyret_utils/timeHandling.h"

bool enableCapture;

std::vector<std::vector<float>> imuData;
std::vector<std::vector<float>> currentData;

FILE * imuLog;
FILE * mocapLog;
bool loggingEnabled = true;

long long int startTime;
long long int accTime;
float accPos;
float linAcc_z;
float receivedCalculated;
std::vector<float> startPosition;
std::vector<float> currentPosition;
std::vector<float> lastSavedPosition;

volatile sig_atomic_t discardSolution = 0;

void discardFunction(int sig){
  ROS_WARN(" Discarding solution\n");
  discardSolution = 1;
}

bool getGaitEvaluationService(dyret_common::GetGaitEvaluation::Request  &req,
		dyret_common::GetGaitEvaluation::Response &res){

  std::vector<float> results;

  switch (req.givenCommand){
    case (req.t_start):
    {
        ROS_INFO("req.t_start received\n");
        enableCapture = true;
        startTime = getMs();

        time_t t = time(0);   // get time now
        struct tm * now = localtime( & t );

        if (loggingEnabled == true){
            char fileNameBufferImu[100];
            sprintf(fileNameBufferImu,"/home/tonnesfn/catkin_ws/customLogs/imuLogs/imu_%04u%02u%02u%02u%02u%02u.csv", now->tm_year+1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
            imuLog = fopen(fileNameBufferImu, "w");
            fprintf(imuLog, "time (ms), msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,"
                            "msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,"
                            "roll, pitch, yaw\n");
            char fileNameBufferMocap[100];
            sprintf(fileNameBufferMocap,"/home/tonnesfn/catkin_ws/customLogs/mocapLogs/mocap_%04u%02u%02u%02u%02u%02u.csv", now->tm_year+1900, now->tm_mon+1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
            mocapLog = fopen(fileNameBufferMocap, "w");
            fprintf(mocapLog, "time (ms), "
                              "msg->pose.position.x, "
                              "msg->pose.position.y, "
                              "msg->pose.position.z, "
                              "msg->pose.orientation.w, "
                              "msg->pose.orientation.x, "
                              "msg->pose.orientation.y, "
                              "msg->pose.orientation.z\n");
        }

        for (int i = 0; i < startPosition.size(); i++) startPosition[i] = currentPosition[i];

        break;
    }
    case (req.t_pause):
        ROS_INFO("req.t_pause received\n");
        enableCapture = false;
        accTime += getMs() - startTime;

        if (loggingEnabled == true){
            if (imuLog != NULL){
                fclose(imuLog);
                fclose(mocapLog);
                imuLog = NULL;
                mocapLog = NULL;
            }
        }

        break;
    case (req.t_resetStatistics):
      enableCapture = false;

      //printf("req.t_resetStatistics received\n");
      if (loggingEnabled == true){
          if (imuLog != NULL){
              fclose(imuLog);
              fclose(mocapLog);
              imuLog = NULL;
              mocapLog = NULL;
          }
      }

      accTime = 0.0;
      accPos = 0.0;

      for (int i = 0; i < imuData.size(); i++) imuData[i].clear();
      for (int i = 0; i < currentData.size(); i++) currentData[i].clear();

      break;
    case (req.t_getResults):

      if (discardSolution == 1){
        ROS_WARN("Discarded!\n");
        discardSolution = 0;
      } else if (linAcc_z < 0){
        ROS_WARN("Upside down!\n");
      } else if (linAcc_z < 8){
        ROS_WARN("Sideways!\n");
      } else {

        results.resize(7); // 0: speed, 1: sum(SD['angVel'], 2: sum(SD['linAcc']), 3: sum(SD*('orientation'), 4: current, 5: mocapDistance, 3+2/50

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
            currentSums[i] = std::accumulate(currentData[i].begin(), currentData[i].end(), 0.0);
            currentMeans[i] = currentSums[i] / currentData[i].size();
        }
        averagePowerDraw = std::accumulate(currentMeans.begin(), currentMeans.end(), 0.0);
        ampHours = averagePowerDraw * (((((double) accTime) / 1000.0) / 60.0) / 60.0);
        powerFitness = (fabs(accPos) / 1000.0) / ampHours; // m / Ah

        // Calculate speed fitness value:
        float calculatedInferredSpeed = (fabs(accPos) / 1000.0) / (((double) accTime / 1000.0) / 60); // speed in m/min

        float mocapDist = sqrt(pow(startPosition[0] - lastSavedPosition[0],2) + pow(startPosition[1] - lastSavedPosition[1],2) + pow(startPosition[2] - lastSavedPosition[2],2));
        float mocapSpeed = mocapDist / (((double) accTime / 1000.0) / 60);

        ROS_INFO("Origin: %.2f, %.2f, %.2f\n", startPosition[0], startPosition[1], startPosition[2]);
        ROS_INFO("End:    %.2f, %.2f, %.2f\n", lastSavedPosition[0], lastSavedPosition[1], startPosition[2]);

        // Calculate mocap fitness value
        ROS_INFO("Distance: %.2f (S: %.2f)\n", mocapDist, mocapSpeed);

        // Assign fitness values:
        results[0] = calculatedInferredSpeed;            // Inferred speed in in m/min
        results[1] = - (SDs[0] + SDs[1] + SDs[2]);       // angVel
        results[2] = - (SDs[3] + SDs[4] + SDs[5]);       // linAcc
        results[3] = - (SDs[6] + SDs[7] + SDs[8]);       // Combined angle stability (roll + pitch)
        results[4] = powerFitness;                       // Efficiency
        results[5] = mocapSpeed;                         // Inferred speed in m/min
        results[6] = results[3] + (results[2] / 50.0);   // Combined IMU stability
      }

      break;
  };

  res.results = results;

}

void mocapDataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  currentPosition[0] = msg->pose.position.x;
  currentPosition[1] = msg->pose.position.y;
  currentPosition[2] = msg->pose.position.z;

  if (enableCapture){
      lastSavedPosition[0] = msg->pose.position.x;
      lastSavedPosition[1] = msg->pose.position.y;
      lastSavedPosition[2] = msg->pose.position.z;

      if (loggingEnabled == true){
          fprintf(mocapLog, "%lld, ", getMs() - startTime);
          fprintf(mocapLog, "%f, %f, %f, %f, %f, %f, %f\n",
                  msg->pose.position.x,
                  msg->pose.position.y,
                  msg->pose.position.z,
                  msg->pose.orientation.w,
                  msg->pose.orientation.x,
                  msg->pose.orientation.y,
                  msg->pose.orientation.z);
      }
  }
}

void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg){
  if (enableCapture){

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

    if (loggingEnabled == true){
        fprintf(imuLog, "%lld, ", std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() - startTime);
        fprintf(imuLog, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                                                  msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
                                                  roll, pitch, yaw);
    }
  }

  linAcc_z = msg->linear_acceleration.z;
}

void servoStatesCallback(const dyret_common::ServoStateArray::ConstPtr& msg){
  if (enableCapture){
      for(int i = 0; i < 12; i++){
          currentData[i].push_back(msg->servoStates[i].current);
      }
  }
}

void gaitInferredPos_Callback(const dyret_common::DistAng::ConstPtr& msg)
{
  if (msg->msgType == msg->t_measurementInferred){
      accPos += msg->distance;
  } else if (msg->msgType == msg->t_measurementCalculated){
      receivedCalculated = msg->distance;
  }
}

int main(int argc, char **argv) {
  lastSavedPosition.resize(3); // 3D mocap
  currentPosition.resize(3);
  startPosition.resize(3); // 3D mocap
  imuData.resize(9);      // 9 axes
  currentData.resize(12); // 12 servos
  imuLog = NULL;
  mocapLog = NULL;
  accPos = 0.0;

  enableCapture = false;
  linAcc_z = 9.81;

  ros::init(argc, argv, "gaitEvaluator");
  ros::NodeHandle n;

  ros::Subscriber imuData_sub = n.subscribe("/imu/data", 100, imuDataCallback);
  ros::Subscriber gaitInferredPos_sub = n.subscribe("gaitInferredPos", 1000, gaitInferredPos_Callback);
  ros::ServiceServer gaitEvalService = n.advertiseService("get_gait_evaluation", getGaitEvaluationService);
  ros::Subscriber servoStates_sub = n.subscribe("/dyret/servoStates", 1, servoStatesCallback);
  ros::Subscriber mocapData_sub = n.subscribe("/Robot_1/pose", 5, mocapDataCallback);

  //waitForRosInit(gaitInferredPos_sub, "gaitInferredPos");
  //waitForRosInit(imuData_sub, "imuData");
  //waitForRosInit(servoStates_sub, "servoStates");

//  signal(SIGINT, discardFunction);

  while(ros::ok()) ros::spin();

}
