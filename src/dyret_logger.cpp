#include "ros/ros.h"
#include "ros/console.h"
#include <rosbag/bag.h>

#include "dyret_common/State.h"
#include "dyret_common/Pose.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include  "sensor_msgs/CompressedImage.h"

#include "dyret_controller/LoggerCommand.h"

rosbag::Bag bag;
bool loggingEnabled = false;
bool alreadySavedImage = false;

ros::Time lastSavedDepthTime;
ros::Time lastSavedColorTime;

bool loggerCommandCallback(dyret_controller::LoggerCommand::Request  &req,
                           dyret_controller::LoggerCommand::Response &res) {

    std::string bagPath = req.logPath + "/" + req.individual + ".bag";

    if (req.command == req.INIT_LOG){
        ROS_INFO("Received INIT_LOG command with path \"%s\" and individual \"%s\"", req.logPath.c_str(), req.individual.c_str());
        bag.open(bagPath.c_str(), rosbag::bagmode::Write);
        bag.setCompression(rosbag::CompressionType::LZ4);
        alreadySavedImage = false;
    } else if (req.command == req.ENABLE_LOGGING){
        ROS_INFO("Received START_LOGGING command");
        loggingEnabled = true;
    } else if (req.command == req.DISABLE_LOGGING){
        ROS_INFO("Received DISABLE_LOGGING command");
        loggingEnabled = false;
    } else if (req.command == req.SAVE_LOG) {
        ROS_INFO("Received SAVE_LOG command");
        loggingEnabled = false;
        bag.close();
    } else {
        ROS_ERROR("Received unknown command %d", req.command);
    }

    return true;

}

void stateCallback(const dyret_common::State::ConstPtr &msg) {
    if (loggingEnabled){
        try {
            bag.write("/dyret/state", msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing state message to log: %s", e.what());
        }
    }
}

void commandCallback(const dyret_common::Pose::ConstPtr &msg) {
    if (loggingEnabled){
        try {
            bag.write("/dyret/command", msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing command message to log: %s", e.what());
        }
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    if (loggingEnabled){
        try {
            bag.write("/dyret/sensor/imu", msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing imu message to log: %s", e.what());
        }
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (loggingEnabled){
        try {
            bag.write("/dyret/sensor/pose", msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing pose message to log: %s", e.what());
        }
    }
}

void optoforceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &topic) {
    if (loggingEnabled){
        try {
            bag.write(topic, msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing optoforce message to log: %s", e.what());
        }
    }
}

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (loggingEnabled){
        try {
            bag.write("/dyret/sensor/camera/pointcloud", msg->header.stamp, msg);
        } catch (rosbag::BagException e){
            ROS_ERROR("Exception while writing pointcloud message to log: %s", e.what());
        }
    }
}

// Only save the depth image once a second
void depthImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    if (loggingEnabled){
        ros::Time currentTime = ros::Time::now();
        if ((currentTime - lastSavedDepthTime) > ros::Duration(1)) {
            lastSavedDepthTime = currentTime;

            try {
                bag.write("/dyret/sensor/camera/depth", msg->header.stamp, msg);
            } catch (rosbag::BagException e) {
                ROS_ERROR("Exception while writing depth image message to log: %s", e.what());
            }
        }
    }
}

void colorImageCallback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    if (loggingEnabled){
        ros::Time currentTime = ros::Time::now();
        if ((currentTime - lastSavedColorTime) > ros::Duration(1)) {
            lastSavedColorTime = currentTime;

            try {
                bag.write("/dyret/sensor/camera/color/compressed", msg->header.stamp, msg);
                alreadySavedImage = true;
            } catch (rosbag::BagException e) {
                ROS_ERROR("Exception while writing color image message to log: %s", e.what());
            }
        }
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dyret_logger");
  ros::NodeHandle n;

  ros::ServiceServer loggerCommandServer = n.advertiseService("/dyret/dyret_logger/loggerCommand", loggerCommandCallback);

  ros::Subscriber state_sub = n.subscribe("/dyret/state", 100, stateCallback);
  ros::Subscriber command_sub = n.subscribe("/dyret/command", 100, commandCallback);
  ros::Subscriber imu_sub = n.subscribe("/dyret/sensor/imu", 100, imuCallback);
  ros::Subscriber pose_sub = n.subscribe("/dyret/sensor/pose", 100, poseCallback);

  ros::Subscriber optoforce_bl_raw_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/bl", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/raw/contact/bl"));
  ros::Subscriber optoforce_br_raw_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/br", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/raw/contact/br"));
  ros::Subscriber optoforce_fl_raw_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/fl", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/raw/contact/fl"));
  ros::Subscriber optoforce_fr_raw_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/raw/contact/fr", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/raw/contact/fr"));

  ros::Subscriber optoforce_bl_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/bl", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/contact/bl"));
  ros::Subscriber optoforce_br_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/br", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/contact/br"));
  ros::Subscriber optoforce_fl_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fl", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/contact/fl"));
  ros::Subscriber optoforce_fr_sub = n.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fr", 100, boost::bind(optoforceCallback, _1, "/dyret/sensor/contact/fr"));

  ros::Subscriber pointcloud_sub = n.subscribe("/dyret/sensor/camera/pointcloud", 1, pointcloudCallback);
  ros::Subscriber depthimage_sub = n.subscribe("/dyret/sensor/camera/depth", 1, depthImageCallback);
  ros::Subscriber colorimage_sub = n.subscribe("/dyret/sensor/camera/color/compressed", 1, colorImageCallback);

  ROS_INFO("dyret_logger running");

  lastSavedDepthTime = ros::Time::now();
  lastSavedColorTime = ros::Time::now();

  ros::spin();

}
