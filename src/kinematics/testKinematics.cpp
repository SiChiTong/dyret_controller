#include <vector>

#include "ros/ros.h"

#include "kinematicTypes.h"
#include "forwardKinematics.h"
#include "dyret_common/CalculateInverseKinematics.h"

ros::ServiceClient inverseKinematicsService_client;

std::vector<std::vector<double>> testInverse(double givenX, double givenY, double givenZ){
  dyret_common::CalculateInverseKinematics srv;

  srv.request.point.x = givenX;
  srv.request.point.y = givenY;
  srv.request.point.z = givenZ;

  printf("  IK(%.2f, %.2f, %.2f):\n", givenX, givenY, givenZ);

  inverseKinematicsService_client.call(srv);

  std::vector<std::vector<double>> returnVector;

  for (int i = 0; i < srv.response.solutions.size(); i++){
    printf("    %.2f, %.2f, %.2f\n", srv.response.solutions[i].anglesInRad[0],
                                     srv.response.solutions[i].anglesInRad[1],
                                     srv.response.solutions[i].anglesInRad[2]);
    std::vector<double> subVector(3);
    subVector[0] = srv.response.solutions[i].anglesInRad[0];
    subVector[1] = srv.response.solutions[i].anglesInRad[1];
    subVector[2] = srv.response.solutions[i].anglesInRad[2];

    returnVector.push_back(subVector);

  }

  return returnVector;

}

std::vector<std::vector<double>> testInverse(std::vector<double> givenVector){
  return testInverse(givenVector[0], givenVector[1], givenVector[2]);
}

std::vector<double> testForward(std::vector<double> givenTestVector){
  vec3P result = forwardKinematics(givenTestVector[0], givenTestVector[1], givenTestVector[2], 0.0, 0.0, 0.0);

  printf("  Forward(%.2f,%.2f,%.2f):\n", givenTestVector[0], givenTestVector[1], givenTestVector[2]);
  printf("    %.2f, %.2f, %.2f\n", result.x(), result.y(), result.z());

  std::vector<double> returnVector(3);
  returnVector[0] = result.x();
  returnVector[1] = result.y();
  returnVector[2] = result.z();

  return returnVector;

}

std::vector<std::vector<double>> testForward(std::vector<std::vector<double>> givenTestVector){
  std::vector<std::vector<double>> returnVector;

  for (int i = 0; i < givenTestVector.size(); i++){
    returnVector.push_back(testForward(givenTestVector[i]));
  }

  return returnVector;
}

void testKinematics(double givenX, double givenY, double givenZ){
  printf("Testing %.2f, %.2f, %.2f:\n", givenX, givenY, givenZ);
  std::vector<std::vector<double>> results;

  results = testInverse(givenX, givenY, givenZ);
  results = testForward(results);

}

int main(int argc, char **argv){

  ros::init(argc, argv, "testKinematics");
  ros::NodeHandle n;

  inverseKinematicsService_client = n.serviceClient<dyret_common::CalculateInverseKinematics>("calculate_inverse_kinematics");

  testKinematics( 0.0, 0.0, -450.0);
  testKinematics(20.0, 0.0, -450.0);
  testKinematics(-20.0, 0.0, -450.0);
  testKinematics( 0.0, 20.0, -450.0);
  testKinematics( 0.0, -20.0, -450.0);
  testKinematics(20.0, 20.0, -450.0);
  testKinematics(-20.0, -20.0, -450.0);

}