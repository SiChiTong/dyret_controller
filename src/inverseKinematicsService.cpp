#include <math.h> 
#include <vector>

#include "ros/ros.h"
#include "dyret_common/CalculateInverseKinematics.h"
#include "dyret_common/dimensions.h"
#include "dyret_common/ActuatorStates.h"

#include "dyret_common/angleConv.h"

double femurActuatorLength, tibiaActuatorLength;

struct vec2A {
	double angles[2];
};

vec2A calculateT2(double x, double p1_x, double y, double p1_y, double z, double L12, double L13, double L23) {
	vec2A *vectorToReturn = new vec2A();

	double L13xy = sqrt(pow((x - p1_x),2) + pow((y - p1_y),2));
	double L13z = z;

	double t2_bot = atan(L13z / L13xy);
	double t2_tot = acos((pow(L12,2) + pow(L13,2) - pow(L23,2)) / (2 * L12*L13));

	if ( (fabs(x) < fabs(p1_x)) || (x < 0 && p1_x > 0) || (y < 0 && p1_y > 0) || (y > 0 && p1_y > 0 && (fabs(y) < fabs(p1_y))) || (x > 0 && p1_x < 0 && (fabs(x) >= fabs(p1_x)) && y >= 0) ) {
		vectorToReturn->angles[1] = M_PI - (t2_bot - t2_tot);
		vectorToReturn->angles[0] = M_PI - (t2_tot + t2_bot);
	} else {
		vectorToReturn->angles[0] = t2_bot - t2_tot;
		vectorToReturn->angles[1] = t2_tot + t2_bot;
	}

	return *vectorToReturn;
}


double round(double originalNumber, int decimals) {
	double num = originalNumber * pow(10.0, (decimals));
	long long int rounded = (long long int)((double)num + 0.5);
	double numberToReturn = ((double)rounded) / pow(10.0, (decimals));
	return numberToReturn;
}

bool calculateInverseKinematics(dyret_common::CalculateInverseKinematics::Request  &req,
		 dyret_common::CalculateInverseKinematics::Response &res)
{
  double point_x = req.point.x;
  double point_y = req.point.z;
  double point_z = -req.point.y;

//  ROS_ERROR("Inverse kinematics on %.2f, %.2f, %.2f", point_x, point_y, point_z);

  std::vector<dyret_common::InverseKinematicsSolution> solutions(4);

  double t1_1 = atan2(point_y,point_x);
  double t1_2 = M_PI + t1_1;

  double L12_corrected = L12 + femurActuatorLength;
  double L23_corrected = L23 + tibiaActuatorLength;

  solutions[0].anglesInRad[0] = t1_1;
  solutions[1].anglesInRad[0] = t1_1;
  solutions[2].anglesInRad[0] = t1_2;
  solutions[3].anglesInRad[0] = t1_2;

  double p1_x_1 = round(cos(t1_1) * 72.0,10);
  double p1_y_1 = round(sin(t1_1) * 72.0,10);
  double p1_z_1 = 0.0;

  double p1_x_2 = round(cos(t1_2) * 72.0,10);
  double p1_y_2 = round(sin(t1_2) * 72.0,10);
  double p1_z_2 = 0.0;

  double L13_1 = sqrt(pow((point_x - p1_x_1), 2) + pow((point_y - p1_y_1), 2) + pow((point_z - p1_z_1), 2));
  double L13_2 = sqrt(pow((point_x - p1_x_2), 2) + pow((point_y - p1_y_2), 2) + pow((point_z - p1_z_2), 2));

  // Calculate first two solutions
  solutions[0].anglesInRad[2] = M_PI - acos((pow(L12_corrected,2) + pow(L23_corrected,2) - pow(L13_1,2)) / (2 * L12_corrected*L23_corrected));
  solutions[1].anglesInRad[2] = (2 * M_PI) - solutions[0].anglesInRad[2];

  vec2A t2Angles = calculateT2(point_x, p1_x_1, point_y, p1_y_1, point_z, L12_corrected, L13_1, L23_corrected);
  solutions[0].anglesInRad[1] = t2Angles.angles[0];
  solutions[1].anglesInRad[1] = t2Angles.angles[1];
  
  // Calculate second two solutions
  solutions[2].anglesInRad[2] = M_PI - acos((pow(L12_corrected,2) + pow(L23_corrected,2) - pow(L13_2,2)) / (2 * L12_corrected*L23_corrected));
  solutions[3].anglesInRad[2] = (2 * M_PI) - solutions[2].anglesInRad[2];

  t2Angles = calculateT2(point_x, p1_x_2, point_y, p1_y_2, point_z, L12_corrected, L13_2, L23_corrected);
  solutions[2].anglesInRad[1] = t2Angles.angles[0];
  solutions[3].anglesInRad[1] = t2Angles.angles[1];

  // Delete invalid solutions
  if ((L13_1 > L12_corrected + L23_corrected) || (L13_1 + L12_corrected < L23_corrected)){
    // Delete first two solutions
    solutions.erase(solutions.begin()); 
    solutions.erase(solutions.begin()); 
  }

  solutions.erase(solutions.end());
  solutions.erase(solutions.end());

  for (int i = 0; i < solutions.size(); i++){
    for (int j = 0; j < 3; j++){
      solutions[i].anglesInRad[j] = normalizeRad(solutions[i].anglesInRad[j]);
    }
    solutions[i].anglesInRad[0] = -(solutions[i].anglesInRad[0] + M_PI/2.0);
  }

  res.solutions = solutions;

/*  ROS_INFO("request: x=%.2f, y=%.2f, z=%.2f", point_x, point_y, point_z);
  for (int i = 0; i < solutions.size(); i++){
    ROS_INFO("%.2f, %.2f, %.2f", solutions[i].anglesInRad[0], solutions[i].anglesInRad[1], solutions[i].anglesInRad[2]);
  }*/

  return true;
}

void actuatorState_Callback(const dyret_common::ActuatorStates::ConstPtr& msg){

    if (msg->position.size() == 8) {
        femurActuatorLength = (msg->position[0] + msg->position[2] + msg->position[4] + msg->position[6]) / 4.0;
        tibiaActuatorLength = (msg->position[1] + msg->position[3] + msg->position[5] + msg->position[7]) / 4.0;
    } else {
        ROS_ERROR("ActuatorState did not contain the 8 positions needed by inverseKinematicsService");
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_inverse_kinematics_server");
  ros::NodeHandle n;

  // Initiate actuator positions to 0:
  femurActuatorLength = 0.0;
  tibiaActuatorLength = 0.0;

  ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", calculateInverseKinematics);
  ros::Subscriber gaitInferredPos_sub = n.subscribe("actuatorStates", 1000, actuatorState_Callback);

  ROS_INFO("Ready to calculate inverse kinematics.");
  ros::spin();

  return 0;
}
