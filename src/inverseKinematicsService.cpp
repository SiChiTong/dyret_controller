#include <math.h> 
#include <vector>

#include "ros/ros.h"
#include "robo_cont_types/calculate_inverse_kinematics.h"
#include "robotConstants.h"

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

bool add(robo_cont_types::calculate_inverse_kinematics::Request  &req,
         robo_cont_types::calculate_inverse_kinematics::Response &res)
{
  ROS_INFO("request: x=%.2f, y=%.2f, z=%.2f", req.point.x, req.point.y, req.point.z);

  std::vector<robo_cont_types::inverse_kinematics_solution> solutions(4);
  
  double t1_1 = atan2(req.point.y,req.point.x);
	double t1_2 = M_PI + t1_1;

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

	double L13_1 = sqrt(pow((req.point.x - p1_x_1), 2) + pow((req.point.y - p1_y_1), 2) + pow((req.point.z - p1_z_1), 2));
	double L13_2 = sqrt(pow((req.point.x - p1_x_2), 2) + pow((req.point.y - p1_y_2), 2) + pow((req.point.z - p1_z_2), 2));

  // Calculate first two solutions
  solutions[0].anglesInRad[2] = M_PI - acos((pow(L12,2) + pow(L23,2) - pow(L13_1,2)) / (2 * L12*L23));
	solutions[1].anglesInRad[2] = (2 * M_PI) - solutions[0].anglesInRad[2];

  vec2A t2Angles = calculateT2(req.point.x, p1_x_1, req.point.y, p1_y_1, req.point.z, L12, L13_1, L23);
	solutions[0].anglesInRad[1] = t2Angles.angles[0];
	solutions[1].anglesInRad[1] = t2Angles.angles[1];
  
  // Calculate second two solutions
  solutions[2].anglesInRad[2] = M_PI - acos((pow(L12,2) + pow(L23,2) - pow(L13_2,2)) / (2 * L12*L23));
	solutions[3].anglesInRad[2] = (2 * M_PI) - solutions[2].anglesInRad[2];

	t2Angles = calculateT2(req.point.x, p1_x_2, req.point.y, p1_y_2, req.point.z, L12, L13_2, L23);
  solutions[2].anglesInRad[1] = t2Angles.angles[0];
	solutions[3].anglesInRad[1] = t2Angles.angles[1];

  // Delete invalid solutions
	if ((L13_1 > L12 + L23) || (L13_1 + L12 < L23)){
    // Delete first two solutions
    solutions.erase(solutions.begin()); 
    solutions.erase(solutions.begin()); 
  }

/* DEBUG:
	if (L13_2 > L12 + L23 || (L13_2 + L12 < L23)) {
    // Delete second two solutions
    solutions.erase(solutions.end());
    solutions.erase(solutions.end());
  }
*/

	solutions.erase(solutions.end());
  solutions.erase(solutions.end());

  res.solutions = solutions;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calculate_inverse_kinematics_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", add);
  ROS_INFO("Ready to add calculate inverse kinematics.");
  ros::spin();

  return 0;
}
