#include <math.h>

#include "interpolation.h"

float getInterpolatedLength(vec3P startPosition, vec3P goalPosition) {
	return sqrt(pow(goalPosition.x() - startPosition.x(), 2) + pow(goalPosition.y() - startPosition.y(), 2) + pow(goalPosition.z() - startPosition.z(), 2));
}

float calculateIncrement(vec3P startPosition, vec3P goalPosition, int numberOfIncrements) {
	float calculatedIncrement = 0.0;

	double totalDistance = sqrt(pow(goalPosition.x() - startPosition.x(), 2) + pow(goalPosition.y() - startPosition.y(), 2) + pow(goalPosition.z() - startPosition.z(), 2));
	calculatedIncrement = totalDistance / numberOfIncrements;

	return calculatedIncrement;

}

vec3P incInterpolation(vec3P currentPosition, vec3P goalPosition, double increment) {
	double totalDistance = sqrt(pow(goalPosition.x() - currentPosition.x(), 2) + pow(goalPosition.y() - currentPosition.y(), 2) + pow(goalPosition.z() - currentPosition.z(), 2));
	if (totalDistance < increment) return goalPosition;
	double stepInPercent = increment / totalDistance;

	double x = currentPosition.x() + ((goalPosition.x() - currentPosition.x()) * stepInPercent);
	double y = currentPosition.y() + ((goalPosition.y() - currentPosition.y()) * stepInPercent);
	double z = currentPosition.z() + ((goalPosition.z() - currentPosition.z()) * stepInPercent);

	vec3P* positionToReturn = new vec3P(x, y, z);

	return *positionToReturn;
}

// Linear interpolation, progress given in percent
vec3P lineInterpolation(vec3P startPosition, vec3P goalPosition, float distance) {
  double totalDistance = sqrt(pow(goalPosition.x() - startPosition.x(), 2) + pow(goalPosition.y() - startPosition.y(), 2) + pow(goalPosition.z() - startPosition.z(), 2));
  double stepInPercent = distance / totalDistance;

  double x = goalPosition.x();
  double y = goalPosition.y();
  double z = goalPosition.z();

  if (distance <= totalDistance){ // Check for valid distance, if it is beyond goal, just return goal
      x = startPosition.x() + ((goalPosition.x() - startPosition.x()) * stepInPercent);
      y = startPosition.y() + ((goalPosition.y() - startPosition.y()) * stepInPercent);
      z = startPosition.z() + ((goalPosition.z() - startPosition.z()) * stepInPercent);
  }

  vec3P positionToReturn = vec3P(x, y, z);

  return positionToReturn;
}

