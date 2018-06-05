#include <vector>
#include "BSplineGait.h"
#include "../external/splineLibrary/utils/arclength.h"
#include "../kinematics/kinematicFunctions.h"

vec3P BSplineGait::getGaitWagPoint(double givenTime, bool walkingForwards){
  vec3P wagPoint = getWagPoint(getWagAmplitude_x(),
                               getWagAmplitude_y(),
                               1.0,
                               givenTime/1000,
                               getWagPhase());

  if (walkingForwards == false){
    wagPoint.points[0] = -wagPoint.points[0];
    wagPoint.points[1] = -wagPoint.points[1];
  }

  return wagPoint;

}

void BSplineGait::bSplineInit(std::vector<vec3P> givenPoints, float givenStepLength, float givenLiftDuration){
  assert(givenLiftDuration >= 0.05f && givenLiftDuration <= 0.2f); // liftDuration has to be between 5% and 20%
  groundPercent = 1.0f - givenLiftDuration;
  stepLength = givenStepLength;

  // Make spline object:
  std::vector<Vector3> gaitPoints(givenPoints.size());

  for (int i = 0; i < givenPoints.size(); i++) {
    gaitPoints[i] = Vector3({(float) givenPoints[i].x(), (float) givenPoints[i].y(), (float) givenPoints[i].z()});
  }

  bSpline = new LoopingCubicHermiteSpline<Vector3>(gaitPoints, 0.5);

  totalLength = bSpline->totalLength();
  groundPercent = stepLength / bSpline->totalLength();

  // Set offsets:
  legPhaseOffset[0][0] = 0.00; // forwards
  legPhaseOffset[1][0] = 0.50;
  legPhaseOffset[2][0] = 0.75;
  legPhaseOffset[3][0] = 0.25;
  legPhaseOffset[0][1] = 0.75; // reverse:
  legPhaseOffset[1][1] = 0.25;
  legPhaseOffset[2][1] = 0.00;
  legPhaseOffset[3][1] = 0.50;

  // Write spline to file:
  float numberOfPointsToGenerate = 1000.0;

//  FILE * fp;
//  fp = fopen("/home/tonnesfn/catkin_ws/bSplineGaitOutput.csv", "w");

  for (float i = 0; i < 1.0f; i = i + (1.0f / numberOfPointsToGenerate)) {
    float scaled;

    // Test ground scaling:
    if (i < groundPercentGoal){
      scaled = (i / groundPercentGoal) * stepLength;
    } else {
      scaled = stepLength + (totalLength - stepLength) * ((i - groundPercentGoal) * (1.0f / (1.0f - groundPercentGoal)));
    }

    Vector3 currentPoint = bSpline->getPosition(ArcLength::solveLengthCyclic(*bSpline, 0.0f, scaled));
    if (currentPoint[2] < groundHeight) currentPoint[2] = groundHeight; // Stop dips and loops below groundHeight

//    fprintf(fp, "%.2f, %.2f, %.2f\n", i, currentPoint[1], currentPoint[2]);

  }

//  fclose(fp);

}

std::vector<vec3P> BSplineGait::createBSplineGaitPoints(double stepHeight, double stepLength, double smoothing, double groundHeight){

    // The two ground points has to be first for ground scaling to work properly
    std::vector<vec3P> result =
    {
      { 0.0f, (float)               (stepLength/2.0f), (float)                       groundHeight }, // Front ground
      { 0.0f, (float)              -(stepLength/2.0f), (float)                       groundHeight }, // Back ground
      { 0.0f, (float)            -((stepLength/2.0f)), (float) (groundHeight + (stepHeight/1.5f)) }, // Back smoothing
      { 0.0f,                                    0.0f, (float)        (groundHeight + stepHeight) }, // Top
      { 0.0f, (float) ((stepLength/2.0f) + smoothing), (float) (groundHeight + (stepHeight/4.0f)) }, // Front smoothing
    };

    return result;
}

BSplineGait::BSplineGait(double stepHeight,
                         double stepLength,
                         double smoothing,
                         double groundHeight,
                         double givenSpread,
                         double givenOffsetFront,
                         double givenOffsetLeft,
                         double rearLegOffset,
                         double givenLiftDuration)
  : Gait(){
  std::vector<vec3P> calculatedGaitPoints = createBSplineGaitPoints(stepHeight, stepLength, smoothing, groundHeight);

  init(calculatedGaitPoints);
  setSpread(givenSpread);
  setOffsets(givenOffsetFront, givenOffsetLeft, rearLegOffset);
  bSplineInit(calculatedGaitPoints, stepLength, givenLiftDuration);

}

std::vector<vec3P> BSplineGait::getPosition(double givenTime, bool walkingForwards) {

	std::vector<vec3P> vectorToReturn(4);

	// For all legs:
	for (int i = 0; i < 4; i++) {
		// Correct for multiplier:
		float currentTime = givenTime / 1000;

		// Use correct leg phase:
        float currentLegPhaseOffset = legPhaseOffset[i][0];
        if (walkingForwards == false) currentLegPhaseOffset = legPhaseOffset[i][1];

		// Calculate with offset:
		currentTime += currentLegPhaseOffset;

		double relativeTime = fmod(currentTime, 1.0);

        float scaledTime = (relativeTime / groundPercentGoal) * stepLength;
        if (relativeTime >= groundPercentGoal){
          scaledTime = stepLength + (totalLength - stepLength) * ((relativeTime - groundPercentGoal) * (1.0f / (1.0f - groundPercentGoal)));
        }

        Vector3 vecRet = bSpline->getPosition(ArcLength::solveLengthCyclic(*bSpline, 0.0f, (float) scaledTime));

        // OffsetFront should always point forwards
		if (walkingForwards) vecRet[1] = vecRet[1] + offsetFront; else vecRet[1] = vecRet[1] - offsetFront;

		// Spread amount is always to the outside of the robot
		if (i == 0 || i == 3){
          vecRet[0] = vecRet[0] - spreadAmount;
		} else {
          vecRet[0] = vecRet[0] + spreadAmount;
		}

		// Cap spline to groundHeight
		if (vecRet[2] < groundHeight) vecRet[2] = groundHeight;

		vectorToReturn[i] = vec3P(vecRet[0], vecRet[1], vecRet[2]); // Stop dips and loops?
	}

	// Add rearLegOffset:
	if (walkingForwards == true){
	    for (int i = 2; i <= 3; i++) vectorToReturn[i].points[1] = vectorToReturn[i].points[1] + rearLegOffset;
	} else {
	    for (int i = 0; i <= 1; i++) vectorToReturn[i].points[1] = vectorToReturn[i].points[1] + rearLegOffset;
	}

	// Invert Y axis if walking in reverse:
  if (walkingForwards != true){
      for (int i = 0; i < 4; i++) vectorToReturn[i].points[1] = -vectorToReturn[i].points[1];
	}

	return vectorToReturn;
}
