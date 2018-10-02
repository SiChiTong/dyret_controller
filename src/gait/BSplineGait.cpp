#include <vector>
#include "BSplineGait.h"
#include "../external/splineLibrary/utils/arclength.h"
#include "../kinematics/kinematicFunctions.h"

void BSplineGait::writeGaitToFile(){
  // Write spline to file:
  float numberOfPointsToGenerate = 1000.0;

  FILE * fp;
  fp = fopen("/home/tonnesfn/catkin_ws/splineGaitOutput.csv", "w");

  fprintf(fp, "stepLength: %.3f, totalLength: %.3f, groundPercentGoal: %.3f\n",
          stepLength,
          totalLength,
          groundPercentGoal);

  for (float i = 0; i < numberOfPointsToGenerate; i++) {
    float scaled = (float) (totalLength / numberOfPointsToGenerate) * i;

    Vector3 currentPoint = bSpline->getPosition(ArcLength::solveLengthCyclic(*bSpline, 0.0f, scaled));

    fprintf(fp, "%.2f, %.2f, %.2f\n", scaled, currentPoint[1], currentPoint[2]);

  }

  fclose(fp);
}

void BSplineGait::initGait(double givenStepHeight,
                           double givenStepLength,
                           double givenSmoothing,
                           double givenGroundHeight,
                           double givenSpread,
                           double givenOffsetFront,
                           double givenRearLegOffset,
                           double givenLiftDuration){

    assert(givenLiftDuration >= 0.05f && givenLiftDuration <= 0.2f); // liftDuration has to be between 5% and 20%

    // Set class variables:
    stepLength = givenStepLength;
    offsetFront = givenOffsetFront;
    spreadAmount = givenSpread;
    groundHeight = givenGroundHeight;
    rearLegOffset = givenRearLegOffset;
    groundPercentGoal = 1.0 - givenLiftDuration;

    // Calculate gait points (the two ground points need to be first):
    std::vector<vec3P> calculatedGaitPoints =
    {
        { 0.0f, (float)                    (givenStepLength/2.0f), (float)                            givenGroundHeight }, // Front ground
        { 0.0f, (float)                   -(givenStepLength/2.0f), (float)                            givenGroundHeight }, // Back ground
        { 0.0f, (float)                 -((givenStepLength/2.0f)), (float) (givenGroundHeight + (givenStepHeight/1.5f)) }, // Back smoothing
        { 0.0f,                                              0.0f, (float)        (givenGroundHeight + givenStepHeight) }, // Top
        { 0.0f, (float) ((givenStepLength/2.0f) + givenSmoothing), (float) (givenGroundHeight + (givenStepHeight/4.0f)) }, // Front smoothing
    };

    // Check lowest Z point and adjust ground height accordingly:
    for (int i = 0; i < calculatedGaitPoints.size(); i++) if (calculatedGaitPoints[i].points[2] < groundHeight) groundHeight = calculatedGaitPoints[i].points[2];

    // Make the spline object:
    std::vector<Vector3> gaitPoints(calculatedGaitPoints.size());
    for (int i = 0; i < calculatedGaitPoints.size(); i++) gaitPoints[i] = Vector3({(float) calculatedGaitPoints[i].x(), (float) calculatedGaitPoints[i].y(), (float) calculatedGaitPoints[i].z()});

    bSpline = new LoopingCubicHermiteSpline<Vector3>(gaitPoints, 0.5);

    // Get total length calculation from new spline
    totalLength = bSpline->totalLength();

}

std::vector<vec3P> BSplineGait::getPosition(double givenTime, bool walkingForwards) {

	std::vector<vec3P> vectorToReturn(4);

	// For all legs:
	for (int i = 0; i < 4; i++) {
		// Correct for multiplier:
        double currentTime = givenTime / 1000;

		// Use correct leg phase:
        double currentLegPhaseOffset = legPhaseOffset[i][0];
        if (walkingForwards == false) currentLegPhaseOffset = legPhaseOffset[i][1];

		// Calculate with offset:
		currentTime += currentLegPhaseOffset;

		// Normalize time within one period
		double relativeTime = fmod(currentTime, 1.0);

		// Calculate the spline position, constrained by the liftDuration
        double splinePosition;
        if (relativeTime < groundPercentGoal) { // We are supposed to be on the ground
            splinePosition = (relativeTime / groundPercentGoal) * stepLength;
        } else { // We are supposed to be in the air
            splinePosition = stepLength + (totalLength - stepLength) * ((relativeTime - groundPercentGoal) * (1.0f / (1.0f - groundPercentGoal)));
        }

        // Get the point from the spline object
        Vector3 vecRet = bSpline->getPosition(ArcLength::solveLengthCyclic(*bSpline, 0.0f, (float) splinePosition));

        // Apply offsetFront so it always point forwards
		if (walkingForwards) vecRet[1] = vecRet[1] + offsetFront; else vecRet[1] = vecRet[1] - offsetFront;

        // Apply spread so it always points to the outside of the robot
		if (i == 0 || i == 3) vecRet[0] = vecRet[0] - spreadAmount; else vecRet[0] = vecRet[0] + spreadAmount;



		// Make sure the point we got from the spline object is not lower than the groundHeight
		if (vecRet[2] < groundHeight) vecRet[2] = groundHeight;

		vectorToReturn[i] = vec3P(vecRet[0], vecRet[1], vecRet[2]);
	}

	// Add rearLegOffset to the back two legs, depending on which way the robot is walking
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
