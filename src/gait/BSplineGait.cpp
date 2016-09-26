#include <vector>
#include "BSplineGait.h"
#include "../kinematics/kinematicFunctions.h"

vec3P BSplineGait::getGaitWagPoint(double givenTime){
  return getWagPoint(getWagAmplitude_x(), getWagAmplitude_y(),
                     1.0,
                     givenTime/1000,
                     getWagPhase());
}

void BSplineGait::bSplineInit(std::vector<vec3P> givenPoints, float givenStepLength){
  stepLength = givenStepLength;

  // Make spline object:
  std::vector<Vector3> gaitPoints(givenPoints.size());

  for (int i = 0; i < givenPoints.size(); i++) {
    gaitPoints[i] = Vector3({(float) givenPoints[i].x(), (float) givenPoints[i].y(), (float) givenPoints[i].z()});
  }

  //bSpline = new LoopingUniformCubicBSpline<Vector3>(gaitPoints);
  //bSpline = new LoopingUniformCRSpline<Vector3>(gaitPoints);
  bSpline = new LoopingCubicHermiteSpline<Vector3>(gaitPoints, 0.5);

  // Set max T and groundContactLine
  maxT = bSpline->getMaxT();
  groundContactLine.resize(2);
  groundContactLine[0] = bSpline->getT(3);
  groundContactLine[1] = bSpline->getT(gaitPoints.size()-1);

  // TODO: Clean up this code:
  float groundLength = bSpline->arcLength(groundContactLine[0], groundContactLine[1]);
  float liftLength = bSpline->arcLength(0, groundContactLine[0]) + bSpline->arcLength(groundContactLine[1], bSpline->getMaxT());
  float groundDuration = groundContactLine[1] - groundContactLine[0];
  float airDuration = bSpline->getMaxT() - groundDuration;
  float groundPercent = (groundDuration / (groundDuration + airDuration));

  float factor = 0.9 / groundPercent;
  float newGndDuration = (groundContactLine[1] - groundContactLine[0]) * factor;
  float newGndEnd = groundContactLine[0] + newGndDuration;
  float newSplineEnd = bSpline->getMaxT() + (newGndDuration - groundDuration);

  gndFactor = factor;
  gndStart = groundContactLine[0];
  gndEnd = newGndEnd;
  maxTime = newSplineEnd;

  gndContactPercent = (gndEnd - gndStart) / maxTime;
  printf("gndContactPercent: %.2f\n", gndContactPercent);

  // Find ground contact height:
  double minZ = INFINITY;

  for (int i = 0; i < gaitPoints.size(); i++) {
    if (gaitPoints[i][2] < minZ) minZ = gaitPoints[i][2];
  }

  // Set lengths:
  totalLength = 0.0;
  lengths.resize(gaitPoints.size());

  for (int i = 0; i < gaitPoints.size(); i++) {
    lengths[i] = bSpline->arcLength((float)i, (float)i + 1);
    totalLength += lengths[i];
  }

  // Set groundLength
  groundLength = bSpline->arcLength(groundContactLine[0], groundContactLine[1]);

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
/*  int numberOfPointsToGenerate = 1000;

  FILE * fp;
  fp = fopen("bSplineGaitOutput.csv", "w");

  for (float i = 0; i < bSpline->getMaxT(); i = i + (bSpline->getMaxT() / numberOfPointsToGenerate)) {
    Vector3 currentPoint = bSpline->getPosition(i);
    if (currentPoint[2] < groundHeight) currentPoint[2] = groundHeight; // Stop dips and loops?
    fprintf(fp, "%.2f, %.2f, %.2f\n", i, currentPoint[1], currentPoint[2]);
  }

  fclose(fp);
*/
}

std::vector<vec3P> BSplineGait::createBSplineGaitPoints(double stepHeight, double stepLength, double smoothing, double groundHeight){
    std::vector<vec3P> result =
    {
      { 0.0f, (float)            -((stepLength/2.0f)), (float) (groundHeight + (stepHeight/1.5f)) }, // Back smoothing
      { 0.0f,                                    0.0f, (float)        (groundHeight + stepHeight) }, // Top
      { 0.0f, (float) ((stepLength/2.0f) + smoothing), (float) (groundHeight + (stepHeight/4.0f)) }  // Front smoothing
    };
    // from front to back
    const int N = 22;
    for (int i = 0; i < N; ++i)
      result.emplace_back(0.0f, (stepLength/2.0) - 2.0f * (stepLength/2.0)*(i / (N-1.0f)), groundHeight);
    return result;
}

BSplineGait::BSplineGait(double stepHeight, double stepLength, double smoothing, double groundHeight, double givenSpread, double givenOffsetFront, double givenOffsetLeft, double rearLegOffset)
  : Gait(){
  std::vector<vec3P> calculatedGaitPoints = createBSplineGaitPoints(stepHeight, stepLength, smoothing, groundHeight);

  init(calculatedGaitPoints);
  setSpread(givenSpread);
  setOffsets(givenOffsetFront, givenOffsetLeft, rearLegOffset);
  bSplineInit(calculatedGaitPoints, stepLength);

}

double BSplineGait::applyGndTimeScaling(double givenTime){
  FILE * scaleLog;
//  scaleLog = fopen("scaleLog.csv", "a+");

  if (givenTime < gndStart){
//      fprintf(scaleLog,"%.2f, %.2f\n", givenTime, givenTime);
//      fclose(scaleLog);
      return givenTime; // No correction needed

  } else if (givenTime > gndEnd){ // Correction needed for extra gnd space
      float oldGndEnd = ((gndEnd - gndStart) / gndFactor) + gndStart;
      float retValue = (givenTime - gndEnd) + oldGndEnd;

//      fprintf(scaleLog,"%.2f, %.2f\n", givenTime, retValue);
//      fclose(scaleLog);

      return retValue;

  } else { // Correction needed, in gnd position
      float retValue = ((givenTime - gndStart) / gndFactor) + gndStart;

//      fprintf(scaleLog,"%.2f, %.2f\n", givenTime, retValue);
//      fclose(scaleLog);

      return retValue;
  }

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

		double scaledTime = applyGndTimeScaling((relativeTime) * maxTime);

		Vector3 vecRet = bSpline->getPosition(scaledTime);

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
