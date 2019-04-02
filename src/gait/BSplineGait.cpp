#include <vector>
#include "BSplineGait.h"
#include "../external/splineLibrary/utils/arclength.h"
#include "../kinematics/kinematicFunctions.h"

std::string getDateString() {
    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);

    std::stringstream ss;

    ss << now->tm_year + 1900
       << std::setw(2) << std::setfill('0') << now->tm_mon + 1
       << std::setw(2) << std::setfill('0') << now->tm_mday
       << std::setw(2) << std::setfill('0') << now->tm_hour
       << std::setw(2) << std::setfill('0') << now->tm_min
       << std::setw(2) << std::setfill('0') << now->tm_sec;

    return ss.str();

}

void BSplineGait::writeGaitToFile(std::string logFilePath){
  // Write spline to file:
  float numberOfPointsToGenerate = 1000.0;

  std::string dateString = getDateString();

  // Save raw points:
  FILE * fp;
  fp = fopen((logFilePath + "_raw.csv").c_str(), "w");

  for (int i = 0; i < numberOfPointsToGenerate; i++) {
    float scaled = (float) (totalLength / numberOfPointsToGenerate) * i;

    Vector3 currentPoint = bSpline->getPosition(ArcLength::solveLengthCyclic(*bSpline, 0.0f, scaled));

    double z = currentPoint[2];
    if (z < groundHeight) z = groundHeight; // Add same ground limit we have in getPosition

    fprintf(fp, "%.2f, %.2f, %.2f\n", scaled, currentPoint[1], z);

  }

  fclose(fp);

  // Save control points:
  fp = fopen((logFilePath + "_cnt.csv").c_str(), "w");

  for (int i = 0; i < controlPoints.size(); i++) {
    fprintf(fp, "%.2f, %.2f, %.2f\n", controlPoints[i].x(), controlPoints[i].y(), controlPoints[i].z());

  }

  fclose(fp);

  // Save description:
  fp = fopen((logFilePath + ".txt").c_str(), "w");

  fprintf(fp, "%s", gaitDescriptionString.c_str());

  fclose(fp);


}

void BSplineGait::writeGaitToFile(std::vector<vec3P> customPoints, LoopingCubicHermiteSpline<Vector3> customSpline, int index){
    // Write spline to file:
    float numberOfPointsToGenerate = 1000.0;

    std::string dateString = getDateString();

    // Save raw points:
    FILE * fp;
    fp = fopen(std::string("/home/tonnesfn/catkin_ws/customLogs/lowLevelSplineGait/" + std::to_string(index) + "_" + std::to_string(gaitDifficultyFactor) + "_raw.csv").c_str(), "w");

    for (int i = 0; i < numberOfPointsToGenerate; i++) {
        float scaled = (float) (customSpline.totalLength() / numberOfPointsToGenerate) * i;

        Vector3 currentPoint = customSpline.getPosition(ArcLength::solveLengthCyclic(customSpline, 0.0f, scaled));

        double z = currentPoint[2];
        if (z < groundHeight) z = groundHeight; // Add same ground limit we have in getPosition

        fprintf(fp, "%.2f, %.2f, %.2f\n", scaled, currentPoint[1], z);

    }

    fclose(fp);

    // Save control points:
    fp = fopen(std::string("/home/tonnesfn/catkin_ws/customLogs/lowLevelSplineGait/" + std::to_string(index) + "_" + std::to_string(gaitDifficultyFactor) + "_cnt.csv").c_str(), "w");

    for (int i = 0; i < customPoints.size(); i++) {
        fprintf(fp, "%.2f, %.2f, %.2f\n", customPoints[i].x(), customPoints[i].y(), customPoints[i].z());

    }

    fclose(fp);

    sleep(2);

}

void BSplineGait::initHighLevelGait(double givenStepHeight,
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
    controlPoints =
    {
        { 0.0f, (float)                    (givenStepLength/2.0f), (float)                            givenGroundHeight }, // Front ground
        { 0.0f, (float)                   -(givenStepLength/2.0f), (float)                            givenGroundHeight }, // Back ground
        { 0.0f, (float)                 -((givenStepLength/2.0f)), (float) (givenGroundHeight + (givenStepHeight/1.5f)) }, // Back smoothing
        { 0.0f,                                              0.0f, (float)        (givenGroundHeight + givenStepHeight) }, // Top
        { 0.0f, (float) ((givenStepLength/2.0f) + givenSmoothing), (float) (givenGroundHeight + (givenStepHeight/4.0f)) }, // Front smoothing
    };

    // Make the spline object:
    std::vector<Vector3> gaitPoints(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); i++) gaitPoints[i] = Vector3({(float) controlPoints[i].x(), (float) controlPoints[i].y(), (float) controlPoints[i].z()});

    printf("highLevelPoints: \n");
    for (int i = 0; i < controlPoints.size(); i++) printf("    %.2f, %.2f, %.2f\n", controlPoints[i].x(), controlPoints[i].y(), controlPoints[i].z());

    bSpline = new LoopingCubicHermiteSpline<Vector3>(gaitPoints, 0.5);

    // Get total length calculation from new spline
    totalLength = bSpline->totalLength();

    gaitDescriptionString.clear();
    gaitDescriptionString += "HighLevelGait:\n  ";

    for (int i = 0; i < 5; i++){
        gaitDescriptionString += "    " + std::to_string(controlPoints[i].x()) + ", " + std::to_string(controlPoints[i].y()) + ", " + std::to_string(controlPoints[i].z()) + "\n";
    }

}

float getMapValue(std::map<std::string, float> givenMap, std::string givenKey); // Defined in gaitController.cpp

void BSplineGait::initLowLevelGait(std::map<std::string, float> gaitConfiguration, double givenGroundHeight){

    assert(getMapValue(gaitConfiguration, "liftDuration") >= 0.05f && getMapValue(gaitConfiguration, "liftDuration") <= 0.2f); // liftDuration has to be between 5% and 20%
    assert(getMapValue(gaitConfiguration, "difficultyFactor") >= 0.0f && getMapValue(gaitConfiguration, "difficultyFactor") <= 1.0f); // Difficulty factor has to be between 0% and 100%

    gaitDifficultyFactor = getMapValue(gaitConfiguration, "difficultyFactor");

    ROS_INFO("Initializing lowLevelGait with difficulty %.1f", getMapValue(gaitConfiguration, "difficultyFactor"));

    groundPercentGoal = 1.0 - getMapValue(gaitConfiguration, "liftDuration");
    groundHeight = givenGroundHeight;
    offsetFront = 0.0;
    spreadAmount = 80.0;
    rearLegOffset = -30.0;
    totalLength = 99999.0; // Init length


    // Calculate gait points (the two ground points need to be first):
    controlPoints.clear();

    if (getMapValue(gaitConfiguration, "p0_y") > getMapValue(gaitConfiguration, "p1_y")) {
      controlPoints.push_back({getMapValue(gaitConfiguration, "p0_x"), getMapValue(gaitConfiguration, "p0_y"), (float) givenGroundHeight});
      controlPoints.push_back({getMapValue(gaitConfiguration, "p1_x"), getMapValue(gaitConfiguration, "p1_y"), (float) givenGroundHeight});
    } else {
      controlPoints.push_back({getMapValue(gaitConfiguration, "p1_x"), getMapValue(gaitConfiguration, "p1_y"), (float) givenGroundHeight});
      controlPoints.push_back({getMapValue(gaitConfiguration, "p0_x"), getMapValue(gaitConfiguration, "p0_y"), (float) givenGroundHeight});
    }

    stepLength = controlPoints[0].y() - controlPoints[1].y();

    // Try all combinations of point order to find one without self-intersection:

    std::vector<vec3P> airPoints = {{getMapValue(gaitConfiguration, "p2_x"), getMapValue(gaitConfiguration, "p2_y"), (float) givenGroundHeight + getMapValue(gaitConfiguration, "p2_z")},
                                    {getMapValue(gaitConfiguration, "p3_x"), getMapValue(gaitConfiguration, "p3_y"), (float) givenGroundHeight + getMapValue(gaitConfiguration, "p3_z")},
                                    {getMapValue(gaitConfiguration, "p4_x"), getMapValue(gaitConfiguration, "p4_y"), (float) givenGroundHeight + getMapValue(gaitConfiguration, "p4_z")}};

    std::vector<int> indexes = {0, 1, 2};
    int i = 0;

    do {

        std::vector<vec3P> tempPoints;

        tempPoints.push_back(controlPoints[0]);
        tempPoints.push_back(controlPoints[1]);

        for (int i = 0; i < 3; i++){
            tempPoints.push_back(airPoints[indexes[i]]);
        }

        // Make the spline object:
        std::vector<Vector3> gaitPoints(tempPoints.size());
        for (int i = 0; i < tempPoints.size(); i++)
            gaitPoints[i] = Vector3({(float) tempPoints[i].x(), (float) tempPoints[i].y(), (float) tempPoints[i].z()});

/*        printf("lowLevelPoints: \n");
        for (int i = 0; i < tempPoints.size(); i++)
            printf("    %.2f, %.2f, %.2f\n", tempPoints[i].x(), tempPoints[i].y(), tempPoints[i].z());*/

        LoopingCubicHermiteSpline<Vector3>* tmpBSpline = new LoopingCubicHermiteSpline<Vector3>(gaitPoints, 0.5);

        // Get total length calculation from new spline
        double tmpTotalLength = tmpBSpline->totalLength();

        if (tmpTotalLength < totalLength) {
            //printf("Replacing original spline (%.2f) with new (%.2f)\n", totalLength, tmpTotalLength);
            controlPoints = tempPoints;
            totalLength = tmpTotalLength;
            delete(bSpline);
            bSpline = tmpBSpline;
        } else {
            delete(tmpBSpline);
        }

        i+=1;
    } while (std::next_permutation(indexes.begin(), indexes.end()));

    gaitDescriptionString.clear();
    gaitDescriptionString += "lowLevelGait:\n";

    gaitDescriptionString += "  points:\n";
    for (int i = 0; i < 5; i++){
        gaitDescriptionString += "    " + std::to_string(controlPoints[i].x()) + ", " + std::to_string(controlPoints[i].y()) + ", " + std::to_string(controlPoints[i].z()) + "\n";
    }

    gaitDescriptionString += "  config:\n";
    for(auto elem : gaitConfiguration){
        gaitDescriptionString += std::string("    ") + elem.first.c_str() + ": " + std::to_string(elem.second) + "\n";
    }
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

		// Apply spread
		vecRet[0] = vecRet[0] - spreadAmount;

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

	// Invert X for right legs:
    vectorToReturn[1].points[0] = -vectorToReturn[1].points[0];
    vectorToReturn[2].points[0] = -vectorToReturn[2].points[0];

	return vectorToReturn;
}
