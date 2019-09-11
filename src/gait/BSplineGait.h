#pragma once

#include <vector>
#include <map>
#include <string>
#include <sstream>

#include "../kinematics/kinematicTypes.h"
#include "../external/splineLibrary/spline.h"
#include "../external/splineLibrary/splines/cubic_hermite_spline.h"

class BSplineGait {
  private:
    LoopingCubicHermiteSpline<Vector3>* bSpline = NULL;

    // Leg phase offsets forwards and reverse, with a leg order of FL, BR, FR, BL
    const double legPhaseOffset[4][2] = { {0.00, 0.75}, {0.50, 0.25}, {0.75, 0.00}, {0.25, 0.50} };
    const double zHeightOffset = 25.0; // How many millimeters we add for scaling 2.0

    double stepLength;
    double offsetFront;
    double totalLength;
    double spreadAmount;
    double rearLegOffset;
    double groundPercentGoal; // Time spent on the ground (minimum 3/4 = 75%)
    double gaitDifficultyFactor = -1;
    float scalingFactor = 1.0;

    std::array<double, 4> groundHeights;

    std::string gaitDescriptionString;
    std::vector<vec3P> controlPoints;

    void writeGaitToFile(std::vector<vec3P> customPoints, LoopingCubicHermiteSpline<Vector3> customSpline, int index);

  public:
    void initHighLevelGait(double stepHeight,
                           double stepLength,
                           double smoothing,
                           std::array<double, 4> givenGroundHeights,
                           double givenSpread,
                           double givenOffsetFront,
                           double rearLegOffset,
                           double givenLiftDuration);

    void initLowLevelGait(std::map<std::string, float> gaitConfiguration, std::array<double, 4> givenGroundHeights);

    std::vector<vec3P> getPosition(double givenTime, bool walkingForwards);

    double getStepLength() { return stepLength;}

    void writeGaitToFile(std::string logFilePath);

};
