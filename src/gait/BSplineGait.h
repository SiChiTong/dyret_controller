#pragma once

#include <vector>

#include "../kinematics/kinematicTypes.h"
#include "../external/splineLibrary/spline.h"
#include "../external/splineLibrary/splines/cubic_hermite_spline.h"

class BSplineGait {
  private:
    LoopingCubicHermiteSpline<Vector3>* bSpline;

    // Leg phase offsets forwards and reverse, with a leg order of FL, BR, FR, BL
    double const legPhaseOffset[4][2] = { {0.00, 0.75}, {0.50, 0.25}, {0.75, 0.00}, {0.25, 0.50} };

    double stepLength;
    double offsetFront;
    double totalLength;
    double spreadAmount;
    double groundHeight;
    double rearLegOffset;
    double groundPercentGoal; //Time spent on the ground (minimum 3/4 = 75%)

  public:
    void initGait(double stepHeight,
                  double stepLength,
                  double smoothing,
                  double givenGroundHeight,
                  double givenSpread,
                  double givenOffsetFront,
                  double rearLegOffset,
                  double givenLiftDuration);

    std::vector<vec3P> getPosition(double givenTime, bool walkingForwards);

    double getStepLength() { return stepLength;}

    void writeGaitToFile();

};
