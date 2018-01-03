#pragma once

#include <vector>

#include "../kinematics/kinematicTypes.h"
#include "gait.h"
#include "../external/splineLibrary/spline.h"
#include "../external/splineLibrary/splines/cubic_hermite_spline.h"

class BSplineGait : public Gait {
  private:
    LoopingCubicHermiteSpline<Vector3>* bSpline;

    float groundPercent;

    float groundPercentGoal = 0.90f; //Time spent on the ground (minimum 3/4 = 75%)

    std::vector<vec3P> createBSplineGaitPoints(double stepHeight,
                                               double stepLength,
                                               double smoothing,
                                               double groundHeight);

    void bSplineInit(std::vector<vec3P> givenPoints,
                     float stepLength,
                     float givenLiftDuration);

  public:
    BSplineGait(double stepHeight,
                double stepLength,
                double smoothing,
                double groundHeight,
                double givenSpread,
                double givenOffsetFront,
                double givenOffsetLeft,
                double rearLegOffset,
                double givenLiftDuration);

    vec3P getGaitWagPoint(double givenTime, bool walkingForwards);
    std::vector<vec3P> getPosition(double givenTime, bool walkingForwards);
};
