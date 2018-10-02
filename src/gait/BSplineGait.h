#pragma once

#include <vector>

#include "../kinematics/kinematicTypes.h"
#include "../external/splineLibrary/spline.h"
#include "../external/splineLibrary/splines/cubic_hermite_spline.h"

class BSplineGait {
  private:
    LoopingCubicHermiteSpline<Vector3>* bSpline;

    float groundPercentGoal = 0.90f; //Time spent on the ground (minimum 3/4 = 75%)

    double totalLength = 0.0;
    float spreadAmount  = 0.0;
    float legPhaseOffset[4][2];
    float groundHeight = 0.0;
    float stepLength;

    float offsetFront   = 0.0;
    float rearLegOffset = 0.0;

    float wagPhase = 0.0;
    float wagAmplitude_x = 0.0;
    float wagAmplitude_y = 0.0;

    std::vector<vec3P> createBSplineGaitPoints(double stepHeight,
                                               double stepLength,
                                               double smoothing,
                                               double givenGroundHeight);

    void bSplineInit(std::vector<vec3P> givenPoints,
                     float stepLength,
                     float givenLiftDuration);

  public:
    BSplineGait(double stepHeight,
                double stepLength,
                double smoothing,
                double givenGroundHeight,
                double givenSpread,
                double givenOffsetFront,
                double givenOffsetLeft,
                double rearLegOffset,
                double givenLiftDuration);

    vec3P getGaitWagPoint(double givenTime, bool walkingForwards);
    std::vector<vec3P> getPosition(double givenTime, bool walkingForwards);

    float getStepLength() { return stepLength;}
    float getWagPhase() { return wagPhase; };
    float getWagAmplitude_x(){ return wagAmplitude_x; };
    float getWagAmplitude_y(){ return wagAmplitude_y; };

    void enableWag(float givenWagPhase, float givenWagAmplitude_x, float givenWagAmplitude_y) {
        wagPhase       = givenWagPhase;
        wagAmplitude_x = givenWagAmplitude_x;
        wagAmplitude_y = givenWagAmplitude_y;
    }
};
