#pragma once

#include <vector>

#include "../kinematics/kinematicTypes.h"
#include "gait.h"
#include "../external/splineLibrary/spline.h"
#include "../external/splineLibrary/basis/looping_uniform_cubic_bspline.h"
#include "../external/splineLibrary/hermite/cubic/looping_uniform_cr_spline.h"
#include "../external/splineLibrary/hermite/cubic/looping_cubic_hermite_spline.h"

class BSplineGait : public Gait {

private:
	//LoopingUniformCubicBSpline<Vector3>* bSpline;
  //LoopingUniformCRSpline<Vector3>* bSpline;
  LoopingCubicHermiteSpline<Vector3>* bSpline;

	float maxT;

	float gndStart, gndEnd, gndFactor, maxTime, gndContactPercent;

	std::vector<float> groundContactLine; // Contains T values of ground contact line
	std::vector<float> lengths;		        // Used in getPosition to go from time to T value

	std::vector<vec3P> createBSplineGaitPoints(double stepHeight, double stepLength, double smoothing, double groundHeight);

	double applyGndTimeScaling(double givenTime);
	void bSplineInit(std::vector<vec3P> givenPoints, float stepLength);

public:
	BSplineGait(double stepHeight, double stepLength, double smoothing, double groundHeight, double givenSpread, double givenOffsetFront, double givenOffsetLeft, double rearLegOffset);

	vec3P getGaitWagPoint(double givenTime);
	std::vector<vec3P> getPosition(double givenTime, bool walkingForwards);
	float getGndContactPercent(){ return gndContactPercent; };
};
