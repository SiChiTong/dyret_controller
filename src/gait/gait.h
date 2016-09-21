#pragma once

#include <vector>

#include "../kinematics/kinematicTypes.h"
#include "GaitSegment.h"

class Gait {

private:
	void markGroundSegments();

protected:

	std::vector<vec3P> gaitPointVector;
	std::vector<GaitSegment> gaitSegments;

	float legPhaseOffset[4][2];

	float spreadAmount  = 0.0;
	float offsetFront   = 0.0;
	float offsetLeft    = 0.0;
	float rearLegOffset = 0.0;

	float wagPhase = 0.0;
	float wagAmplitude_x = 0.0;
	float wagAmplitude_y = 0.0;

	float groundHeight = 0.0;

	bool reverse = false;

	float stepLength;

	int numberOfPoints;

	double gaitSpeedMultiplier = 1.0;

	double increment = 0.01; // Increment for speed calculation
	double totalTime = 0.0;
	
	double totalLength = 0.0;
	double groundLength = 0.0;

	void calculateMaxSpeedFactor();
	
public:

	float getWagPhase() { return wagPhase; };
	float getWagAmplitude_x(){ return wagAmplitude_x; };
	float getWagAmplitude_y(){ return wagAmplitude_y; };

	void runNormal() { reverse = false; }
	void runReversed() { reverse = true; };

	Gait() {

	}

	void enableWag(float givenWagPhase, float givenWagAmplitude_x, float givenWagAmplitude_y);

	void init(std::vector<vec3P> givenPoints){
	  gaitPointVector = givenPoints;
    numberOfPoints = givenPoints.size();

    // Add all segments:
    gaitSegments.resize(gaitPointVector.size());

    for (int i = 0; i < numberOfPoints; i++) {

      if (gaitPointVector[i].points[2] < groundHeight) groundHeight = gaitPointVector[i].points[2];

      gaitSegments[i].start = gaitPointVector[i];
      if (i == (numberOfPoints-1)) {
        gaitSegments[i].end = gaitPointVector[0];
      } else {
        gaitSegments[i].end = gaitPointVector[i + 1];
      }
    }

    markGroundSegments();
	}

	void setSpread(float givenSpreadAmount){
	  spreadAmount = givenSpreadAmount;
	}

	void setOffsets(float givenOffsetFront, float givenOffsetLeft, float givenRearLegOffset){
	  offsetFront = givenOffsetFront;
	  offsetLeft = givenOffsetLeft;
	  rearLegOffset = givenRearLegOffset;
	}

	Gait(std::vector<vec3P> givenPoints) {
		init(givenPoints);
	}

	float getStepLength(){ return stepLength; };

	double getGaitSpeedMultiplier() { return gaitSpeedMultiplier; };
	double getTotalTime() {return totalTime;}
	virtual std::vector<vec3P> getPosition(double givenTime, bool walkingForwards) = 0;
};
