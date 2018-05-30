#include <math.h>

#include "IncPoseAdjuster.h"

#include "kinematicFunctions.h"
#include "movementFunctions.h"

void printPosition(std::vector<vec3P> givenPosition, std::string givenDescription){
  fprintf(stderr, "%s:\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n\t%.2f, %.2f, %.2f\n",
          givenDescription.c_str(),
          givenPosition[0].x(), givenPosition[0].y(), givenPosition[0].z(),
          givenPosition[1].x(), givenPosition[1].y(), givenPosition[1].z(),
          givenPosition[2].x(), givenPosition[2].y(), givenPosition[2].z(),
          givenPosition[3].x(), givenPosition[3].y(), givenPosition[3].z());

}

bool IncPoseAdjuster::Spin(){
  if (currentPoseStates[0] == FINISHED &&
      currentPoseStates[1] == FINISHED &&
      currentPoseStates[2] == FINISHED &&
      currentPoseStates[3] == FINISHED){

      moveAllLegsToGlobalPosition(goalPose, positionCommand_pub);

      reachedPose = true;
      return true;

  } else if (currentPoseStates[0] == INIT_STEPDOWN){
      // In this state, we make sure every leg is on the ground

      if (startPositions.size() == 0){ // First loop of INIT_STEPDOWN

          printf("INIT_STEPDOWN\n");

          // Initialize startPositions with current leg positions:
          startPositions = currentLegPositions(*servoAnglesInRad, legActuatorLengths);

          //printPosition(startPositions, "StartPosition");

          // groundHeight is set as the minimum
          groundHeight = (float) fmin(fmin(goalPose[0].z(), goalPose[1].z()),fmin(goalPose[2].z(), goalPose[3].z()));

          // Set goal positions:
          positionArray = startPositions;
          for (int i = 0; i < 4; i++) positionArray[i].points[2] = groundHeight;

          //printPosition(positionArray, "positionArray");

          currentProgress = 0.0;

      } else {

          currentProgress += stepDownSpeed;

          if (interpolatingLegMoveOpenLoop(positionArray, startPositions, currentProgress, positionCommand_pub) == true){

              startPositions.clear();
              for (int i = 0; i < 4; i++){
                  currentPoseStates[i] = LEAN_GENERATE;
              }
              printf("L* -> Lean Generate\n");
              sleep(stateTransitionDelay);
          }
      }

  } else {

      for (int i = 0; i < 4; i++){ // For all legs
          // For each leg:
          int legId;

          switch (i){
            case 0:
              legId = 1;
              break;
            case 1:
              legId = 0;
              break;
            case 2:
              legId = 3;
              break;
            case 3:
              legId = 2;
              break;
          }

          if (currentPoseStates[legId] == FINISHED) continue; // Done -> next leg

          switch(currentPoseStates[legId]){
            case LEAN_GENERATE:
              {
                std::vector<vec3P> currentPositions(4);
                double leanAmount = 35.0;

                if (legId == 0) currentLean = { -leanAmount,  leanAmount}; // Lener bak høyre
                if (legId == 1) currentLean = {  leanAmount,  leanAmount}; // Lener bak venstre
                if (legId == 2) currentLean = {  leanAmount, -leanAmount}; // Lener foran høyre
                if (legId == 3) currentLean = { -leanAmount, -leanAmount}; // Lener foran venstre

                currentPoseStates[legId] = LEAN_INTERPOLATE;
                printf("L%u: -> leanInterpolate\n", legId);
                sleep(stateTransitionDelay);

                break;
              }
            case LEAN_INTERPOLATE: // leanInterpolate
              {
                if (startPositions.size() == 0){
                  // Startpositions has not been initialized:
                  startPositions = currentLegPositions(*servoAnglesInRad, legActuatorLengths);
                  printPosition(startPositions, "startPositions");
                  printPosition(add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0)), "leanGoal");
                  currentProgress = 0.0;
                } else {
                 currentProgress += leanSpeed;

                 if (interpolatingLegMoveOpenLoop(add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0)),
                                                  positionArray,
                                                  currentProgress,
                                                  positionCommand_pub) == true){
                     currentPoseStates[legId] = LIFT;
                     printf("L%u: -> lift\n", legId);
                     currentProgress = 0.0;
                     startPositions.clear();
                     sleep(stateTransitionDelay);
                 }
                }

                break;
              }
            case LIFT:
              {
                if (startPositions.size() == 0){
                  // Startpositions has not been initialized:
                    startPositions.resize(1);
                    startPositions[0] = currentLegPos(legId, *servoAnglesInRad, legActuatorLengths);

                    tmpLegPoseVar = startPositions[0];
                    tmpLegPoseVar.points[2] = goalPose[legId].points[2] + stepHeight; // LegLiftHight
                    currentProgress = 0.0;
                } else{
                    currentProgress += liftSpeed;
                }

                if (interpolatingLegMoveOpenLoop(legId,
                                                 tmpLegPoseVar,
                                                 startPositions[0],
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                    startPositions.clear();
                    currentPoseStates[legId] = LINE_INTERPOLATE;
                    currentProgress = 0.0;
                    printf("L%u: -> LineInterpolate\n", legId);
                    sleep(stateTransitionDelay);

                }

                break;

              }
            case LINE_INTERPOLATE: // lineInterpolate
              {
                vec3P raisedLegPose = add(goalPose[legId], vec3P(currentLean[0], currentLean[1], 0.0)); // Incorporate lean into new positions
                raisedLegPose.points[2] = raisedLegPose.points[2] + stepHeight; // Raise leg by stepHeight

                if (interpolatingLegMoveOpenLoop(legId,
                                                 raisedLegPose,
                                                 tmpLegPoseVar,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                    currentPoseStates[legId] = STEPDOWN;
                    printf("L%u: -> stepDown\n", legId);
                    sleep(stateTransitionDelay);
                }

                currentProgress += legMoveSpeed;

                break;
              }
            case STEPDOWN: // stepdown
              {

                if (startPositions.size() == 0){
                    // Startpositions has not been initialized:
                    startPositions.resize(1);
                    startPositions[0] = currentLegPos(legId, *servoAnglesInRad, legActuatorLengths);
                    currentProgress = 0.0;
                  } else{
                    currentProgress += stepDownSpeed;
                  }

                  if (interpolatingLegMoveOpenLoop(legId,
                                                   add(goalPose[legId], vec3P(currentLean[0], currentLean[1], 0.0)),
                                                   startPositions[0],
                                                   currentProgress,
                                                   positionCommand_pub) == true){
                    startPositions.clear();
                    currentPoseStates[legId] = LEAN_BACK;
                    positionArray[legId] = goalPose[legId];
                    printf("L%u: -> LEAN_BACK\n",legId);
                    sleep(stateTransitionDelay);
                  }

                  break;
            }
            case LEAN_BACK:
            {
                if (startPositions.size() == 0){
                    // Startpositions has not been initialized:
                    startPositions = currentLegPositions(*servoAnglesInRad, legActuatorLengths);
                    currentProgress = 0.0;
                } else {
                    currentProgress += leanSpeed;
                }

                if (interpolatingLegMoveOpenLoop(positionArray,
                                                 startPositions,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                    startPositions.clear();
                    currentProgress = 0.0;
                    currentPoseStates[legId] = FINISHED;
                    printf("L%u: -> FINISHED\n", legId);
                    sleep(stateTransitionDelay);
                }

                break;
              }
          }

          break; // Only one leg at a time

      }
  }

  return false;
}
