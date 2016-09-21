#include <math.h>

#include "IncPoseAdjuster.h"

#include "kinematicFunctions.h"
#include "movementFunctions.h"

bool IncPoseAdjuster::Spin(){
  if (currentPoseStates[0] == FINISHED &&
      currentPoseStates[1] == FINISHED &&
      currentPoseStates[2] == FINISHED &&
      currentPoseStates[3] == FINISHED){

      moveAllLegsToGlobal(goalPose, inverseKinematicsService_client, dynCommands_pub);

      reachedPose = true;
      return true;

  } else if (currentPoseStates[0] == INIT_STEPDOWN){ // Global

      if (startPositions.size() == 0){

          printf("INIT_STEPDOWN\n");

          // Startpositions has not been initialized:
          startPositions = currentLegPositions(*servoAnglesInRad);

          groundHeight = fmin(fmin(goalPose[0].points[2], goalPose[1].points[2]),fmin(goalPose[2].points[2], goalPose[3].points[2]));

          // Set goal positions:
          positionArray = startPositions;
          for (int i = 0; i < 4; i++) positionArray[i].points[2] = groundHeight;

          currentProgress = 0.0;

      } else {
          currentProgress += stepDownSpeed;

          if (interpolatingLegMoveOpenLoop(positionArray, startPositions, currentProgress, inverseKinematicsService_client, dynCommands_pub) == true){
              startPositions.clear();
              for (int i = 0; i < 4; i++){
                  currentPoseStates[i] = LEAN_GENERATE;
              }
              printf("L* -> Lean Generate\n");
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
                vec3P leanVector;
                if (legId == 0) currentLean = { -leanAmount,  leanAmount}; // Lener bak høyre
                if (legId == 1) currentLean = {  leanAmount,  leanAmount}; // Lener bak venstre
                if (legId == 2) currentLean = {  leanAmount, -leanAmount}; // Lener foran høyre
                if (legId == 3) currentLean = { -leanAmount, -leanAmount}; // Lener foran venstre

                currentPoseStates[legId] = LEAN_INTERPOLATE;
                printf("L%u: -> leanInterpolate\n", legId);

                break;
              }
            case LEAN_INTERPOLATE: // leanInterpolate
              {
                if (startPositions.size() == 0){
                  // Startpositions has not been initialized:
                  startPositions = currentLegPositions(*servoAnglesInRad);
                  currentProgress = 0.0;
                } else {
                 currentProgress += leanSpeed;

                 if (interpolatingLegMoveOpenLoop(add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0)),
                                                  positionArray,
                                                  currentProgress,
                                                  inverseKinematicsService_client, dynCommands_pub) == true){
                     currentPoseStates[legId] = LIFT;
                     printf("L%u: -> lift\n", legId);
                     currentProgress = 0.0;
                     startPositions.clear();
                 }
                }

                break;
              }
            case LIFT:
              {
                if (startPositions.size() == 0){
                  // Startpositions has not been initialized:
                    startPositions.resize(1);
                    startPositions[0] = currentLegPos(legId, *servoAnglesInRad);

                    tmpLegPoseVar = startPositions[0];
                    tmpLegPoseVar.points[2] = goalPose[legId].points[2] + stepHeight; // LegLiftHight
                    currentProgress = 0.0;
                } else{
                    currentProgress += liftSpeed;
                }

                if (interpolatingLegMoveOpenLoop(legId, tmpLegPoseVar, startPositions[0], currentProgress, *servoAnglesInRad, inverseKinematicsService_client, dynCommands_pub) == true){
                    startPositions.clear();
                    currentPoseStates[legId] = LINE_INTERPOLATE;
                    currentProgress = 0.0;
                    printf("L%u: -> LineInterpolate\n", legId);

                }

                break;

              }
            case LINE_INTERPOLATE: // lineInterpolate
              {
                vec3P raisedLegPose = add(goalPose[legId], vec3P(currentLean[0], currentLean[1], 0.0)); // Incorporate lean into new positions
                raisedLegPose.points[2] = raisedLegPose.points[2] + stepHeight; // Raise leg by stepHeight

                if (interpolatingLegMoveOpenLoop(legId, raisedLegPose, tmpLegPoseVar, currentProgress, *servoAnglesInRad, inverseKinematicsService_client, dynCommands_pub) == true){
                    currentPoseStates[legId] = STEPDOWN;
                    printf("L%u: -> stepDown\n", legId);
                }

                currentProgress += legMoveSpeed;

                break;
              }
            case STEPDOWN: // stepdown
              {

                if (startPositions.size() == 0){
                    // Startpositions has not been initialized:
                    startPositions.resize(1);
                    startPositions[0] = currentLegPos(legId, *servoAnglesInRad);
                    currentProgress = 0.0;
                  } else{
                    currentProgress += stepDownSpeed;
                  }

                  if (interpolatingLegMoveOpenLoop(legId, add(goalPose[legId], vec3P(currentLean[0], currentLean[1], 0.0)), startPositions[0], currentProgress, *servoAnglesInRad, inverseKinematicsService_client, dynCommands_pub) == true){
                    startPositions.clear();
                    currentPoseStates[legId] = LEAN_BACK;
                    positionArray[legId] = goalPose[legId];
                    printf("L%u: -> LEAN_BACK\n",legId);
                  }

                  break;
            }
            case LEAN_BACK:
            {
                if (startPositions.size() == 0){
                    // Startpositions has not been initialized:
                    startPositions = currentLegPositions(*servoAnglesInRad);
                    currentProgress = 0.0;
                } else {
                    currentProgress += leanSpeed;
                }

                if (interpolatingLegMoveOpenLoop(positionArray, startPositions, currentProgress, inverseKinematicsService_client, dynCommands_pub) == true){
                    startPositions.clear();
                    currentProgress = 0.0;
                    currentPoseStates[legId] = FINISHED;
                    printf("L%u: -> FINISHED\n", legId);
                }

                break;
              }
          }

          break; // Only one leg at a time

      }
  }

  return false;
}
