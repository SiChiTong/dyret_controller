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
  // Calculate groundHeight:
  groundHeight = -430 - ((legActuatorLengths[0][0] + legActuatorLengths[0][1]) * 0.8f);
  for (int i = 0; i < 4; i++) goalPose[i].points[2] = groundHeight; // Update goalPose continuously

  printf("groundHeight: %.2f (l0: %.2f, l1: %.2f)\n",groundHeight, legActuatorLengths[0][0], legActuatorLengths[0][1]);

  if (currentPoseStates[0] == FINISHED &&
      currentPoseStates[1] == FINISHED &&
      currentPoseStates[2] == FINISHED &&
      currentPoseStates[3] == FINISHED){

      moveAllLegsToGlobalPosition(goalPose, positionCommand_pub);

      reachedPose = true;
      return true;


  } else if (currentPoseStates[0] == INIT_STEPDOWN){ // Global state INIT_STEPDOWN
      // In this state, we make sure every leg is on the ground

      if (currentProgress == 0.0){ // First loop of INIT_STEPDOWN

          printf("INIT_STEPDOWN\n");

          // Set goal positions:
          positionArray = currentLegPositions(*servoAnglesInRad, *legActuatorLengths);;

      } else { // Actually doing the stepping down

          std::vector<vec3P> tmpPosition = positionArray;
          for (int i = 0; i < 4; i++) tmpPosition[i].points[2] = groundHeight;

          if (interpolatingLegMoveOpenLoop(tmpPosition, positionArray, currentProgress, positionCommand_pub) == true){

              positionArray = tmpPosition; // We have now reached the new position

              for (int i = 0; i < 4; i++){
                  currentPoseStates[i] = LEAN_GENERATE;
              }
              printf("L* -> Lean Generate\n");
              sleep(stateTransitionDelay);
          }
      }

      currentProgress += stepDownSpeed;

  // Non-global states:
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
            case LEAN_GENERATE: // Calculate where to lean based on which leg we are going to move
              {
                std::vector<vec3P> currentPositions(4);
                double leanAmount = 35.0;

                if (legId == 0) currentLean = { -leanAmount,  leanAmount}; // Lener bak høyre
                if (legId == 1) currentLean = {  leanAmount,  leanAmount}; // Lener bak venstre
                if (legId == 2) currentLean = {  leanAmount, -leanAmount}; // Lener foran høyre
                if (legId == 3) currentLean = { -leanAmount, -leanAmount}; // Lener foran venstre

                currentPoseStates[legId] = LEAN_INTERPOLATE;
                printf("L%u: -> leanInterpolate\n", legId);
                currentProgress = 0.0;
                sleep(stateTransitionDelay);

                break;
              }
            case LEAN_INTERPOLATE: // Move all four legs to lean in the generated direction
              {
               currentProgress += leanSpeed;

               if (interpolatingLegMoveOpenLoop(add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0)),
                                                positionArray,
                                                currentProgress,
                                                positionCommand_pub) == true){
                   currentPoseStates[legId] = LIFT;
                   printf("L%u: -> lift\n", legId);
                   currentProgress = 0.0;
                   sleep(stateTransitionDelay);
               }


                break;
              }
            case LIFT: // Lift the one leg that is now being moved to the new correct position
              {
                currentProgress += liftSpeed;

                std::vector<vec3P> tmpStartPosition = add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0));
                std::vector<vec3P> tmpEndPosition = tmpStartPosition;
                tmpEndPosition[legId].points[2] = goalPose[legId].points[2] + stepHeight;

                if (interpolatingLegMoveOpenLoop(tmpEndPosition,
                                                 tmpStartPosition,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                    currentPoseStates[legId] = LINE_INTERPOLATE;
                    currentProgress = 0.0;
                    printf("L%u: -> LineInterpolate\n", legId);
                    sleep(stateTransitionDelay);

                }

                break;

              }
            case LINE_INTERPOLATE: // Move the one lifted leg to the correct position (still lifted)
              {
                std::vector<vec3P> tmpStartPosition = add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0));
                tmpStartPosition[legId].points[2] = goalPose[legId].points[2] + stepHeight;

                std::vector<vec3P> tmpEndPosition = tmpStartPosition;
                tmpEndPosition[legId] = add(goalPose[legId], vec3P(currentLean[0], currentLean[1], stepHeight)); // Incorporate lean into new position

                if (interpolatingLegMoveOpenLoop(tmpEndPosition,
                                                 tmpStartPosition,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                    currentPoseStates[legId] = STEPDOWN;
                    positionArray[legId] = goalPose[legId]; // This leg is now in the correct position
                    printf("L%u: -> stepDown\n", legId);
                    currentProgress = 0.0;
                    sleep(stateTransitionDelay);
                }

                currentProgress += legMoveSpeed;

                break;
              }
            case STEPDOWN: // Step down with the leg that just moved to the correct position
              {

                std::vector<vec3P> tmpEndPosition = add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0));

                std::vector<vec3P> tmpStartPosition = tmpEndPosition;
                tmpStartPosition[legId].points[2] = tmpStartPosition[legId].points[2] + stepHeight;


                currentProgress += stepDownSpeed;

                if (interpolatingLegMoveOpenLoop(tmpEndPosition,
                                                 tmpStartPosition,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
                  currentProgress = 0.0;
                  currentPoseStates[legId] = LEAN_BACK;
                  positionArray[legId] = goalPose[legId];
                  printf("L%u: -> LEAN_BACK\n",legId);
                  sleep(stateTransitionDelay);
                }

                break;
            }
            case LEAN_BACK: // Lean back to the center and repeat for the rest of the legs
            {
                currentProgress += leanSpeed;

                std::vector<vec3P> tmpStartPosition = add(positionArray, vec3P(currentLean[0], currentLean[1], 0.0));

                if (interpolatingLegMoveOpenLoop(positionArray,
                                                 tmpStartPosition,
                                                 currentProgress,
                                                 positionCommand_pub) == true){
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
