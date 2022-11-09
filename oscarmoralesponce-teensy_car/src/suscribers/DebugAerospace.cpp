#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "config.h"
#include "state.h"
#include "robot.h"
#include "uartCommand.h"
#include <arm_math.h>
#include "state.h"
#include "Aerospace.h"


#define TEST_PREPARE_TO_MOVE_STAND_ALONE 1
#define TEST_PREPARE_TO_MOVE_COOPERATIVE 2
#define TEST_FAILURE_RECOVERY 3
#define TEST_RELOCATE_STAND_ALONE 4
#define TEST_CHANGE_FACE_STAND_ALONE 5
#define TEST_WAIT_STAND_ALONE 6
#define TEST_WAIT_COOPERRATIVE 7
#define TEST_CHANGE_FACE_COOPERATIVE 8
#define TEST_WAIT_COOPERRATIVE_RECOVERY 9
#define TEST_APPROACHING 10



#define TEST_CASE TEST_WAIT_COOPERRATIVE_RECOVERY 


extern ExternalSensor externalSensor;
extern StateData stateData;
      
void debugPrintState(uint8_t state)
{
    switch (state)
    {
      case DEBUG:
	  Serial.print("DEBUG ");
	break;
      case INIT:
	  Serial.print("INIT ");
	break;
      case END:
	  Serial.print("END ");
	break;
      case BROADCAST:
	  Serial.print("BROADCAST ");
	break;
      case APPROACHING:
	  Serial.print("APPROACHING ");
	break;
      case PREPARE_TO_MOVE:
	  Serial.print("PREPARE_TO_MOVE ");
	break;
      case CHANGE_FACE:
	  Serial.print("CHANGE_FACE ");
	break;
      case SCANNING:
	  Serial.print("SCANNING ");
	break;
      case PUSH_BOX:
	  Serial.print("PUSH_BOX ");
	break;
	 case WAIT:
	  Serial.print("WAIT ");
	break;
	case WAIT_TEST:
	  Serial.print("WAIT_TEST ");
	break;
      case RELOCATE_ON_FACE:
	Serial.print("RELOCATE_ON_FACE");
	break;
      case HORIZONTAL_LEFT:
	  Serial.print("HORIZONTAL_LEFT ");
	break;
      case HORIZONTAL_RIGHT:
	  Serial.print("HORIZONTAL_RIGHT ");
	break;
      case HORIZONTAL_CORRECTION:
	  Serial.print("HORIZONTAL_CORRECTION ");
	break;
      case HORIZONTAL:
	  Serial.print("HORIZONTAL ");
	break;
      case VERTICAL:
	  Serial.print("VERTICAL ");
	break;
      case ROTATE_CAR:
	  Serial.print("ROTATE_CAR ");
	break;
      case CORRECTION_STATE:
	  Serial.print("CORRECTION_STATE ");
	break;
      case CENTER:
	  Serial.print("CENTER ");
	break;
      case WAITING:
	  Serial.print("WAITING ");
	break;
     case PUSHING:
	  Serial.print("PUSHING ");
	break;	
      case MOVE_LEFT_UNITL_FREE:
	  Serial.print("MOVE_LEFT_UNITL_FREE ");
	break;
      case MOVE_RIGHT_UNITL_FREE:
	  Serial.print("MOVE_RIGHT_UNITL_FREE ");
	break;
      case RESET_STATE:
	  Serial.print("RESET_STATE ");
	break;
      case ALIGN:
	  Serial.print("ALIGN ");
	break;
      case SCANNINGROTATE:
	  Serial.print("SCANNINGROTATE ");
	break;
      case STABILIZE:
	  Serial.print("STABILIZE ");
	  break;
      case INTERPUSH:
	  Serial.print("INTERPUSH ");
	  break;
      case TEST_PUSH_BOX:
	  Serial.print("TEST_PUSH_BOX ");
	  break;
     case OBSTACLE_AVOIDANCE:
	  Serial.print("OBSTACLE_AVOIDANCE");
	  break;
      default: 
	Serial.print("UNKNOWN ");
	Serial.print(state);
	break;
    } 
   
}
           
void debugPrint(uint8_t globalState, LDMap *ldmap)
{
   
    Serial.print("DD2,");
    Serial.print(ldmap->position_x);
    Serial.print(",");
    Serial.print(ldmap->position_y);
    Serial.print(",");
    Serial.print(ldmap->speed);
    Serial.print(",");
    Serial.print(ldmap->heading);
    Serial.print(",");
    Serial.print(stateData.boxLength);
    Serial.print(",");
    Serial.print(stateData.boxWidth);
    Serial.print(",");
    Serial.print(stateData.Xdistance);
    Serial.print(",");
    Serial.print(stateData.Ydistance);
    Serial.print(",");
    Serial.print(stateData.distanceToMove);
    Serial.print(",");
    Serial.print(stateData.currentFace);
    Serial.print(",");
    Serial.print(stateData.distanceBetweenRobots);
    Serial.print(",");
    Serial.println(stateData.previousRankFailures);
    Serial.print("DD3,");
    Serial.print(externalSensor.distance);
    Serial.print(",");
    Serial.print(externalSensor.offset);
    Serial.print(",");
    Serial.print(externalSensor.angle);
    Serial.print(",");
    Serial.print(externalSensor.dist_0);
    Serial.print(",");
    Serial.print(externalSensor.dist_1);
    Serial.print(",");
    Serial.print(externalSensor.dist_2);
    Serial.print(",");
    Serial.println(externalSensor.dist_3);
}


// prepareToMoveStandAlone: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance > 0
//           Ydistance in (-200, 200)
//           Ydistance < 0  If Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 0
//           rank = 1
//           active = 1
//   PostCondition:
// 		currentFace = 2
//		stateData.boxLength within +/-10cm error  
//		stateData.boxWidth within +/-10cm error 
//		stateData.boxLength / stateData.boxWidth = 1.22 +/- .1
//      robot is in center 
//      distanceBetweenRobots = (stateData.boxLength-30)/2 = 20
inline void prepareToMoveStandAlone(uint8_t *globalState)
{
    *globalState = PREPARE_TO_MOVE;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 1;  // DEBUG
    stateData.active = 1; // DEBUG
    stateData.Xdistance = 100;
    stateData.Ydistance = -90;
    stateData.currentFace = 0;
    stateData.faceTraverse = -3;
     stateData.localState = RESET_STATE;
}

// changeFaceStandAlone: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance > 0
//           Ydistance in (-200, 200) If Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 1
//           rank = 1
//           active = 1
//           stateData.boxLength within +/-10cm error  
//	 	     stateData.boxWidth within +/-10cm error 
//   PostCondition:
// 		currentFace = currentFace + 1 if Ydistance<0, otherwise currentFace -1
//      robot is in center 
//      distanceBetweenRobots = (stateData.boxLength-30)/2 = 20
inline void changeFaceStandAlone(uint8_t *globalState)
{
    *globalState = CHANGE_FACE;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 1;  // DEBUG
    stateData.active = 1; // DEBUG
    stateData.Xdistance = 50;
    stateData.Ydistance = -90;
    stateData.boxLength = 40;
    stateData.boxWidth = 35;
    stateData.currentFace = 2;
    stateData.faceTraverse = -2;
    stateData.isRobotInCorridor = true;
//     stateData.faceTraverse = -1;
}


// pushStandAlone: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance > 0
//           Ydistance in (-200, 200) If Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 2
//           rank = 1
//           active = 1
//           robot is in center 
//           stateData.boxLength within +/-10cm error  
//	 	     stateData.boxWidth within +/-10cm error 
inline void pushStandAlone(uint8_t *globalState)
{
    *globalState = WAIT_TEST;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 1;  // DEBUG
    stateData.active = 1; // DEBUG
    stateData.Xdistance = 100;
    stateData.Ydistance = -90;
    stateData.currentFace = 2;
    stateData.boxLength = 67;
    stateData.boxWidth = 58;
    stateData.distanceBetweenRobots = (stateData.boxLength - LENGTH) / 2;
}




// Appraoching: 
//   Preconditions:
//           Xdistance > 0
//           Ydistance > 0
//           rank = 1
//           active = 1
inline void approachingStandAlone(uint8_t *globalState)
{
    *globalState = APPROACHING;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 1;  // DEBUG
    stateData.active = 1; // DEBUG
    stateData.Xdistance = 100;
    stateData.Ydistance = -90;
}



// prepareToMoveCooperative: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance in (50, 200)
//           Ydistance in (-200, 200) if Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 0
//           rank = 2
//           active = 2 
//   PostCondition:
//           At the end of PREPARE_TO_MOVE
// 		currentFace = 2
//		stateData.boxLength is equal to dimension that robot with rank 1 obtained
//		stateData.boxWidth is equal to dimension that robot with rank 1 obtained
//		stateData.boxLength / stateData.boxWidth = 7/5
//              robot is in the left from the initial setting, robot with rank 1 is on the right
inline void prepareToMoveCooperative(uint8_t *globalState)
{
  
    switch (getId())
    {
      case 4:
	  *globalState = WAIT_TEST;   // DEBUG
	  stateData.localState = INIT;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 1;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance = 100;
	  stateData.Ydistance = -80;
	  stateData.boxLength = 72;
	  stateData.boxWidth = 60;
	  stateData.currentFace = 2;
	  stateData.isRobotInCorridor = false;
	  stateData.distanceBetweenRobots = (stateData.boxLength - LENGTH) / 2;

      
	break;
      case 5:
	  *globalState = PREPARE_TO_MOVE;   // DEBUG
	  stateData.localState = RESET_STATE;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 2;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance =  100;
	  stateData.Ydistance = -80;
	   stateData.boxLength = 72;
	  stateData.boxWidth = 60;
	  stateData.currentFace = 0;
	  stateData.faceTraverse = 3;
	break;
    }
    
}



// failureRelocate: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance in (50, 200)
//           Ydistance in (-200, 200) If Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 0
//           rank = 1
//           active = 1
//   PostCondition:
//           At the end of PREPARE_TO_MOVE
// 		currentFace = 2
//		stateData.boxLength in (60, 80)  // measures with the sensors
//		stateData.boxWidth in (50, 70)   // measures with the sensors
//		stateData.boxLength / stateData.boxWidth = 7/5
//              robot is in center = 24
inline void failureRelocate(uint8_t *globalState)
{
    *globalState = WAIT_TEST;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 2;  // DEBUG
    stateData.active = 2; // DEBUG
    stateData.Xdistance = 100;
    stateData.Ydistance = 90;
    stateData.currentFace = 2;
    stateData.boxLength = 70; 
    stateData.boxWidth = 55; 
    stateData.distanceBetweenRobots = 24; 
}


inline void failureRelocateInWait(uint8_t *globalState)
{
    *globalState = WAIT;   // DEBUG
    stateData.localState = INIT;
    stateData.stateSecondLevel = INIT;
    stateData.rank = 2;  // DEBUG
    stateData.active = 2; // DEBUG
    stateData.Xdistance = 100;
    stateData.Ydistance = 90;
    stateData.currentFace = 2;
    stateData.boxLength = 50; 
    stateData.boxWidth = 45; 
    stateData.distanceBetweenRobots = 24; 
}




// prepareToMoveCooperative: 
//   Preconditions:
//           stateData.filter_dist_0 <= THRESHOLD
//           stateData.filter_dist_1 <= THRESHOLD
//           Xdistance in (50, 200)
//           Ydistance in (-200, 200) if Ydistance < 0, then LEFT otherwise RIGHT 
//           currentFace = 0
//           rank = 2
//           active = 2 
//   PostCondition:
//           At the end of PREPARE_TO_MOVE
// 		currentFace = 2
//		stateData.boxLength is equal to dimension that robot with rank 1 obtained
//		stateData.boxWidth is equal to dimension that robot with rank 1 obtained
//		stateData.boxLength / stateData.boxWidth = 7/5
//              robot is in the left from the initial setting, robot with rank 1 is on the right
inline void waitCooperative(uint8_t *globalState)
{
  
    switch (getId())
    {
      case 4:
	  *globalState = WAIT_TEST;   // DEBUG
	  stateData.localState = INIT;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 1;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance = 100;
	  stateData.Ydistance = 40;
	  stateData.currentFace =1;
	  stateData.boxLength = 72;
	  stateData.boxWidth = 57;
	  stateData.isRobotInCorridor = false;
// 	  stateData.faceTraverse = 2;
 	  stateData.distanceBetweenRobots = 20;
//       
	break;
      case 5:
	  *globalState = WAIT_TEST;   // DEBUG
	  stateData.localState = INIT;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 2;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance = 100;
	  stateData.Ydistance = 40;
	  stateData.currentFace = 1;
	  stateData.boxLength = 72;
	  stateData.boxWidth = 57;
	  stateData.isRobotInCorridor = false;
// 	  stateData.faceTraverse = 2;
 	  stateData.distanceBetweenRobots = 20;
	break;
    }    
}


inline void waitCooperativeRecovery(uint8_t *globalState)
{
  
    switch (getId())
    {
      case 4:
	  *globalState = WAIT;   // DEBUG
	  stateData.localState = INIT;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 2;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance = 50;
	  stateData.Ydistance = -50;
	  stateData.currentFace = 1;
	  stateData.boxLength = 72;
	  stateData.boxWidth = 62;
	  stateData.isRobotInCorridor = true;
// 	  stateData.faceTraverse = 2;
 	  stateData.distanceBetweenRobots = 20;
//       
	break;
      case 5:
	  *globalState = WAIT;   // DEBUG
	  stateData.localState = INIT;
	  stateData.stateSecondLevel = INIT;
	  stateData.rank = 1;  // DEBUG
	  stateData.active = 2; // DEBUG
	  stateData.Xdistance = 50;
	  stateData.Ydistance = -50;
	  stateData.currentFace = 1;
	  stateData.boxLength = 72;
	  stateData.boxWidth = 62;
	  stateData.isRobotInCorridor = true;
// 	  stateData.faceTraverse = 2;
 	  stateData.distanceBetweenRobots = 20;
	break;
    }
    
}




void debugMode(uint8_t *globalState)
{
      memset(&stateData, sizeof(stateData), 0);

#if TEST_CASE == TEST_APPROACHING
      Serial.println("TEST_APPROACHING");
      approachingStandAlone(globalState);
      
#elif TEST_CASE == TEST_PREPARE_TO_MOVE_STAND_ALONE
      Serial.println("TEST_PREPARE_TO_MOVE_STAND_ALONE");
      prepareToMoveStandAlone(globalState);
#elif  TEST_CASE == TEST_PREPARE_TO_MOVE_COOPERATIVE
      Serial.println("TEST_PREPARE_TO_MOVE_COOPERATIVE");
      prepareToMoveCooperative(globalState);
#elif TEST_CASE == TEST_FAILURE_RECOVERY
      Serial.println("TEST_FAILURE_RECOVERY");
      failureRelocate(globalState);
#elif TEST_CASE == TEST_CHANGE_FACE_STAND_ALONE
      Serial.println("TEST_CHANGE_FACE_STAND_ALONE");
      changeFaceStandAlone(globalState);
#elif TEST_CASE == TEST_WAIT_STAND_ALONE
      Serial.println("TEST_WAIT_STAND_ALONE");
      pushStandAlone(globalState);
#elif TEST_CASE == TEST_WAIT_COOPERRATIVE
      Serial.println("Test wait coopertive");
      waitCooperative(globalState);
#elif TEST_CASE == TEST_WAIT_COOPERRATIVE_RECOVERY
      Serial.println("Test wait coopertive");
      waitCooperativeRecovery(globalState);

      
#elif TEST_CASE == TEST_CHANGE_FACE_COOPERATIVE
      Serial.println("TEST_CHANGE_FACE_COOPERATIVE");
      changeFaceCooperative(globalState);
      
#endif
}

      
