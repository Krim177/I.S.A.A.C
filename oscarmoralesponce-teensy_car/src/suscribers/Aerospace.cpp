#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "config.h"
#include "state.h"
#include "robot.h"
#include "uartCommand.h"
#include <arm_math.h>
#include "Aerospace.h"

#if APPLICATION  == AEROSPACE	



static uint8_t positionInTheBox = OVERPASS_THE_BOX;
static uint8_t numberOfTries = 0;
static arm_pid_instance_f32 pidAligning;
static bool newSensorReading = false;

static SemaphoreHandle_t mutex;

ExternalSensor externalSensor;
StateData stateData;
      

/*********************************************************************************************
*  OnExternalSensorOrtho receives the reading from the external computer, i.e., Raspberry PI
* Params: 
*     externalSensorReading: is the reading that the external comptuer sends
* 	globalState: The state of the main state machine
*******************************************************************************************/
uint8_t OnExternalSensorOrtho(ExternalSensor *externalSensorReading, LDMap *ldmap, uint8_t *globalState)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    externalSensor.dist_0 = externalSensorReading->dist_0;
    externalSensor.dist_1 = externalSensorReading->dist_1;
    externalSensor.dist_2 = externalSensorReading->dist_2;
    externalSensor.dist_3 = externalSensorReading->dist_3;  
//     
    switch (*globalState)
    {
      case SCANNING:
          if (externalSensorReading->code != 0)
          {
              if (externalSensorReading->distance > 0)
              {
                  externalSensor.distance = externalSensorReading->distance;
                  externalSensor.angle= externalSensorReading->angle;
                  externalSensor.offset= externalSensorReading->offset / 10;
		  if (stateData.angle > 0 && externalSensor.angle > 0)
		    stateData.angle = min(stateData.angle, externalSensor.angle);
		  else if (stateData.angle < 0 && externalSensor.angle < 0)
		    stateData.angle = max(stateData.angle, externalSensor.angle);
		  else
		    stateData.angle = (externalSensor.angle + stateData.angle) / 2;
		  
                  stateData.Ydistance = externalSensorReading->offset;
                  stateData.Xdistance = abs(externalSensor.distance);
		  newSensorReading = true;
              }
              else 
		newSensorReading = false;
              externalSensor.code = externalSensorReading->code;
          }
          break;
      
      case INIT :
	      externalSensor.code = max(externalSensorReading->code, externalSensor.code);
	      break;
     case PREPARE_TO_MOVE:
	    newSensorReading = true;
	    switch (stateData.localState)
	    {
		case MOVE_LEFT_UNITL_FREE:
		case MOVE_RIGHT_UNITL_FREE:
 			switch (stateData.stateSecondLevel)
 			{
 			    case HORIZONTAL:
				if (areBothSensorsOutOfBounds() && stateData.rank == 1) {
				      if (stateData.currentFace == 1)
 					stateData.boxWidth = abs(ldmap->position_y) + LENGTH /2;
				      if (stateData.currentFace == 2)
 					stateData.boxLength= abs(ldmap->position_y) + LENGTH /2;
 				}
 				break;
 			  default:
				break;
 			}
		break;
	    default:
		    break;
	    }
	    break;
        case OBSTACLE_AVOIDANCE:
        case CHANGE_FACE:
	       newSensorReading = true;
      case BROADCAST:
          break;
      case APPROACHING:
          if (externalSensorReading->distance > 0)
          {
              newSensorReading = true;
              externalSensor.offset= externalSensorReading->offset/10;
              externalSensor.angle= externalSensorReading->angle;
          }   
          else 
	    newSensorReading = false;
	      break;
      default:
	  newSensorReading = true;
	  break;
    }		
    xSemaphoreGive(mutex);
    return 0;
}


void reset(LDMap *ldmap)
{
    ekf_reset(); 
    ldmap->heading = 0;
    ldmap->position_x = 0;
    ldmap->position_y = 0;
    arm_pid_reset_f32(&pidAligning);
	    
}

/*********************************************************************************************
*  Callback: stopWhenBothSensorsAreInBounds
*          true if the car is at distance at most UPPER_BOUND_DISTANCE
*******************************************************************************************/ 
static bool stopWhenBothSensorsAreInBounds()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint16_t distance0 = externalSensor.dist_0;
    uint16_t distance1 = externalSensor.dist_1;     
    xSemaphoreGive(mutex);

    if ((distance0 <= DISTANCE_THERSHOLD) && (distance1 <= DISTANCE_THERSHOLD) &&  abs(distance0 - distance1) <= MAX_DIFFERENCE)	
    {
        return true;
    }
    return false;
}


/*********************************************************************************************
*  Callback: stopWhenAtLeastOneSensorsIsInBound
*          true if the car is at distance at most UPPER_BOUND_DISTANCE
*******************************************************************************************/ 
static bool stopWhenAtLeastOneSensorsIsInBound()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint16_t distance0 = externalSensor.dist_0;
    uint16_t distance1 = externalSensor.dist_1;     
    xSemaphoreGive(mutex);

    if ((distance0 <= DISTANCE_THERSHOLD) || (distance1 <= DISTANCE_THERSHOLD))	
    {
        return true;
    }
    return false;
}

// /*********************************************************************************************
// *  Callback: stopWhenAtLeastOneSensorsIsInBound
// *          true if the car is at distance at most UPPER_BOUND_DISTANCE
// *******************************************************************************************/ 
// static bool stopWhenAtLeastOneSensorsIsOutOfBound()
// {
//     xSemaphoreTake(mutex, portMAX_DELAY);
//     uint16_t distance0 = externalSensor.dist_0;
//     uint16_t distance1 = externalSensor.dist_1;     
//     xSemaphoreGive(mutex);
// 
//     if ((distance0 > DISTANCE_THERSHOLD) || (distance1 > DISTANCE_THERSHOLD))	
//     {
//         return true;
//     }
//     return false;
// }



/*********************************************************************************************
*  Callback: stopWhenBothSensorsAreInBounds
*          true if the car is at distance at most dist
*******************************************************************************************/ 
static bool stopWhenBothSensorsAreOutOfBounds()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
     uint16_t distance0 = externalSensor.dist_0;
    uint16_t distance1 = externalSensor.dist_1;     
    xSemaphoreGive(mutex);

    if ((distance0 > DISTANCE_THERSHOLD) && (distance1 > DISTANCE_THERSHOLD))	
    {
        return true;
    }
    return false;
}


/*********************************************************************************************
*  Callback: stopWhenBothSensorsAreInBounds
*          true if the car is at distance at most dist
*******************************************************************************************/ 
static bool stopWhenLeftSensorisInBound()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint16_t distance3 = externalSensor.dist_3;
    xSemaphoreGive(mutex);

    if (distance3 < stateData.distanceBetweenRobots)	
    {
        return true;
    }
    return false;
}



/*********************************************************************************************
*  Callback: stopWhenBothSensorsAreInBounds
*          true if the car is at distance at most dist
*******************************************************************************************/ 
static bool stopWhenRightSensorisInBound()
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint16_t distance2 = externalSensor.dist_2;
    xSemaphoreGive(mutex);

    if (distance2 < stateData.distanceBetweenRobots)	
    {
        return true;
    }
    return false;
}

void onAlignLeft(Quaternion *p, Quaternion *q)
{
     float orientation = 0;
     if (newSensorReading && externalSensor.dist_1 < DISTANCE_THERSHOLD && abs(externalSensor.dist_1 - stateData.initialDistance) < 30)
     {
    	orientation = arm_pid_f32(&pidAligning, stateData.initialDistance - externalSensor.dist_1);
     
//  	Serial.print(stateData.initialDistance - externalSensor.dist_1);
//    	Serial.print(" PID ");
//    	Serial.println(orientation);
        Quaternion r;
        initQuaternion(&r, orientation, 0.0f, 0.0f);
        diff(p, &r, q);
	newSensorReading = false;
	
     }
     else
    {
//        Serial.println("Left no Aligning ");
       copyQuaternion(p, q);
    }
}

void onAlignRight(Quaternion *p, Quaternion *q)
{
       float orientation = 0;
//       copyQuaternion(p, q);
//        Serial.print(externalSensor.dist_0);
//        Serial.print(" ****ALIGN*** ");
//        Serial.println(stateData.initialDistance);
      if (newSensorReading && externalSensor.dist_0 < DISTANCE_THERSHOLD && abs(externalSensor.dist_1 - stateData.initialDistance) < 50)
     {
 	orientation = arm_pid_f32(&pidAligning, externalSensor.dist_0 - stateData.initialDistance);
     
// 	Serial.print(externalSensor.dist_0 - stateData.initialDistance );
//  	Serial.print(" RPID ");
//  	Serial.println(orientation);
        Quaternion r;
        initQuaternion(&r, -2, 0.0f, 0.0f);
        diff(p, &r, q);
	newSensorReading = false;
     }
     else
     {
//        Serial.println("Right no Aligning ");
       copyQuaternion(p, q);
    }
  
}

void moveAligned(CarCommand *command, LDMap *ldmap, uint8_t dir)
{
    if (dir == HORIZONTAL_LEFT)
    {
// 	command->adaptingOrientation = onAlignLeft;
	moveLeft(command, false, 10);
    }
    else
    {
// 	command->adaptingOrientation = onAlignRight;
	moveRight(command, false, 10);
    }
}


inline bool isAligned(int16_t minDistance, int16_t maxDistance)
{
   int16_t diffDist = externalSensor.dist_1- externalSensor.dist_0;
//    bool robotLessThanMinDistance = externalSensor.dist_0 <= minDistance && externalSensor.dist_1 <= minDistance;
   bool robotGreaterThanMaxDistance = externalSensor.dist_0 >= maxDistance && externalSensor.dist_1 >= maxDistance;
   
   return abs(diffDist) < ALIGN_THRESHOLD && !robotGreaterThanMaxDistance;
}

/*********************************************************************************************
*  align is the sate machine that approaches the box
* Prerequisite:
*      externalSensor.code > 0 || externalSensor.code > 1
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      externalSensor.dist_1 <= DISTANCE_THERSHOLD || externalSensor.dist_0 <= DISTANCE_THERSHOLD
*     externalSensor.dist_2 <= DISTANCE_THERSHOLD
* ******************************************************************************************/
uint8_t align(CarCommand *command, LDMap *ldmap, uint8_t minDistance, uint8_t maxDistance) 
{
   
    if (externalSensor.dist_1 < DISTANCE_THERSHOLD && externalSensor.dist_0 < DISTANCE_THERSHOLD)  
    {       
	int16_t diffDist = externalSensor.dist_1 - externalSensor.dist_0;
	
// 	bool robotLessThanMinDistance = externalSensor.dist_0 < minDistance && externalSensor.dist_1 < minDistance;
        bool robotGreaterThanMaxDistance = externalSensor.dist_0 > maxDistance && externalSensor.dist_1 > maxDistance;
	
	reset(ldmap);
// 	if ((diffDist > 0  || diffDist < -ALIGN_THRESHOLD) && stateData.direction == HORIZONTAL_LEFT)
	if (abs(diffDist) >= ALIGN_THRESHOLD)
	{
//   	  Serial.print(" LEFT diffDist ");
//   	  Serial.println(diffDist);
	  if (diffDist > 0)
	    setAngleToRotate(5);
	  else 
	      setAngleToRotate(-5);
	  rotate(command, true);
	}
// 	else if (robotLessThanMinDistance)
// 	{
// 	  setDistanceToTraverse(2);
// 	  moveForward(command, false, 8, true);
// 	  Serial.println(minDistance);
// 	  Serial.println("Too Close");
// 	}
	else if (robotGreaterThanMaxDistance)
	{
	   setDistanceToTraverse(3);
	   moveBackward(command, false, 7, true);
// 	   Serial.println("Too far");
	}
	else 
	{
	  return COMPLETED_TIOA;
	}
    }
    return INCOMPLETED_TIOA;
}

  

/*********************************************************************************************
*  scanningCode is the sate machine that scans looking for the code. When it finds the QRcode , it align the 
* robot to the QRcode 
* Prerequisite:
* 	externalSensor.code > 0
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      robot is align to the QRcode 
* ******************************************************************************************/
void scanningCodeInitRound(CarCommand *command, LDMap *ldmap, uint8_t state, uint32_t clock) 
{
    switch (state) 
    {
        case INIT:            
	    setAngleToRotate(10);
	    rotate(command, true);
            break;
        case ALIGN:
	  rotate(command, true);
          break;
    }
}

uint8_t scanningCodeEndRound(LDMap *ldmap, uint8_t *state, uint32_t clock) 
{
    switch (*state) 
    {
        case INIT:
	    if (newSensorReading && externalSensor.distance > 0)
	    {
		setAngleToRotate(externalSensor.angle);	      
		*state = ALIGN;
	    }
            break;
        case ALIGN:
	  if (rotationCompleted())
	  {
	      reset(ldmap);
	      return COMPLETED_TIOA;	
	  }
          break;
    }
    return INCOMPLETED_TIOA;
}



/*********************************************************************************************
*  approachingStateMachine is the sate machine that approaches the box
* Prerequisite:
* 	externalSensor.code > 0
*      externalSensor.distance > 0
*      externalSensor.offset != 0
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      Robot is within 10 cm from the front face of the box with QRode equal to externalSensor.code
* ******************************************************************************************/
void approachingStateMachineInitRound(CarCommand *command, LDMap *ldmap, uint8_t state, uint32_t clock) 
{
    switch (state) 
    {
        case VERTICAL:
	       moveBackward(command); 
            break;
        case HORIZONTAL:
            if (externalSensor.offset < 0)
                moveRight(command, false, 10); 
            else 
                moveLeft(command, false, 10);
            break;
        case APPROACHING:
            command->stopCondition = stopWhenAtLeastOneSensorsIsInBound;
            moveBackward(command, false, 10);
            break;
    }
}


/*********************************************************************************************
*  approachingStateMachine is the sate machine that approaches the box
* Prerequisite:
* 	externalSensor.code > 0
*      externalSensor.distance > 0
*      externalSensor.offset != 0
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      Robot is within 10 cm from the front face of the box with QRode equal to externalSensor.code
* ******************************************************************************************/
uint8_t approachingStateMachineEndRound(LDMap *ldmap, uint8_t *state, uint32_t clock) 
{
    switch (*state) 
    {
        case INIT:
            if (((stateData.isRobotInCorridor && stateData.currentFace > 0 && stateData.rank <= stateData.active) || (stateData.rank <= stateData.active)))
            {
               *state = VERTICAL;
                setDistanceToTraverse(30); 
            }
            break;
        case VERTICAL:
            if (distanceCompleted())
            {				    
                setDistanceToTraverse(MAX_ROBOT_DISTANCE);
                *state = HORIZONTAL;
            }
            break;
        case HORIZONTAL:
            if (newSensorReading && abs(externalSensor.offset) <= 2)
            {
                setDistanceToTraverse(MAX_OBJECT_DISTANCE);
                *state = APPROACHING;
            }
            break;
        case APPROACHING:
            if (isAtLeastOneSensorsInBound())
              return COMPLETED_TIOA;
            break;
    }
    return INCOMPLETED_TIOA;
}




/*********************************************************************************************
*  moveUntilFree Move the robot along the face of the box to the left/right. It measures the 
*    distance of the box where it starts if computDistance if true
* Prerequisite:
* 	Robot is within 15 and 5 cm from the face of the box
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      first: Stops the first time the car is completely in
*      computDistance: Compute the distance of the face since it starts
*      overpass: The the car overpass the edge so it can rotate
* Postcondition:
*      Robot is on the left/righ of the box free of obstruction if overpass = true 
* 
*      stateData.distance is the distance that the robot traverses, i.e., the length of the box 
*      from the initial position
* ******************************************************************************************/
void moveUntilFreeInitRound(CarCommand *command, LDMap *ldmap,  uint8_t state) 
{
    switch (state)
    {
      case ALIGN:
// 	    if (areBothSensorsInBounds())
	      align(command, ldmap, MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL);
// 	    else
// 	    {
// 		setDistanceToTraverse(5);
// 		if (stateData.direction == HORIZONTAL_LEFT)
// 		  moveLeft(command, false, 10);
// 		else
// 		  moveRight(command, false, 10);
// 	    }
	      
	    break;
	case CORRECTION_STATE:
	   command->stopCondition = stopWhenBothSensorsAreInBounds;
	   if (externalSensor.dist_1 > DISTANCE_THERSHOLD && externalSensor.dist_0 > DISTANCE_THERSHOLD)
	   {
	       if (stateData.direction == HORIZONTAL_RIGHT)
	       {
		  moveRight(command, false, 10);
// 		  Serial.print(externalSensor.dist_0);
// 		  Serial.print("Moving Right");
// 		  Serial.println(externalSensor.dist_1);
	       }
	       else
	       {
// 		 Serial.print(externalSensor.dist_0);
// 		  Serial.print("Moving Left");
// 		  Serial.println(externalSensor.dist_1);
		  
		  moveLeft(command, false, 10); 
	       }
	   }
	   else if (externalSensor.dist_0 >= (externalSensor.dist_1 + ALIGN_THRESHOLD))
	   {
// 	     Serial.println("Sensor 0 not in range");
	      moveRight(command, false, 10);
	   }
	    else 
	    {
// 	     Serial.println("Sensor 1 not in range");
	      moveLeft(command, false, 10);
	    }
	    break;	
	    
	case RESET_STATE:
  	    command->stopCondition = stopWhenBothSensorsAreInBounds;
	    if (stateData.direction == HORIZONTAL_LEFT)
	      moveLeft(command, false, 10);
	    else
	      moveRight(command, false, 10);
	    break;	
	
	case HORIZONTAL:
	    command->stopCondition = stopWhenBothSensorsAreOutOfBounds;
	    if (stateData.direction == HORIZONTAL_LEFT)
		moveLeft(command);
	    else
		moveRight(command);
	    break; 
//  	    moveAligned(command, ldmap, stateData.direction);
// 	    break;
	case HORIZONTAL_CORRECTION:		
	    if (stateData.direction == HORIZONTAL_LEFT)
		moveLeft(command, false, 10, true);
	    else
		moveRight(command, false, 10, true);
	  break;
    }
}


uint8_t moveUntilFreeEndRound(LDMap *ldmap,  uint8_t *state) 
{
    switch (*state)
    {
        case INIT:
          
            if (areBothSensorsInBounds())
            {
                *state = ALIGN;
            }
            else
            {
                setDistanceToTraverse(DISTANCE_CORRECTION); 
                *state = CORRECTION_STATE;
            }
          
          break;
        case ALIGN:
	  if (areBothSensorsInBounds())
	  {
	    if (isAligned(MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL))
	    {
	      reset(ldmap);
	      if (positionInTheBox == BEGINING_TRAVERSE)
		  return COMPLETED_TIOA;
	      else
	      {
		  *state  = HORIZONTAL;
		  if (stateData.direction == HORIZONTAL_LEFT)
		      stateData.initialDistance = externalSensor.dist_1;
		  else
		      stateData.initialDistance = externalSensor.dist_0;

		  setDistanceToTraverse(MAX_OBJECT_SIZE);
	      }
	    }  
	  }
	  else
	  {
	      setDistanceToTraverse(5); 
              *state = CORRECTION_STATE; 
	  }
          
          
          break;
    case CORRECTION_STATE:
	    if (areBothSensorsInBounds())
	    {
	       *state = ALIGN;
	    }
        break;	
	 
	case HORIZONTAL:
	  
	  if (areBothSensorsOutOfBounds())
	  {
	    if (positionInTheBox == OVERPASS_THE_BOX)
	    {

	      if (stateData.direction == HORIZONTAL_LEFT)
		      setDistanceToTraverse(DISTANCE_OVERPASS_LEFT);
	     else
	        setDistanceToTraverse(DISTANCE_OVERPASS_RIGHT);

	     *state = HORIZONTAL_CORRECTION;
	    }
	    else 
	    {
            setDistanceToTraverse(DISTANCE_OVERPASS_LEFT);
            if (stateData.direction == HORIZONTAL_LEFT)
                stateData.direction = HORIZONTAL_RIGHT;
            else
                stateData.direction = HORIZONTAL_LEFT;
            *state = RESET_STATE;    
	    }
	  }
	  break;
    case RESET_STATE:
 	  if (areBothSensorsInBounds())
	   {
// 	    if (stateData.rank != 1)
// 	    {
// 	      if (stateData.direction == HORIZONTAL_LEFT)
// 		stateData.direction = HORIZONTAL_RIGHT;
// 	      else
// 		stateData.direction = HORIZONTAL_LEFT;
// 	    }
	    return COMPLETED_TIOA;
        }
        break;
     case HORIZONTAL_CORRECTION:		
	   if (distanceCompleted())
	
	   return COMPLETED_TIOA;
	   break;
    }
    return INCOMPLETED_TIOA;
}

/*********************************************************************************************
* relocateBox relocate the robots to be ready to push
* Prerequisite:
* 	dist_0 < DISTANCE_THERSHOLD pr dist_1 < DISTANCE_THERSHOLD
* 	stateData.boxLength is the length of the box
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      Robot is placed at the approximate center of the box face
* ******************************************************************************************/
void relocateOnFaceInitRound(CarCommand *command, LDMap *ldmap,  uint8_t state, uint32_t clock)
{
  switch (state)
  {
    case INIT:
      
      align(command, ldmap, MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL);
      break;
    case HORIZONTAL:	
         command->stopCondition = stopWhenLeftSensorisInBound;
//         moveAligned(command, ldmap, stateData.direction);
	moveRight(command, false, 10);
      break;
    case ALIGN:
      align(command, ldmap, MIN_DISTANCE_PUSHING, MAX_DISTANCE_PUSHING);
      break;
    }
}

int16_t getDistanceToMove(int16_t *distanceBetweenRobots)
{
    int16_t distanceToMove=0;
    float length;
    if ((stateData.currentFace % 2) == 0)
	  length = stateData.boxLength; 
    else
	  length = stateData.boxWidth;
    
    *distanceBetweenRobots = length/(stateData.active + 1);
//     if (stateData.active == 1)
//     {
// 	  distanceToMove = *distanceBetweenRobots + LENGTH/2.0f;
//     }
//     else
    {
      
      if (stateData.direction == HORIZONTAL_LEFT)
	distanceToMove = (*distanceBetweenRobots)*stateData.rank - (LENGTH/2.0f);
      else 
	distanceToMove = (*distanceBetweenRobots)*(stateData.active - stateData.rank + 1) - (LENGTH/2.0f);

    }
    return max(distanceToMove, 0);
}

int8_t relocateOnFaceEndRound(LDMap *ldmap,  uint8_t *state, uint32_t clock, uint32_t rankPrevious)
{
  switch (*state)
  {
    case INIT:
	
       if (stateData.rank == 1 || rankPrevious == WAIT ||  rankPrevious == WAIT_TEST)
      {
 	if (isAligned(MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL))
	{
	    stateData.direction = HORIZONTAL_RIGHT;
	    int16_t Dn1 = stateData.distanceBetweenRobots;
	    
	    
 	    stateData.distanceToMove = getDistanceToMove(&stateData.distanceBetweenRobots);
	    
	    stateData.distanceToMove = (int16_t)(stateData.rank*(Dn1 - stateData.distanceBetweenRobots));    
// 	    Serial.print(stateData.distanceToMove);
// 	    Serial.println(" Distance to Move");
 	    if (stateData.distanceToMove > 21)
 	      stateData.distanceToMove -= 9; 
 	    reset(ldmap);
	    setDistanceToTraverse(stateData.distanceToMove);

	    *state  = HORIZONTAL;
	    stateData.initialDistance = externalSensor.dist_1;
        }
      }
      break;
    case HORIZONTAL:	
      if (distanceCompleted() || (stateData.direction == HORIZONTAL_LEFT && externalSensor.dist_2 <= stateData.distanceBetweenRobots) || 
 		    (stateData.direction == HORIZONTAL_RIGHT  && externalSensor.dist_3 <= stateData.distanceBetweenRobots))
      {
	  *state = ALIGN;
	  reset(ldmap);
      }
      break;
    case ALIGN:
      if (isAligned(MIN_DISTANCE_PUSHING, MAX_DISTANCE_PUSHING))
	return COMPLETED_TIOA;
    }
    return INCOMPLETED_TIOA;
}

/*********************************************************************************************
*  pushPosition Move the robot along the face of the box until it reaches the approximate 
*    center of the box
* Prerequisite:
* 	dist_0 < DISTANCE_THERSHOLD pr dist_1 < DISTANCE_THERSHOLD
* 	stateData.distance is the meassure of the box
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      Robot is placed at the approximate center of the box face
* ******************************************************************************************/
void centerPositionInitRound(CarCommand *command, LDMap *ldmap,  uint8_t state, uint32_t clock)
{    
  switch (state)
  {
    case INIT:
	align(command, ldmap, MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL);
	break;
    case HORIZONTAL:
  	if (stateData.direction == HORIZONTAL_RIGHT)
	{
  	  command->stopCondition = stopWhenLeftSensorisInBound;
	  moveRight(command, false, 10, true);
	}
  	else 
	{
   	  command->stopCondition = stopWhenRightSensorisInBound;
	  moveLeft(command, false, 10, true);
	}

        break;
    case ALIGN:
	align(command, ldmap, MIN_DISTANCE_PUSHING, MAX_DISTANCE_PUSHING);
	break;
    }
}

int8_t centerPositionEndRound(LDMap *ldmap,  uint8_t *state, uint32_t clock)
{    
  switch (*state)
  {
    case INIT:

      	if (isAligned(MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL))
	{
	      
	    stateData.distanceToMove = getDistanceToMove(&stateData.distanceBetweenRobots);
	    if (stateData.direction == HORIZONTAL_LEFT && stateData.rank == 1 && stateData.active > 1)
	    {
	      if (stateData.currentFace % 2 == 0)
		stateData.distanceToMove = 5;
	      else
		stateData.distanceToMove = 0;
	    }
	      
	    if (stateData.direction == HORIZONTAL_RIGHT && stateData.rank == stateData.active && stateData.active > 1)
	    {
	      if (stateData.currentFace % 2 == 0)
		stateData.distanceToMove = 5;
	      else
		stateData.distanceToMove = 0;
	    }
	    
	   
	    reset(ldmap);
	   
	   // if (stateData.distanceToMove >= 20 && stateData.active == 1)
	     // stateData.distanceToMove -= 8;
	    
 	    setDistanceToTraverse(stateData.distanceToMove*0.85);
	    if (stateData.direction == HORIZONTAL_LEFT)
	      stateData.initialDistance = externalSensor.dist_1;
	    else
	      stateData.initialDistance = externalSensor.dist_0;
	    *state  = HORIZONTAL;
	    
	}

 	break;
    case HORIZONTAL:	
        // Move left past the box
	if (distanceCompleted() || (stateData.direction == HORIZONTAL_RIGHT &&
                                externalSensor.dist_2 <= stateData.distanceBetweenRobots) || (stateData.direction == HORIZONTAL_LEFT  && externalSensor.dist_3 <= stateData.distanceBetweenRobots))
	
		  *state = ALIGN;

        break;
    case ALIGN:
        if (isAligned(MIN_DISTANCE_PUSHING, MAX_DISTANCE_PUSHING))
            return COMPLETED_TIOA;
        break;
    }
    return INCOMPLETED_TIOA;
}



/*********************************************************************************************
* Move the robot to the center of the box at f faces
* Prerequisite:
* 	Robot is in one side  of the face of the box 
* Params: 
*      command: command to send to the car
* 	ldamp: local state of the car (speed, position, etc)
*      state: state in the current state machine
*      clock: global clock
* Postcondition:
*      Robot is placed at the approximate center of the box face in the opposit side
* ******************************************************************************************/
void changeFaceInitFace(CarCommand *command, LDMap *ldmap,  uint8_t state, uint32_t clock) 
{
    switch (state) 
    {
        case MOVE_RIGHT_UNITL_FREE:
            moveUntilFreeInitRound(command, ldmap, stateData.stateSecondLevel);
	   break;
	case MOVE_LEFT_UNITL_FREE:
 	   moveUntilFreeInitRound(command, ldmap, stateData.stateSecondLevel);
	   break;
	case ROTATE_CAR:
 	   rotate(command, true);
	   break;
	case CENTER:
	   centerPositionInitRound(command, ldmap, stateData.stateSecondLevel, clock);
	   break;
    }
}

uint8_t changeFaceEndRound(LDMap *ldmap,  uint8_t *state, uint32_t clock) 
{
    switch (*state) 
    {
        case INIT:  
          break;
        case RESET_STATE:
            stateData.stateSecondLevel = INIT;
            if (stateData.faceTraverse < 0)
            {
                *state = MOVE_RIGHT_UNITL_FREE;
                stateData.direction = HORIZONTAL_LEFT;
            }
            else 
            {
                *state = MOVE_LEFT_UNITL_FREE;
                stateData.direction = HORIZONTAL_RIGHT;
            }
            if (abs(stateData.faceTraverse) > 1)
              positionInTheBox = OVERPASS_THE_BOX;
            else 
            {
              if (stateData.active > 1)
                    positionInTheBox = BEGINING_TRAVERSE;
              else
              {
                if (stateData.rank == 1 && !stateData.isRobotInCorridor)
                  positionInTheBox = FULLY_TRAVERSE;
                else  
                  positionInTheBox = BEGINING_TRAVERSE;
              }
            }
            break;  
        case MOVE_RIGHT_UNITL_FREE:
            if (moveUntilFreeEndRound(ldmap, &stateData.stateSecondLevel) == COMPLETED_TIOA)
            {
                stateData.faceTraverse++;
                if (stateData.faceTraverse < 0)
                {	
                    stateData.currentFace--; 
                    if (stateData.currentFace < 0)
                        stateData.currentFace = 3;
                    *state = ROTATE_CAR;
                    setAngleToRotate(-90);
                }
                else
                {
                    stateData.stateSecondLevel = INIT;
                    *state = CENTER;
                }

            }
            break;

        case MOVE_LEFT_UNITL_FREE:
          if (moveUntilFreeEndRound(ldmap, &stateData.stateSecondLevel)== COMPLETED_TIOA)
          {
            stateData.faceTraverse--;
            if (stateData.faceTraverse > 0)
            {	
              stateData.currentFace = (stateData.currentFace+1)%4;
              *state = ROTATE_CAR;
              setAngleToRotate(90);	      
            }
            else
            {
              stateData.stateSecondLevel = INIT;
              *state = CENTER; 
            }  
          }
          break;
        case ROTATE_CAR:
            if (rotationCompleted())
            {
              stateData.stateSecondLevel = INIT;
              *state = RESET_STATE;
              reset(ldmap);
            }
            break;
        case CENTER:
            return centerPositionEndRound(ldmap, &stateData.stateSecondLevel, clock);
    }
    return INCOMPLETED_TIOA;
}


/*********************************************************************************************
* pushBox: push the box to the main road 
* Prerequisite:
*      robot is in the center of the box
* Params: 
* 
*      command: command to send to the car
*      clock: global clock
* Postcondition:
*      Robot and box is free of obstacles
* ******************************************************************************************/
void pushBoxInitRound(CarCommand *command) 
{
    moveBackward(command, false, 15 + (stateData.active -1)*2, true);	
}

uint8_t testPushBoxEndRound(LDMap *ldmap) 
{       
          int16_t pos = stateData.distanceToMove + ldmap->position_x;

    /*if (isInRelocate)
    {
        stateData.active++;
        return RELOCATE_ON_FACE;
    }*/
    if ((numberOfTries > 4 || pos  <= 5) && stateData.rank == 1)
    {
        if (stateData.errorMovements >= 3) // && distanceTraversed() < TEST_DISTANCE/2)
	{
	    stateData.active++;
	    stateData.errorMovements = 0;
	    return RELOCATE_ON_FACE;
	}
        else    
        {
          stateData.distanceToMove = stateData.Xdistance/3;
          setDistanceToTraverse(stateData.distanceToMove);
          stateData.errorMovements = 0;
	  stateData.isRobotInCorridor = true;
// 	  return READY_TO_PUSH;
	  
        }

   }
   return INTERPUSH;
}


void setXYDistance(LDMap *ldmap)
{
    switch (stateData.currentFace)
    {
        case 0:
            stateData.Xdistance -= ldmap->position_x;
            break;
        case 2:
            stateData.Xdistance += ldmap->position_x;
            break;
        case 1:
            stateData.Ydistance -= ldmap->position_x;
            break;
        case 3:
            stateData.Ydistance += ldmap->position_x;
            break;
    }
}

void setTraversal(uint8_t isAnchorOn)
{
    stateData.localState = INIT;
//     Serial.print(" ***READY ");
//     Serial.println(isAnchorOn);
    switch (stateData.currentFace)
    {
      case 0:
        if (isAnchorOn == LEFT)
           stateData.faceTraverse = 2;
        else 
           stateData.faceTraverse = -2;
        break;
      case 1:
        stateData.faceTraverse = 2;
        break;
      case 2:
        if (isAnchorOn == RIGHT)
           stateData.faceTraverse = -2;
        else 
           stateData.faceTraverse = 2;
        break;
      case 3:
        stateData.faceTraverse =  -2;
        break;
    } 
}

uint8_t pushBoxEndRound(LDMap *ldmap, bool isInChangeFace, uint8_t isAnchorOn) 
{   
    int16_t pos = stateData.distanceToMove + ldmap->position_x;
    if (stateData.currentFace % 2 == 0)
    {
       stateData.errorMovements = 0;
    }
    if ((pos > 5 && stateData.errorMovements < 3) || stateData.rank > 1) 
    {
	  return INTERPUSH;
    }    
    else if (pos > 5 && stateData.errorMovements >= 3)  // There is an obstacle
    {
        setXYDistance(ldmap);
        stateData.errorMovements = 0;
//         Serial.print(" Not Completed (Obstacle) ");
//         Serial.print(isAnchorOn);
        stateData.localState = INIT;
        if (stateData.currentFace == 1)
            stateData.faceTraverse =  -2;
        else if (stateData.currentFace == 3)
            stateData.faceTraverse =  2;
        return OBSTACLE_AVOIDANCE;
    }	   
    else 
    {
        Serial.print("Completing ");
	Serial.println(pos);
        setXYDistance(ldmap);
        if (pos <= 5 && stateData.currentFace == 2 && stateData.YDone)
        {
            stateData.XDone = true;
        }
	else if (pos <= 5 && (stateData.currentFace % 2) == 1)
        {
            stateData.YDone = true;
        }
        
	
	if (stateData.YDone && stateData.XDone)
	  return END;
	else
        { 
	    stateData.localState = INIT;
            setTraversal(isAnchorOn);
            return CHANGE_FACE;
        }
    }
    return INTERPUSH;
}

void onInitAerospace()
{
    mutex = xSemaphoreCreateMutex();
    stateData.active = 0;
    stateData.faceTraverse = 0;
    stateData.currentFace  = 0;
    stateData.boxWidth = 0;
    stateData.boxLength = 0;
    stateData.Xdistance = 0;
    stateData.Ydistance = 0;
    stateData.YDone = false;
    stateData.XDone = false;
    stateData.isRobotInCorridor = false;
    stateData.localState = INIT;
    externalSensor.distance = 0;
    externalSensor.angle = 0;
    
    
    pidAligning.Kp = ALIGN_PROPORTIONAL;
    pidAligning.Ki = ALIGN_INTEGRAL;
    pidAligning.Kd = ALIGN_DERIVATE;


    arm_pid_init_f32(&pidAligning, 1);

}

void sendMessages(uint8_t globalState)
{
    MessageData messageData;
    
    messageData.param1 = SET_PARAM1(stateData.rank, stateData.active);
    messageData.param2 = SET_PARAM2(externalSensor.code, stateData.currentFace, stateData.isRobotInCorridor);
    messageData.Xdistance = stateData.Xdistance;
    messageData.Ydistance = stateData.Ydistance;
    messageData.globalState = globalState;
    messageData.boxWidth = stateData.boxWidth;
    messageData.boxLength = stateData.boxLength;
    sendMessageData(&messageData);
}


// Main state machine
uint8_t onNewOrthoRobot(CarCommand *command, LDMap *ldmap, uint8_t globalState, uint32_t clock)
{
   
    xSemaphoreTake(mutex, portMAX_DELAY);
    sendTaskToExternalComputer(externalSensor.code, globalState, stateData.localState);
    
    Serial.print("DD1,");
    Serial.print((int)getId());
    Serial.print(",");
    debugPrintState(globalState);
    Serial.print(",");
    debugPrintState(stateData.localState);
    Serial.print(",");
    debugPrintState(stateData.stateSecondLevel);
    Serial.print(",");
    Serial.print(stateData.rank);
    Serial.print(",");
    Serial.print(stateData.active);
     Serial.print(",");
    Serial.print(externalSensor.code);
    Serial.println("");
    command->adaptingOrientation = NULL;
    sendMessages(globalState);
    
    switch (globalState) 
    {
//          case DEBUG:
//  	      moveRight(command, false, 20, true);      
// 	    align(command, ldmap, MIN_DISTANCE_HORIZONTAL, MAX_DISTANCE_HORIZONTAL);
// 	      break;
	case INIT:
	  break;
	case SCANNING:
	  scanningCodeInitRound(command, ldmap,  stateData.localState, clock);
	  break;    
	case BROADCAST:
	  break;
	case APPROACHING:
        approachingStateMachineInitRound(command,  ldmap, stateData.localState, clock);
        break;
    case PREPARE_TO_MOVE:
        changeFaceInitFace(command, ldmap, stateData.localState, clock);  
        break;
    case RELOCATE_ON_FACE:
        relocateOnFaceInitRound(command, ldmap, stateData.localState, clock);
        break;
    case PUSH_BOX:
    case TEST_PUSH_BOX:
        pushBoxInitRound(command);
        break;
    case OBSTACLE_AVOIDANCE:
    case CHANGE_FACE:
//         if (stateData.localState == INIT && !areBothSensorsInBounds())
//         {
//             command->stopCondition = stopWhenBothSensorsAreInBounds;
//             if (stateData.rank == 1 && stateData.active > 1)
//                 moveLeft(command, false, 10, true);
            
//             if (stateData.rank == stateData.active && stateData.active > 1)
//                 moveRight(command, false, 10, true);
//         }
//         else
            changeFaceInitFace(command, ldmap, stateData.localState, clock);
        break;
    case WAIT:
    case WAIT_TEST:
        break;
    case STABILIZE:
        command->stopCondition = stopWhenLeftSensorisInBound;
	moveRight(command);
     break;
	case END:
	    break;
	default:
	    break;
    }
     xSemaphoreGive(mutex);
    return 0;
}



// Main state machine
uint8_t onEndOrthoRobot(LDMap *ldmap, uint8_t *globalState, uint32_t clock, uint8_t errCode, int16_t lastDistance, int16_t lastAngle)
{	 
    bool existsRobotInOtherFace = false;
    uint8_t code = externalSensor.code;
    uint8_t rank = 0;
    uint8_t maxRank  = 0;
    uint8_t membersInBroadcast=0, membersWithRank=0;
    uint8_t membersInWait=0;
    uint8_t stateRobotRankMinus1=INIT, stateRobotRankPlus1=INIT;
    int8_t activeInFace;
    uint8_t stateOfActive = 0;
    uint8_t membersLocal = 0;
    uint8_t isAnchorOn = LEFT;
    bool isRobotInCorridor = false;

    
//     Serial.print("*****ERROR CODE ");
//     Serial.println(errCode);
 // Getting the data that robots shared in the previous round   
    stateData.previousRankFailures++;
    switch (*globalState) 
    {
       case INIT:      
	   stateData.previousRankFailures = 0;
           membersLocal =  getScanningCode(&code); 
	  
	   break;
       case SCANNING:
	    stateData.previousRankFailures = 0;
	    membersLocal =  getMembers(); 
	    break;
       case BROADCAST:
	    stateData.previousRankFailures = 0;
// 	    membersLocal =  getBroadcastData(&rank, &maxRank, &membersWithRank, &stateOfActive,  (uint8_t)BROADCAST, &membersInBroadcast, PREPARE_TO_MOVE);
	    
	    break;
       case APPROACHING: 
	  getApproachingData(stateData.rank, &stateData.boxLength, &stateData.boxWidth,  &stateData.isRobotInCorridor,  
			       &stateData.previousRankFailures, &activeInFace, &stateData.Xdistance, &stateData.Ydistance, &stateData.active);
	 
// 	    if (stateData.rank > stateData.active && stateData.isRobotInCorridor)
	    {
// 		stateData.currentFace = activeInFace;
	    }
	   break;
	  
	case RELOCATE_ON_FACE:
	  
	  getRelocateData(stateData.rank, &stateData.active, &stateData.previousRankFailures);
	  break;
	case PREPARE_TO_MOVE:
    case OBSTACLE_AVOIDANCE:
	case CHANGE_FACE:
	  
	  getPrepareMove(stateData.rank, &stateRobotRankMinus1, &stateRobotRankPlus1, &stateData.previousRankFailures, &stateData.isRobotInCorridor);
	  break;
	case INTERPUSH: 
	  //if (stateData.isRobotInCorridor)
	    getInterPush(stateData.rank, stateData.active, stateData.currentFace, INTERPUSH, &stateOfActive, &membersInWait, &stateData.previousRankFailures, &isAnchorOn, &isRobotInCorridor);
	  //else
	    //getInterPush(stateData.rank, INTERPUSH, RELOCATE_ON_FACE, &existsRobotInOtherFace, &membersInWait, &stateData.previousRankFailures);
	  
	  break;
	case TEST_PUSH_BOX:
	case PUSH_BOX:
	  getPushData(stateData.rank, RELOCATE_ON_FACE, &existsRobotInOtherFace, &isAnchorOn, &stateData.previousRankFailures);
	  break;
	
	case WAIT_TEST:
	case WAIT:
	  getWaitData(stateData.rank, stateData.active, stateData.currentFace, &stateData.Xdistance, &stateData.Ydistance, WAIT, WAIT_TEST, INTERPUSH, &membersInWait,  &stateData.previousRankFailures);
	  break;
        default:
	    break;
	  
    }
    
    xSemaphoreTake(mutex, portMAX_DELAY);
    debugPrint(*globalState, ldmap);

    
    switch (*globalState) 
    {
       case INIT:  
            stateData.rank = 0;
            externalSensor.code = code;
            if (code > 0)
            {
		  stateData.localState = INIT;
 		 *globalState = SCANNING;
#ifdef __INTIAL_SETTING__
  		Serial.println("Set Debug params");
  		debugMode(globalState);
 #endif
            }
             stateData.members |= membersLocal;
            break;
        case SCANNING:
	      
            stateData.members |= membersLocal;
// 	    Serial.print("Members ");
// 	    Serial.println(membersLocal);
	    if (scanningCodeEndRound(ldmap, &stateData.localState, clock) == COMPLETED_TIOA) {
		  stateData.localState = INIT;
		  *globalState = BROADCAST;
	    }
           break;
        case BROADCAST:
	    membersLocal =  getBroadcastData(&rank, &maxRank, &membersWithRank, &stateOfActive,  (uint8_t)BROADCAST, &membersInBroadcast, PREPARE_TO_MOVE);
            stateData.members |= membersLocal;
// 	    Serial.println("Max Rank ");
//             Serial.println(maxRank);
            if (maxRank > stateData.rank)
            {
                stateData.rank = maxRank + 1;
                *globalState = APPROACHING;
                stateData.localState = INIT;
//                 stateData.active++;
            }
            else
            {

                if (membersInBroadcast == stateData.members)
                {
                    stateData.rank = rank;
                }
                if(stateData.members == membersWithRank)
                {
                    stateData.active++;
                    *globalState = APPROACHING;
                    stateData.localState = INIT;
                }
            }
            break;
	  
        case APPROACHING:
          if (approachingStateMachineEndRound(ldmap, &stateData.localState, clock) == COMPLETED_TIOA) {
                stateData.localState = RESET_STATE;
                *globalState = PREPARE_TO_MOVE;
		stateData.faceTraverse = stateData.isRobotInCorridor ?  activeInFace + 1 :  3;
            }
	  
	  break;
        
    case PREPARE_TO_MOVE:
	    if (changeFaceEndRound(ldmap, &stateData.localState, clock) == COMPLETED_TIOA) {  
	      stateData.localState = INIT;
	      if (stateData.isRobotInCorridor)
            *globalState = WAIT;
              else
            *globalState = WAIT_TEST;
	      stateData.errorMovements  = 0;
	  }
	   break;
        
	case RELOCATE_ON_FACE:
	    if (relocateOnFaceEndRound(ldmap, &stateData.localState, clock, stateRobotRankMinus1) == COMPLETED_TIOA) {
            stateData.localState = INIT;
            stateData.errorMovements  = 0;
            if (stateData.isRobotInCorridor)
              *globalState = WAIT;
            else
              *globalState = WAIT_TEST;
	    }
        break;
	    
    case OBSTACLE_AVOIDANCE:
	case CHANGE_FACE:
//         if (!areBothSensorsInBounds() && stateData.localState == INIT)
//         {
            
            
//         }
// 	else
//         {
            if (stateData.localState == INIT)
            {
//                 Serial.print(stateRobotRankMinus1);
//                 Serial.print("CHANGEFACE face = ");
//                 Serial.println(stateData.faceTraverse);
                stateData.localState = RESET_STATE;
                if (stateData.active > 1)
                {
        // 		    if (activeInFace > 0 && (stateData.rank < stateData.active && stateRobotRankMinus1  != WAIT))
                    if (stateData.faceTraverse > 0 && (stateData.rank > 1 && stateRobotRankMinus1  != WAIT))
                    {
//                     Serial.println("CF first if true");
                    stateData.localState = INIT;
                    }
                    if (stateData.faceTraverse < 0 && (stateData.rank < stateData.active && stateRobotRankPlus1 != WAIT))
                    {
//                     Serial.println("CF 2nd if true");
                    stateData.localState = INIT; 
                    }
                }
            }

            if (changeFaceEndRound(ldmap, &stateData.localState, clock ) == COMPLETED_TIOA) 
            {
                stateData.localState = INIT;
                *globalState = WAIT;
            }
//         }
	    break;
      case WAIT_TEST: 
	  if (membersInWait == ((1 << stateData.active) - 1))
	  {
	      
	      stateData.errorMovements = 0; 
	      *globalState = TEST_PUSH_BOX;
	      stateData.distanceToMove = TEST_DISTANCE;
	      setDistanceToTraverse(stateData.distanceToMove);
	  }
        break;
     case TEST_PUSH_BOX:
	    stateData.errorMovements += errCode;	 
	    numberOfTries++;
	    *globalState = testPushBoxEndRound(ldmap);
	    break;
      case INTERPUSH: 
	  if (!stateData.isRobotInCorridor && isRobotInCorridor)
	  {
	      stateData.distanceToMove = stateData.Xdistance/2;
	      setDistanceToTraverse(abs(stateData.distanceToMove));
	      stateData.errorMovements = 0;
	      stateData.isRobotInCorridor = true;
	  }
	  if (membersInWait == ((1 << stateData.active) - 1) && getOperationMode() == COOPERATIVE)
	  {
	      if (stateData.isRobotInCorridor)
		  *globalState = PUSH_BOX;
	      else
		  *globalState = TEST_PUSH_BOX;
	  }
	  else if (stateData.rank > 1)
	  {      
	      if (stateOfActive == CHANGE_FACE)
	      {
		stateData.localState = INIT;
		*globalState = CHANGE_FACE;
		setTraversal(isAnchorOn);
		stateData.isRobotInCorridor  = true;
	     }
	      else if (stateOfActive == OBSTACLE_AVOIDANCE)
	      {
              //setXYDistance(ldmap);
		stateData.localState = INIT;
		if (stateData.currentFace == 1)
		  stateData.faceTraverse =  -2;
		else if (stateData.currentFace == 3)
		  stateData.faceTraverse =  2;
		*globalState = OBSTACLE_AVOIDANCE;
	      }
	      else if (stateOfActive == RELOCATE_ON_FACE)
	      {
              stateData.active++;
              stateData.localState = INIT;
              *globalState = RELOCATE_ON_FACE;
	      }
	      else if (stateOfActive == END)
	      {
		      *globalState = END;
	      }
		      
	  }
	  break;
      case WAIT:
        
	  if (membersInWait == ((1 << stateData.active) - 1))
	  {
	      stateData.errorMovements = 0;
	      *globalState = PUSH_BOX;
	      stateData.localState = INIT;
	      if (stateData.currentFace == 0)
		stateData.distanceToMove = LENGTH;
	      else if (stateData.currentFace == 2 && stateData.rank == 1)
	      {
		if (stateData.YDone)
		  stateData.distanceToMove = stateData.Xdistance;
		else
		  stateData.distanceToMove = stateData.Xdistance / 2;
	      }
	      else if (stateData.rank == 1)
		stateData.distanceToMove = abs(stateData.Ydistance);
	      if (stateData.rank > 1)
		stateData.distanceToMove = MAX_DISTANCE_X;
	      
	      
	      setDistanceToTraverse(stateData.distanceToMove);
	  }
	  break;
      
      case PUSH_BOX:
        if (errCode == 0)
            stateData.errorMovements = 0;
 	 stateData.errorMovements += errCode;	 
  	 *globalState = pushBoxEndRound(ldmap, existsRobotInOtherFace, isAnchorOn);

	  break;
      case STABILIZE:
	if (distanceCompleted() || (stateData.direction == HORIZONTAL_LEFT && externalSensor.dist_2 <= stateData.distanceBetweenRobots) || 
 		    (stateData.direction == HORIZONTAL_RIGHT  && externalSensor.dist_3 <= stateData.distanceBetweenRobots))
        {
	      if (stateData.isRobotInCorridor)
		*globalState = WAIT;
	      else
		*globalState = WAIT_TEST;
	  }
	break;
      case END:
	  break;
        
    }
    //Failure recovery
    if (stateData.rank > 1 && stateData.previousRankFailures > 3)
    {
//         Serial.print("Failures ");
//         Serial.println(stateData.previousRankFailures);
        stateData.rank--; 
        switch (*globalState)
        {
          case WAIT:
          case WAIT_TEST:
	  case INTERPUSH:
          case PUSH_BOX:

	      
// 	      if (stateData.distanceBetweenRobots > 18)
// 		setDistanceToTraverse(stateData.distanceBetweenRobots - 9);
// 	      else
		setDistanceToTraverse(stateData.distanceBetweenRobots*0.85);
	      
// 	      Serial.print(stateData.distanceBetweenRobots);
// 	      Serial.println(" Moving");
	      *globalState = STABILIZE;
	      stateData.localState = INIT;
	      stateData.direction = HORIZONTAL_RIGHT;
	      break;

          case PREPARE_TO_MOVE:    
            case OBSTACLE_AVOIDANCE:
          case CHANGE_FACE:
              if (stateData.localState == CENTER)
              {
                *globalState = RELOCATE_ON_FACE;
                stateData.localState = INIT;
                stateData.direction = HORIZONTAL_RIGHT;

              }
              break;
        }
    }
    xSemaphoreGive(mutex);
    
    return 0;
}

#endif