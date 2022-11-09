#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


#include <PWMServo.h>
#include <arm_math.h>
#include "pubsub.h"
#include "datastructures.h"

#include "config.h"


#define R 0
#define RPLUS1 1
#define RMINUS1 2

static bool bInit = false;


LDMap dynamicMap[3][LDM_SIZE];
SemaphoreHandle_t mtx;

LDMap *getLDMP(uint8_t id)
{
    return &dynamicMap[RMINUS1][id-1];
}


void insertLDMAP(uint8_t r, LDMap *state)
{
    memcpy(&dynamicMap[r][state->id], state, sizeof(LDMap)); 
}


void ldmInit()
{
   
    if (bInit == false)
    {
          bInit = true;
          mtx = xSemaphoreCreateMutex();
          memset(&dynamicMap[0], 0, sizeof(LDMap)*LDM_SIZE);
          memset(&dynamicMap[1], 0, sizeof(LDMap)*LDM_SIZE);
          memset(&dynamicMap[2], 0, sizeof(LDMap)*LDM_SIZE);
    } 
}

bool ldmTest()
{
    return bInit;
  }

void newRound()
{
    uint8_t i;
    if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
    {       
      for (i = 0; i < LDM_SIZE; i++)
      {
	memcpy(&dynamicMap[RMINUS1][i], &dynamicMap[R][i], sizeof(LDMap));
	memcpy(&dynamicMap[R][i], &dynamicMap[RPLUS1][i], sizeof(LDMap));
	dynamicMap[RPLUS1][i].id = 0;
      }
      xSemaphoreGive(mtx);
      
    }
}

int  minMatching(Point *point, uint32_t n)
{
    return 0;
}	

float  angleOfPoint(Point point)
{
    float a = 0;
    uint8_t i = getId()-1;
    if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
    {
	float x = dynamicMap[R][i].position_x;
	float y = dynamicMap[R][i].position_y;
	float angle = dynamicMap[R][i].heading * M_PI/180.0f;
	xSemaphoreGive(mtx);
	
	float x1 = point.x - x;
	float y1 = point.y - y;
	
	
	float x2 =  cos(angle)*x1 - sin(-angle)*y1;
	float y2 =  sin(-angle)*x1 + cos(angle)*y1;
	
	
	if (y2 == 0 && x2 == 0)
		return 0;
	a = atan2(y2, x2) * 180/M_PI;
    }
    return a;
}


float distance(Point point)
{
    float dist = 0;
    uint8_t i = getId()-1;
    if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
    {
	float x1 = dynamicMap[R][i].position_x;
	float y1 = dynamicMap[R][i].position_y;
 	xSemaphoreGive(mtx);
	
	float x = point.x - x1;
	float y = point.y - y1;
	
	arm_sqrt_f32(x*x + y*y, &dist);
    }
    return dist;
}


int join(int propose)
{
    int ret = 0;
    uint8_t i;
    if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
    {
      for (i = 0; i< LDM_SIZE; i++)
      {
	  if (dynamicMap[R][i].id > 0 && dynamicMap[R][i].propose != propose)
	  {
	    ret = 1;
	    break;
	  }
      }
      xSemaphoreGive(mtx);  
    }
    return ret;
}

void SEC(float &radius, float &x, float &y)
{
  
}

int minMatching(Point *points)
{
    return 0;
}

void convexHull()
{
  
}


#define getRank GET_RANK(dynamicMap[RMINUS1][i].messageData.param1)
#define getActive GET_ACTIVE(dynamicMap[RMINUS1][i].messageData.param1)
#define getCode GET_CODE(dynamicMap[RMINUS1][i].messageData.param2)
#define getFace GET_FACE(dynamicMap[RMINUS1][i].messageData.param2)
#define getCorridor GET_CORRIDOR(dynamicMap[RMINUS1][i].messageData.param2)

#define COMPUTE_MEMBERS members |= 1 << i
#define FAILURE_DETECTOR  if (rank <=1)  *previousRankFailures = 0;  \
	    else if (getRank == (rank-1))  *previousRankFailures = 0;
#define IS_IN_STATE(state) dynamicMap[RMINUS1][i].messageData.globalState == state ? true : false; 
	    
uint8_t getScanningCode(uint8_t *code)
{
    uint8_t members = 0;
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    {   
	if (dynamicMap[RMINUS1][i].id > 0) 
	{
	    //Computes the members in the system
	    COMPUTE_MEMBERS;
	    uint8_t codej = getCode;
	    
			      // Compute the maximum code. All robots should agree on the code
	     *code = max(codej, *code);
	     
	}                    
	
    }
    return members;
}


uint8_t getMembers()
{
    uint8_t members = 0;
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    {   
	if (dynamicMap[RMINUS1][i].id > 0) 
	{
	    //Computes the members in the system
	    COMPUTE_MEMBERS;
	}                    
	
    }
    return members;
}


uint8_t getBroadcastData(uint8_t *rank, uint8_t *maxRank, uint8_t *memberswithRank, uint8_t  *stateOfActive, uint8_t BroadCastState, uint8_t *membersInBroadcast, uint8_t minState)
{
    uint8_t members = 0;
    uint8_t id = getId() - 1;
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    { 
	
	if (dynamicMap[RMINUS1][i].id > 0) 
	{
	    COMPUTE_MEMBERS;
	    // Computes the rank
	    if (abs(dynamicMap[RMINUS1][i].messageData.Ydistance) < abs(dynamicMap[RMINUS1][id].messageData.Ydistance) ||
		(abs(dynamicMap[RMINUS1][i].messageData.Ydistance) == abs(dynamicMap[RMINUS1][id].messageData.Ydistance) && i <= id) )  
		(*rank)++;
	    
	    
	    uint8_t rankj = getRank;
	    uint8_t activej = getActive;
	    
	    // Members that have rank already computed
	    if (rankj > 0)
		*memberswithRank |= 1 << i;
	    
	      //Computes the members in the system in state2
	    if (dynamicMap[RMINUS1][i].messageData.globalState == BroadCastState)
		*membersInBroadcast |= 1 << i;
		    // gets the maximum rank 
	    if (rankj > *maxRank && dynamicMap[RMINUS1][i].messageData.globalState >= minState)
		*maxRank = rankj;
	    
	    if (rankj < activej)
		  *stateOfActive = max(dynamicMap[RMINUS1][i].messageData.globalState, *stateOfActive);
	    
	}    
   }
  return members;
}




void getApproachingData(uint8_t rank, uint8_t *boxLength, uint8_t *boxWidth, bool *isRobotInCorridor,
			   uint8_t *previousRankFailures, int8_t *face, int16_t *Xdistance, int16_t *Ydistance, uint8_t *active)
{
    uint8_t id = getId() - 1;
    uint8_t i;

    for (i = 0; i < LDM_SIZE; i++)
    {
	
	if (dynamicMap[RMINUS1][i].id>0)
	{
	  FAILURE_DETECTOR;           	    
	  
	  if (abs(dynamicMap[RMINUS1][i].messageData.Xdistance) > abs(*Xdistance))
	    *Xdistance = dynamicMap[RMINUS1][i].messageData.Xdistance;
	  
	  if (abs(dynamicMap[RMINUS1][i].messageData.Ydistance) > abs(*Ydistance) || 
	      (abs(dynamicMap[RMINUS1][i].messageData.Ydistance) > abs(*Ydistance) && dynamicMap[RMINUS1][i].id < id))
	    *Ydistance = dynamicMap[RMINUS1][i].messageData.Ydistance;
	  
	  
	  uint8_t rankj = getRank;
	    
	  //Gets the distance of robot 
	  
	  if (rankj == 1 && dynamicMap[RMINUS1][i].id != id)
	  {
	      *boxLength = dynamicMap[RMINUS1][i].messageData.boxLength;
	      *boxWidth = dynamicMap[RMINUS1][i].messageData.boxWidth; 
	      *face = getFace;
	      *isRobotInCorridor = getCorridor;
	      *active = getActive;
	  }
	}
    }
}


void getPrepareMove(uint8_t rank, uint8_t *stateRankMinus1, uint8_t *stateRankPlus1, uint8_t *previousRankFailures, bool *isRobotInCorridor)
{
    uint8_t i;
    uint8_t id = getId() - 1;
    *stateRankMinus1 = 0;
    *stateRankPlus1 = 0;
    for (i = 0; i < LDM_SIZE; i++)
    {
	// Resets previousRankFailures if it has received messages from robot rank - 1 or actualRank = 0
	FAILURE_DETECTOR;           	    	
	
	if (dynamicMap[RMINUS1][i].id > 0) 
	{
	      uint8_t rankj = getRank;
	
	        //Gets the distance of robot 1
	      if (rankj == 1 && dynamicMap[RMINUS1][i].id != id)
		  *isRobotInCorridor = getCorridor;
		
	      if (rankj == (rank-1))
		  *stateRankMinus1 = dynamicMap[RMINUS1][i].messageData.globalState;
	      if (rankj == (rank+1))
		  *stateRankPlus1 = dynamicMap[RMINUS1][i].messageData.globalState;
	}        
    }
}


void getRelocateData(uint8_t rank, uint8_t *active, uint8_t *previousRankFailures)
{
    uint8_t id = getId() - 1;
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    {
	// Resets previousRankFailures if it has received messages from robot rank - 1 or actualRank = 0
	
	FAILURE_DETECTOR;           	    

	uint8_t rankj = getRank;
	uint8_t activej = getActive;
	    
	//Gets the distance of robot 1
	if (rankj == 1 && dynamicMap[RMINUS1][i].id != id )
	    *active = activej;
	
    }
}

void getWaitData(uint8_t rank, uint8_t active, uint8_t currentFace, int16_t *Xdistance, int16_t *Ydistance, uint8_t waitState, uint8_t waitTestState,
	uint8_t interpush, uint8_t *memberInWait,  uint8_t *previousRankFailures)
{
    uint8_t id = getId() - 1;
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    {
	FAILURE_DETECTOR;           	    	
	
	if (dynamicMap[RMINUS1][i].id > 0) 
	{
	    
	    uint8_t rankj = getRank;
	    uint8_t facej = getFace;
	    
	    if (rankj == 1 && dynamicMap[RMINUS1][i].id != id)
	    {
		*Xdistance = dynamicMap[RMINUS1][i].messageData.Xdistance;
		*Ydistance = dynamicMap[RMINUS1][i].messageData.Ydistance; 
	    }
	    if ((dynamicMap[RMINUS1][i].messageData.globalState == waitState || dynamicMap[RMINUS1][i].messageData.globalState == waitTestState || dynamicMap[RMINUS1][i].messageData.globalState == interpush) && 
		rankj > 0 && rankj <= active && facej == currentFace)
		*memberInWait |= 1 << (rankj-1);
	}    
	    
	
    }
}  


void getInterPush(uint8_t rank, uint8_t active, uint8_t currentFace, uint8_t interPush, uint8_t *stateLeader,  uint8_t *memberInWait, uint8_t *previousRankFailures, uint8_t *isAnchorOn, bool *isInCorridor)
{
    uint8_t i;
    for (i = 0; i < LDM_SIZE; i++)
    {
        FAILURE_DETECTOR;

        uint8_t rankj = getRank;
	 uint8_t facej = getFace;
        if (dynamicMap[RMINUS1][i].id > 0 && rankj == 1) 
        {
	    *isInCorridor = getCorridor;
            *stateLeader = dynamicMap[RMINUS1][i].messageData.globalState;
            //*Xdistance = dynamicMap[RMINUS1][i].messageData.Xdistance;
            //*Ydistance = dynamicMap[RMINUS1][i].messageData.Ydistance;
            if (dynamicMap[RMINUS1][i].messageData.Ydistance > 0)
                *isAnchorOn  = RIGHT;
            else
                *isAnchorOn = LEFT; 
        }
        if (dynamicMap[RMINUS1][i].id > 0 && rankj > 0 && rankj <= active) 
        {            
            if (dynamicMap[RMINUS1][i].messageData.globalState == interPush && currentFace ==  facej)
                *memberInWait |= 1 << (rankj-1);
        }    
    }
    
}

void getPushData(uint8_t rank, uint8_t relocateInFace, bool *isInDiffState, uint8_t *isAnchorOn, 
		 uint8_t *previousRankFailures)
{
    uint8_t i;
    uint8_t id = getId() - 1;
    *isInDiffState = false;
    for (i = 0; i < LDM_SIZE; i++)
    {
	FAILURE_DETECTOR;           	    
	if (dynamicMap[RMINUS1][i].id > 0)
	{
        if (!*isInDiffState)
	       *isInDiffState = IS_IN_STATE(relocateInFace);
	    uint8_t rankj = getRank;
	    if (rankj == 1) // && dynamicMap[RMINUS1][i].id != id)
	    {  
// 		*Xdistance = dynamicMap[RMINUS1][i].messageData.Xdistance;
// 		*Ydistance = dynamicMap[RMINUS1][i].messageData.Ydistance;
		if (dynamicMap[RMINUS1][i].messageData.Ydistance > 0)
		    *isAnchorOn  = RIGHT;
		else
		    *isAnchorOn = LEFT; 
	    }
	    
	}
    }
}
/*
uint8_t getStateData(uint8_t *code, int16_t *Xdistance, int16_t *Ydistance, 
                     uint8_t *boxLength, uint8_t *boxWidth, bool *isRobotInCorridor,
                     uint8_t *isAnchorOn, uint8_t *active,                      
                     uint8_t *rank, uint8_t *maxRank, uint8_t actualRank,
                     uint8_t *stateRankMinus1, uint8_t *stateRankPlus1, 
                     uint8_t *memberswithRank, 
                     uint8_t *previousRankFailures, 
                     uint8_t state2, uint8_t *membersInState2, 
                     uint8_t state3, uint8_t state5, uint8_t *membersInState3,
                     uint8_t state1, bool *isInstate1,
                     uint8_t state4, bool *isInstate4,
                     int8_t *face, uint8_t *stateOfActive)
{
    uint8_t members = 0;
    uint8_t id = getId() - 1;
    uint8_t i;
    if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
     {
        for (i = 0; i < LDM_SIZE; i++)
        {
//  	    Serial.print(i);
//  	    Serial.print(" --- ");
//  	    Serial.print(dynamicMap[RMINUS1][i].id);
// 	    Serial.print("TESTIN ");
// 	    Serial.println(dynamicMap[RMINUS1][i].messageData.rank);
// 	    Serial.println(*previousRankFailures);
	  
	    // Resets previousRankFailures if it has received messages from robot rank - 1 or actualRank = 0
	    if (actualRank <=1)
	      *previousRankFailures = 0;
	    else if (dynamicMap[RMINUS1][i].messageData.rank == (actualRank-1))
		*previousRankFailures = 0;
            
	    
	    if (dynamicMap[RMINUS1][i].id > 0) 
            {
                //Computes the members in the system
                members |= 1 << i;
                
                 // Compute the maximum code. All robots should agree on the code
                *code = max(dynamicMap[RMINUS1][i].messageData.qrCode, *code);
                *isRobotInCorridor = (*isRobotInCorridor || dynamicMap[RMINUS1][i].messageData.inCorridor);
                
                //Computes the members in the system in state2
                if (dynamicMap[RMINUS1][i].messageData.globalState == state2)
                    *membersInState2 |= 1 << i;
                
		// Computes the rank
		if (abs(dynamicMap[RMINUS1][i].messageData.Ydistance) < abs(dynamicMap[RMINUS1][id].messageData.Ydistance) ||
                    (abs(dynamicMap[RMINUS1][i].messageData.Ydistance) == abs(dynamicMap[RMINUS1][id].messageData.Ydistance) && i <= id) )
		  
                    (*rank)++;
                // Members that have rank already computed
                if (dynamicMap[RMINUS1][i].messageData.rank > 0)
                    *memberswithRank |= 1 << i;
                
                
                // gets the maximum rank and the anchor direction
                if (dynamicMap[RMINUS1][i].messageData.rank > *maxRank)
                {
                    *maxRank = dynamicMap[RMINUS1][i].messageData.rank;
                    if (dynamicMap[RMINUS1][i].messageData.Ydistance >= 0)
                        *isAnchorOn  = LEFT;
                    else
                        *isAnchorOn = RIGHT; 
                }

                
                if ((dynamicMap[RMINUS1][i].messageData.globalState == state3 || dynamicMap[RMINUS1][i].messageData.globalState == state5) && 
                    dynamicMap[RMINUS1][i].messageData.rank > 0)
                        *membersInState3 |= 1 << (dynamicMap[RMINUS1][i].messageData.rank-1);
                
                if (dynamicMap[RMINUS1][i].id != (getId()-1))
                {
                   
                    *active = max(dynamicMap[RMINUS1][i].messageData.active, *active);
                    
                    if (dynamicMap[RMINUS1][i].messageData.rank == (actualRank-1))
                        *stateRankMinus1 = dynamicMap[RMINUS1][i].messageData.globalState;
                    if (dynamicMap[RMINUS1][i].messageData.rank == (actualRank+1))
                        *stateRankPlus1 = dynamicMap[RMINUS1][i].messageData.globalState;
                    //Gets the distance of robot 1
                    if (dynamicMap[RMINUS1][i].messageData.rank == 1)
                    {
                        *Xdistance = dynamicMap[RMINUS1][i].messageData.Xdistance;
                        *Ydistance = dynamicMap[RMINUS1][i].messageData.Ydistance;
                            
                        *boxLength = dynamicMap[RMINUS1][i].messageData.boxLength;
                        *boxWidth = dynamicMap[RMINUS1][i].messageData.boxWidth; 
			
                    }

                    // isInstate1 sets to true if there is a robot in state1
                    if (dynamicMap[RMINUS1][i].messageData.globalState == state1)
                        *isInstate1 = true;
                    if (dynamicMap[RMINUS1][i].messageData.globalState == state4)
                        *isInstate4 = true;
                    
                    if (dynamicMap[RMINUS1][i].messageData.rank <= *active)
                        *face = dynamicMap[RMINUS1][i].messageData.face;
                    
                    if (dynamicMap[RMINUS1][i].messageData.rank == *active)
                        *stateOfActive = dynamicMap[RMINUS1][i].messageData.globalState;
                }
	    }    
                
           
        }
        xSemaphoreGive(mtx); 
  }
  return members;
}*/


void getDebugger()
{
	uint8_t i;
	if (xSemaphoreTake(mtx, portMAX_DELAY) == pdTRUE)
	{
	  for (i = 0; i < LDM_SIZE; i++)
	  {
	    if (dynamicMap[R][i].id > 0 && dynamicMap[R][i].id != getId())
	    {
		    Serial.print(dynamicMap[R][i].position_x);
	    }
	  }
	  xSemaphoreGive(mtx);
	}
}
/*
uint32_t getClock(uint8_t id)
{
    return  dynamicMap[R][id -1].clock;
}*/




