#ifndef   __LDM_MAP__
#define   __LDM_MAP__


LDMap *getLDMP(uint8_t index);
uint8_t LDMapSize();

void ldmInit();
bool ldmTest();


uint8_t  numberOfMembers();
uint16_t ldmMembersInProgress();
uint16_t ldmMembersInSync(uint16_t mem);


uint32_t minWeightMatching(Point *points, uint32_t n);
float angleOfPoint(Point point);
float distance(Point point);
int join(int prop);
void SEC(float &radius, float &x, float &y);
void newRound();
void addState(LDMap *state);


uint8_t getQRCode(uint8_t code);
// uint8_t getMinDistance();
// uint8_t getMaxDistance();
uint8_t getSenderID();
uint8_t getBroadcastingMembers(uint16_t getCode);
uint16_t getActiveMembers();
uint8_t getRankingMembers();

// int16_t getAnchorOffset();
// uint8_t rankRobot();
// uint8_t getState(uint8_t pushState);
void getDebugger();

uint32_t getClock();


void startSending();
void stopSending();
void addRobotState(LDMap *ldm);

void insertLDMAP(uint8_t r, LDMap *state);


uint8_t getScanningCode(uint8_t *code);
uint8_t getMembers();
uint8_t getBroadcastData(uint8_t *rank, uint8_t *maxRank, uint8_t *memberswithRank, uint8_t  *stateOfActive, 
		      uint8_t BroadcastState, uint8_t *membersInBroadcast, uint8_t minState);

void getApproachingData(uint8_t rank, uint8_t *boxLength, uint8_t *boxWidth, bool *isRobotInCorridor,
			   uint8_t *previousRankFailures, int8_t *face, int16_t *Xdistance, int16_t *Ydistance, uint8_t *active);

void getPrepareMove(uint8_t rank, uint8_t *stateRankMinus1, uint8_t *stateRankPlus1, uint8_t *previousRankFailures, bool *isRobotInCorridor);
void getRelocateData(uint8_t rank, uint8_t *active, uint8_t *previousRankFailures);
void getChangeFaceData(uint8_t rank, uint8_t *stateRankMinus1, uint8_t *stateRankPlus1, 
		       int8_t *boxLength, int8_t *boxWidth, uint8_t *previousRankFailures);

void getWaitData(uint8_t rank, uint8_t active,  uint8_t currentFace, int16_t *Xdistance, int16_t *Ydistance, uint8_t waitState, uint8_t waitTestState,
      uint8_t interpush, uint8_t *memberInWait,  uint8_t *previousRankFailures);

void getInterPush(uint8_t rank, uint8_t active, uint8_t currentFace, uint8_t interPush, uint8_t *stateLeader,  uint8_t *memberInWait, uint8_t *previousRankFailures, uint8_t *isAnchorOn, bool *isInCorridor);

void getPushData(uint8_t rank,  uint8_t relocateInFace, bool *isInDiffState, uint8_t *isAnchorOn,  uint8_t *previousRankFailures);


#endif
