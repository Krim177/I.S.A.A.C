#ifndef   __GPS_TASK__
#define   __GPS_TASK__

typedef struct 
{
    float longitude;
    float latitude;
    unsigned long time;
} GPS_Data;

void gps6mv2Init();
bool gps6mv2Test();



#endif