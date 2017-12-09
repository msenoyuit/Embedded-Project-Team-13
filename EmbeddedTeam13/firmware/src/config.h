#define SCOUT_ID 0
#define TRUCK_ID 1

#define THIS_ROVER_ID TRUCK_ID

#if THIS_ROVER_ID == SCOUT_ID
#define IS_SCOUT
#else
#define IS_TRUCK
#endif
