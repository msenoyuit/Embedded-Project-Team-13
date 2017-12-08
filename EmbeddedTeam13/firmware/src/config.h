//rover 0 is the scout, rover 1 is the truck
#define THIS_ROVER_ID 0

#if THIS_ROVER_ID == 0
#define IS_SCOUT
#else
#define IS_TRUCK
#endif
