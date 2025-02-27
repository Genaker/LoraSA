
#include <Arduino.h>
#include <map>
#include <unordered_map>
#include <vector>

uint16_t testChannels[16] = {1500, 2000, 1350, 1400, 1505, 1506, 1507, 1508,
                             1509, 1510, 1511, 1512, 1513, 1514, 1515, 1516};

/*
commands      sending message              comments
-----------------------------------------------------
roll	      rc 1 <value>	        // 	move left or right
pitch         rc 2 <value>          // move forward or backwards
yaw	          rc 4 <value>	        // turn left or right
throttle      rc 3 <value>  	    // move up or down
*/
enum Command
{
    HEART_BEAT = 0, // Corresponds to rc 0
    ROLL = 1,       // Corresponds to rc 1
    PITCH = 2,      // Corresponds to rc 2
    THROTTLE = 3,   // Corresponds to rc 3
    YAW = 4,        // Corresponds to rc 4
    ////  ----- Not Assigned Yet -----
    AUX1 = 5, // Corresponds to rc 5
    AUX2 = 6, // Corresponds to rc 6
    AUX3 = 7, // Corresponds to rc 7
    AUX4 = 8, // Corresponds to rc 8
    AUX5 = 9, // Corresponds to rc 9
    AUX6 = 10 // Corresponds to rc 10
};

// Create a map from Command to string
std::unordered_map<Command, String> commandToStringMap = {{HEART_BEAT, "HEART_BEAT"},
                                                          {ROLL, "ROLL"},
                                                          {PITCH, "PITCH"},
                                                          {YAW, "YAW"},
                                                          {THROTTLE, "THROTTLE"}};

// Define the mapping table
std::vector<std::pair<uint8_t, uint16_t>> channelValueMappingTable = {
    {0, 1300},  {1, 1325},  {2, 1350},  {3, 1375}, {4, 1400},  {5, 1425},
    {6, 1450},  {7, 1475},  {8, 1500},  {9, 1525}, {10, 1550}, {11, 1575},
    {12, 1600}, {13, 1625}, {14, 1650}, {15, 1675}};
// 0 - 1300 0
// 1300 - 1325 1
// 1325 - 1350 2
// 1350 - 1375 3
// 1375 - 1400 4
// 1400 - 1425 5
// 1425 - 1450 6
// 1450 - 1475 7
// 1475 - 1500 8
// 1500 - 1525 9
// 1525 - 1550 10
// 1550 - 1575 11
// 1575 - 1600 12
// 1600 - 1625 13
// 1625 - 1650 14
// 1650 - 1675 15

#define INIT_SBUS_ARRAY                                                                  \
    {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,                                     \
     1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}
