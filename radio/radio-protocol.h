#include <Arduino.h>
#include <RadioCommands.h>

uint8_t convertTo4Bit(uint16_t value11Bit);
int16_t map4BitTo11Bit(uint8_t value4Bit);
uint8_t map11BitTo4Bit(uint16_t value11Bit);

uint8_t convertTo4Bit(uint16_t value11Bit) { return map11BitTo4Bit(value11Bit); }

uint8_t map11BitTo4Bit(uint16_t value11Bit)
{
    // Initialize variables to track the closest match
    uint8_t closest4BitValue = 0;
    uint16_t smallestDifference = UINT16_MAX;

    const auto &lastEntry = channelValueMappingTable.back();
    // Find the closest match in the table
    for (const auto &entry : channelValueMappingTable)
    {

        uint8_t key = entry.first;     // Access the key
        uint16_t value = entry.second; // Access the value
        if (value11Bit >= lastEntry.second)
        {
            closest4BitValue = lastEntry.first;
            break;
        }
        else if (value11Bit <= value)
        {
            closest4BitValue = key;
            break;
        }
    }

    return closest4BitValue;
}

int16_t map4BitTo11Bit(uint8_t value4Bit)
{
    // Iterate through the mapping table to find the corresponding 11-bit value
    for (const auto &entry : channelValueMappingTable)
    {
        uint8_t key = entry.first;     // Access the 4-bit key
        uint16_t value = entry.second; // Access the 11-bit value

        if (key == value4Bit)
        {
            return value; // Return the 11-bit value if the key matches
        }
    }

    // If no match is found, return a default value or handle the error
    // For example, return 0 - 1500 or throw an exception
    return 1500; // Or handle the error as needed
}

// Test function
void testMap11BitTo4Bit()
{
    Serial.println("Test Mapping 11-bit to 4-bit");

    // Test cases: {input, expected_output}
    std::vector<std::pair<uint16_t, uint8_t>> testCases = {
        {1290, 0}, {1300, 0},  {1310, 1},  {1325, 1},  {1340, 2},
        {1500, 8}, {1600, 12}, {1700, 15}, {1675, 15}, {1800, 15}};
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
    bool failed = false;

    for (const auto &testCase : testCases)
    {
        uint16_t input = testCase.first;
        uint8_t expectedOutput = testCase.second;
        uint8_t actualOutput = map11BitTo4Bit(input);
        bool assert = actualOutput == expectedOutput;
        if (!assert)
        {
            Serial.print("Test Failed->");
            failed = true;
        }
        Serial.println("Test input " + String(input) + ": expected " +
                       String(expectedOutput) + ", got " + String(actualOutput));
        delay(100);
    }
    if (failed)
    {
        Serial.println("Test Failed");
        delay(500);
    }
}

// Test function
void testMap4BitTo11Bit()
{
    Serial.println("Testing map4BitTo11Bit");
    // Test cases: {input, expected_output}
    std::vector<std::pair<uint8_t, uint16_t>> testCases = {
        {0, 1300}, {0, 1300},  {1, 1325},  {1, 1325},  {2, 1350},
        {8, 1500}, {12, 1600}, {15, 1675}, {14, 1650}, {18, 1500}};

    // 1300 0
    // 1325 1
    // 1350 2
    // 1375 3
    // 1400 4
    // 1425 5
    // 1450 6
    // 1475 7
    // 1500 8
    // 1525 9
    // 1550 10
    // 1575 11
    // 1600 12
    // 1625 13
    // 1650 14
    // 1675 15
    bool failed = false;

    for (const auto &testCase : testCases)
    {
        uint16_t input = testCase.first;
        uint16_t expectedOutput = testCase.second;
        uint16_t actualOutput = map4BitTo11Bit(input);
        bool assert = actualOutput == expectedOutput;
        if (!assert)
        {
            Serial.print("Test Failed->");
            failed = true;
        }
        Serial.println("Test input " + String(input) + ": expected " +
                       String(expectedOutput) + ", got " + String(actualOutput));
        delay(100);
    }
    if (failed)
    {
        Serial.println("Test Failed");
        delay(500);
    }
}
