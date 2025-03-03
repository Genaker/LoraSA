#include <Arduino.h>
#include <map>

std::map<int, int> readSerialInput();
void dumpCommand(const std::map<int, int> &map);
std::map<int, int> processSerialCommand(const String &input);

std::map<int, int> readSerialInput()
{
    std::map<int, int> result;
    String input = "";
    if (Serial.available() > 0)
    {
        input = Serial.readStringUntil(
            '\n');    // Read the incoming data until a newline character
        input.trim(); // Remove any leading or trailing whitespace
    }

    if (!input.isEmpty())
    {
        Serial.println("SERIAL Data: " + input);
        delay(1000);
        result = processSerialCommand(input);
    }

    if (!result.empty())
    {
        Serial.println("The map contains data: ");
        dumpCommand(result);
        delay(1000);
    }

    return result; // Return an empty string if no data is available
}

void dumpCommand(const std::map<int, int> &map)
{
    for (const auto &pair : map)
    {
        Serial.println("Key: " + String(pair.first) + ", Value: " + String(pair.second));
    }
}

std::map<int, int> processSerialCommand(const String &input)
{
    const int MAX_KEYS = 6;

    std::map<int, int> keyValuePairs;
    if (input.startsWith("RC "))
    {
        int key1 = -1, value1 = -1;
        int key2 = -1, value2 = -1;
        int key3 = -1, value3 = -1;
        int key4 = -1, value4 = -1;
        int key5 = -1, value5 = -1;
        int key6 = -1, value6 = -1;

        char command[3];
        int parsed = sscanf(input.c_str(), "%s %d %d %d %d %d %d %d %d %d %d %d %d",
                            command, &key1, &value1, &key2, &value2, &key3, &value3,
                            &key4, &value4, &key5, &value5, &key6, &value6);

        // Create arrays to hold keys and values
        int keys[MAX_KEYS] = {key1, key2, key3, key4, key5, key6};
        int values[MAX_KEYS] = {value1, value2, value3, value4, value5, value6};

        if (parsed >= 3)
        {
            Serial.print("Received command: ");
            Serial.print(command);
            Serial.print(" ");
            for (int i = 0; i < parsed - 2; i++)
            {
                if (values[i] != -1)
                {
                    keyValuePairs[keys[i]] = values[i];
                }
                Serial.print("Key[" + String(i) + "]: ");
                Serial.print(keys[i]);
                Serial.print(" Value[" + String(i) + "]: ");
                Serial.print(String(values[i]) + ", ");
            }
            Serial.println(" ");
            Serial.println(">--------------------------------<");
        }
        else
        {
            Serial.println("Invalid command format or out of range values.");
        }
    }
    else
    {
        Serial.println("Invalid command prefix.");
    }
    return keyValuePairs;
}
