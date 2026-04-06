#include "file_io.h"
#include <LittleFS.h>

void initLittleFS()
{
    if (!LittleFS.begin(true))
    {
        Serial.println("An error has occurred while mounting LittleFS");
    }
    Serial.println("LittleFS mounted successfully");
}

String readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("- failed to open file for reading");
        return String("");
    }
    String content;
    Serial.println("- read from file:");
    while (file.available())
    {
        content += file.readStringUntil('\n') + "\n";
    }
    file.close();
    return content;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\r\n", path);
    Serial.printf("Content: %s\r\n", message);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("- file written");
        delay(500);
    }
    else
    {
        Serial.println("- write failed");
    }
    file.close();
}
