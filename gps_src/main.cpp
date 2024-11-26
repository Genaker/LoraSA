#include <TinyGPS++.h>
#include <time.h>

// Objects for GNSS parsing and serial communication
TinyGPSPlus gps;
HardwareSerial GNSSSerial(2);

// Pin definitions for GNSS module communication
const int GNSS_RXPin = 34;
const int GNSS_TXPin = 33;
const int GNSS_RSTPin = 35; // There is a function built for this in the example below-
                            // currently it isn't used
const int GNSS_PPS_Pin = 36;

// Flags for PPS handling and synchronization status
volatile bool ppsFlag = false;
volatile bool initialSyncDone = false;

// Timestamp for the last valid GNSS data received
unsigned long lastGNSSDataMillis = 0;

void setup()
{
    // Initialize serial communication for debugging
    USBSerial.begin(115200);
    while (!USBSerial)
        ;

    // Start GNSS module communication
    GNSSSerial.begin(115200, SERIAL_8N1, GNSS_TXPin, GNSS_RXPin);
    while (!GNSSSerial)
        ;

    // Configure GNSS reset pin
    pinMode(GNSS_RSTPin, OUTPUT);
    digitalWrite(GNSS_RSTPin, HIGH);

    // Set up PPS pin and attach an interrupt handler
    pinMode(GNSS_PPS_Pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(GNSS_PPS_Pin), ppsInterrupt, RISING);

    // Short delay for GNSS module initialization
    delay(1000);
}

void loop()
{
    // Process incoming GNSS data
    while (GNSSSerial.available())
    {
        if (gps.encode(GNSSSerial.read()))
        {
            // Update the timestamp when valid GNSS data is received
            lastGNSSDataMillis = millis();
            displayGNSSData(); // Display GNSS data for debugging
        }
    }

    // Perform initial synchronization using NMEA time data
    if (!initialSyncDone && gps.date.isValid() && gps.time.isValid())
    {
        setSystemTime();
        initialSyncDone = true;
        USBSerial.println("Initial time synchronization done using NMEA data.");
    }

    // Disable interrupts to safely check and reset the PPS flag
    noInterrupts();
    if (ppsFlag)
    {
        fineTuneSystemTime(); // Adjust system time based on the PPS pulse
        ppsFlag = false;
    }
    // Re-enable interrupts
    interrupts();

    // Check if GNSS data has been absent for more than a minute
    if (millis() - lastGNSSDataMillis > 60000)
    {
        USBSerial.println("Warning: Haven't received GNSS data for more than 1 minute!");
        // Additional actions can be added here, like alerts or module resets.
    }
}

// Interrupt handler for the PPS signal
void ppsInterrupt() { ppsFlag = true; }

// Function to set system time using GNSS data
void setSystemTime()
{
    struct tm timeinfo;
    timeinfo.tm_year = gps.date.year() - 1900;
    timeinfo.tm_mon = gps.date.month() - 1;
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();
    time_t t = mktime(&timeinfo);

    timeval tv = {t, 0};
    settimeofday(&tv, NULL); // Update system time
}

// Function to fine-tune system time using the PPS pulse
void fineTuneSystemTime()
{
    timeval tv;
    gettimeofday(&tv, NULL);
    tv.tv_usec = 0;          // Reset microseconds to zero
    settimeofday(&tv, NULL); // Update system time
    USBSerial.println("System time fine-tuned using PPS signal.");
}

// Debugging function to display GNSS data
void displayGNSSData()
{
    USBSerial.print("Latitude: ");
    USBSerial.println(gps.location.lat(), 6);
    USBSerial.print("Longitude: ");
    USBSerial.println(gps.location.lng(), 6);
    USBSerial.print("Altitude: ");
    USBSerial.println(gps.altitude.meters());
    USBSerial.print("Speed: ");
    USBSerial.println(gps.speed.kmph());
    USBSerial.println("-----------------------------");
}
