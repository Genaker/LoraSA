/**
  RadioLib SX126x Spectrum Scan

  This code perform a spectrum power scan using SX126x.
  The output is in the form of scan lines, each line has 33 power bins.
  First power bin corresponds to -11 dBm, the second to -15 dBm and so on.
  Higher number of samples in a bin corresponds to more power received
  at that level.

  To show the results in a plot, run the Python script
  RadioLib/extras/SX126x_Spectrum_Scan/SpectrumScan.py

  WARNING: This functionality is experimental and requires a binary patch
  to be uploaded to the SX126x device. There may be some undocumented
  side effects!

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

//  #define HELTEC_NO_DISPLAY

#ifdef BT_MOBILE
#include <NimBLEDevice.h>

#define SERVICE_UUID "00001234-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "00001234-0000-1000-8000-00805f9b34ac"

NimBLEServer *pServer = nullptr;
NimBLECharacteristic *pCharacteristic = nullptr;
NimBLEAdvertising *pAdvertising = nullptr;

void initBT()
{
    // Initialize BLE device
    NimBLEDevice::init("RSSI_Radar");

    // Get and print the MAC address
    String macAddress = NimBLEDevice::getAddress().toString().c_str();
    Serial.println("Bluetooth MAC Address: " + macAddress);

    // Create BLE server
    pServer = NimBLEDevice::createServer();

    // Create a BLE service
    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    // Start the service
    pService->start();

    // Start advertising

    pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setName("ESP32_RSSI_Radar"); // Set the device name
    pAdvertising->setMinInterval(300);
    pAdvertising->setMaxInterval(350);
    // pAdvertising->setScanResponse(true); // Allow scan responses

    pAdvertising->start();

    Serial.println("BLE server started.");
}

// Function to send RSSI and Heading Data
void sendBTData(float heading, float rssi)
{
    String data =
        "RSSI_HEADING: '{H:" + String(heading) + ",RSSI:-" + String(rssi) + "}'";
    Serial.println("Sending data: " + data);
    pCharacteristic->setValue(data.c_str()); // Set BLE characteristic value
    pCharacteristic->notify();               // Notify connected client
}

#endif

#include "FS.h"
#include <Arduino.h>
#ifdef WEB_SERVER
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif
#include <File.h>
#include <LittleFS.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unordered_map>
#include <unordered_set>

#include "WIFI_SERVER.h"

#define FORMAT_LITTLEFS_IF_FAILED true

// HardwareSerial Serial0(0);

// #define OSD_ENABLED true
// #define WIFI_SCANNING_ENABLED true
// #define BT_SCANNING_ENABLED true

// Direct access to the low-level SPI communication between RadioLib and the radio module.
#define RADIOLIB_LOW_LEVEL (1)
//  In this mode, all methods and member variables of all RadioLib classes will be made
//  public and so will be exposed to the user. This allows direct manipulation of the
//  library internals.
#define RADIOLIB_GODMODE (1)
#define RADIOLIB_CHECK_PARAMS (0)

#include <charts.h>
#include <comms.h>
#include <config.h>
#include <events.h>
#include <scan.h>
#include <stdlib.h>

#ifndef LILYGO
#include <heltec_unofficial.h>
// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"
#endif // end ifndef LILYGO

#if defined(LILYGO)
// LiLyGO device does not support the auto download mode, you need to get into the
// download mode manually. To do so, press and hold the BOOT button and then press the
// RESET button once. After that release the BOOT button. Or OFF->ON together with BOOT

// Default LilyGO code
#include <LoRaBoards.h>

// #include "utilities.h"
//  Our Code
#include <LiLyGo.h>
#endif // end LILYGO

#include <bus.h>
#include <heading.h>
#include <radio.h>

DroneHeading droneHeading;
Compass *compass = NULL;

RadioModule *radio2;

#define BT_SCAN_DELAY 60 * 1 * 1000
#define WF_SCAN_DELAY 60 * 2 * 1000
long noDevicesMillis = 0, cycleCnt = 0;
bool present = false;
bool scanFinished = true;

bool radioIsScan = false;

// time to scan BT
#define BT_SCAN_TIME 10

uint64_t wf_start = 0;
uint64_t bt_start = 0;

#define MAX_POWER_LEVELS 33
unsigned int osdCyclesCount = 0;
#define OSD_MAX_CLEAR_CYCLES 10
#ifdef OSD_ENABLED
#include "DFRobot_OSD.h"
#ifndef SIDEBAR_START_ROW
#define SIDEBAR_START_ROW 1
#endif
#ifndef SIDEBAR_END_ROW
#define SIDEBAR_END_ROW 10
#endif
#ifndef OSD_CHART_START_ROW
#define OSD_CHART_START_ROW 14
#endif
// Set SIDEBAR_POSITION to 1 to right side
#define SIDEBAR_POSITION OSD_WIDTH - 7
// #define SIDEBAR_ACCENT_ONLY 1
#define SIDEBAR_DB_LEVEL 80 // Absolute value without minus
#define SIDEBAR_DB_DELTA 2  // detect changes <> threshold

#ifdef LILYGO
#define OSD_SCK 38
#define OSD_CS 39
#define OSD_MISO 40
#define OSD_MOSI 41
#else
// SPI pins
#define OSD_SCK 26
#define OSD_CS 47
#define OSD_MISO 33
#define OSD_MOSI 34
#endif

#endif // End OSD_ENABLED

#define OSD_WIDTH 30
#define OSD_HEIGHT 16
#define OSD_CHART_WIDTH 15
#define OSD_CHART_HEIGHT 5
#define OSD_X_START 1
#define OSD_Y_START 16

#ifndef OSD_CHART_START_COL
#define OSD_CHART_START_COL 2
#endif

// TODO: Calculate dynamically:
// osd_steps = osd_mhz_in_bin / (FM range / LORA radio x Steps)
int osd_mhz_in_bin = 5;
int osd_steps = 12;
int global_counter = 0;

std::unordered_map<int, bool> accentFreq = {{950, true}, {915, true}};
std::unordered_map<int, int> freqX = {};
int accentSize = accentFreq.size();
int indexAccent = 0;
int rowDebug = 0;

#ifdef OSD_ENABLED
DFRobot_OSD osd(OSD_CS);
#endif

#ifndef OSD_BAR_CHAR
#define OSD_BAR_CHAR "."
#endif

#include "global_config.h"
#include "ui.h"

#ifdef COMPASS_ENABLED
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
void displaySensorDetails(void)
{
    sensor_t sensor;
    mag.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       ");
    Serial.println(sensor.name);
    Serial.print("Driver Ver:   ");
    Serial.println(sensor.version);
    Serial.print("Unique ID:    ");
    Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    ");
    Serial.print(sensor.max_value);
    Serial.println(" uT");
    Serial.print("Min Value:    ");
    Serial.print(sensor.min_value);
    Serial.println(" uT");
    Serial.print("Resolution:   ");
    Serial.print(sensor.resolution);
    Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

// Variables for dynamic calibration
float x_min = 1000, x_max = -1000;
float y_min = 1000, y_max = -1000;
float z_min = 1000, z_max = -1000;
#endif
// -----------------------------------------------------------------
// CONFIGURATION OPTIONS
// -----------------------------------------------------------------

typedef enum
{
    METHOD_RSSI = 0u,
    METHOD_SPECTRAL
} TSCAN_METOD_ENUM;

// #define SCAN_METHOD METHOD_SPECTRAL

#define SCAN_METHOD
// #define METHOD_SPECTRAL // Spectral scan method
#define METHOD_RSSI // Uncomment this and comment METHOD_SPECTRAL fot RSSI

// Output Pixel Formula
// 1 = rssi / 4, 2 = (rssi / 2) - 22 or 20
// constexpr int RSSI_OUTPUT_FORMULA = 2;

// Feature to scan diapasones. Other frequency settings will be ignored.
// String SCAN_RANGES = String("850..890,920..950");
String SCAN_RANGES = "";

std::unordered_set<int> ignoredFreq = {/*{916, true}, {915, true}*/};
std::unordered_set<int> frAlwaysShow = {915};

size_t scan_pages_sz = 0;
ScanPage *scan_pages;
size_t scan_page = 0;

struct RSSIData
{
    short rssi;
    unsigned long int time;
};

std::unordered_map<unsigned int, RSSIData> historyRSSI;

// MHZ per page
// to put everything into one page set RANGE_PER_PAGE = FREQ_END - 800
uint64_t RANGE_PER_PAGE; // FREQ_END - CONF_FREQ_BEGIN

uint64_t CONF_FREQ_END, CONF_FREQ_BEGIN; // To Enable Multi Screen scan
//  uint64_t RANGE_PER_PAGE = 50;
//  Default Range on Menu Button Switch

// multiplies STEPS * N to increase scan resolution.
#if !defined(SCAN_RBW_FACTOR)
#define SCAN_RBW_FACTOR 2
#endif

#ifdef USING_SX1280PA
#define SCAN_RBW_FACTOR 2
#endif

constexpr int OSD_PIXELS_PER_CHAR = (STEPS * SCAN_RBW_FACTOR) / OSD_CHART_WIDTH;

#define DEFAULT_RANGE_PER_PAGE 50

// Print spectrum values pixels at once or by line
bool ANIMATED_RELOAD = false;

// TODO: Ignore max power lines
#define UP_FILTER 5
// Trim low signals - nose level
#define START_LOW 6
#define FILTER_SAMPLES_MIN
constexpr bool DRAW_DETECTION_TICKS = true;
int16_t max_x_rssi[STEPS] = {999};
int16_t max_x_window[STEPS / 14] = {999};
int16_t xRSSI[STEPS];
int x_window = 0;
constexpr int WINDOW_SIZE = 15;
// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
// if more than 100 it can freeze
#define SAMPLES 35 //(scan time = 1294)

uint64_t RANGE, range, iterations, median_frequency;
float SINGLE_STEP;

// #define DISABLE_PLOT_CHART false   // unused

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

bool filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

int max_bins_array_value[MAX_POWER_LEVELS];

std::unordered_map<int, int> prevDBvalue;
int max_step_range = 32;

bool detected_y[STEPS]; // 20 - ??? steps

// global variable

// Used as a Led Light and Buzzer/count trigger
bool first_run, new_pixel, detected_x = false;
uint64_t drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL;
#define TRIGGER_LEVEL -80.0
uint64_t drone_detected_frequency_start = 0;
uint64_t drone_detected_frequency_end = 0;
bool single_page_scan = false;

float vbat = 0;

// #define PRINT_DEBUG

#define PRINT_PROFILE_TIME
#ifdef PRINT_PROFILE_TIME
uint64_t loop_start = 0;
uint64_t loop_time = 0;
uint64_t scan_time = 0;
uint64_t scan_start_time = 0;
#endif

// log data via serial console, JSON format:
// Optionally it can be enabled via this flag, although its recommended to use
// platformio config flag -DLOG_DATA_JSON
// #define LOG_DATA_JSON true

#ifdef SEEK_ON_X
#define SERIAL_PORT 1

HardwareSerial SerialPort(SERIAL_PORT);
#endif

// #define WEB_SERVER true

uint64_t x, y, w = WATERFALL_START, i = 0;
int osd_x = 1, osd_y = 2, col = 0, colOSD = OSD_CHART_START_COL, max_bin = 32;

int rssi = 0;
int state = 0;

int CONF_SAMPLES;
#ifdef METHOD_SPECTRAL
int samples = SAMPLES;
#endif
#ifdef METHOD_RSSI
int samples = SAMPLES_RSSI;
#endif

uint8_t result_index = 0;
uint8_t button_pressed_counter = 0;

#ifndef LILYGO
// #define JOYSTICK_ENABLED
#endif

#include "joyStick.h"

// project components
#if (defined(WIFI_SCANNING_ENABLED) || defined(BT_SCANNING_ENABLED)) &&                  \
    defined(OSD_ENABLED)
#include "BT_WIFI_scan.h"
#endif

#if defined(WIFI_SCANNING_ENABLED) && defined(OSD_ENABLED)
scanWiFi(osd)
#endif

#if defined(BT_SCANNING_ENABLED) && defined(OSD_ENABLED)
    scanBT(osd)
#endif

#ifdef OSD_ENABLED
        unsigned short selectFreqChar(int bin, int start_level = 0)
{
    if (bin >= start_level)
    {
        // level when we are starting show levels symbols
        // you can override with your own character for example 0x100 = " " empty char
        return power_level[33];
    }
    else if (bin >= 0 && bin < MAX_POWER_LEVELS)
        return power_level[bin];
    // when wrong bin number or noc har assigned we are showing "!" char
    return 0x121;
}

void osdPrintSignalLevelChart(int col, int signal_value)
{
#ifdef METHOD_RSSI
    signal_value = (signal_value / 6);
#endif
    int barSize = 5;
    int maxLevel = 30;
    int dbPerChar = (maxLevel - drone_detection_level) / 5;
    // signal_value = abs(signal_value);

#ifdef OSD_DOT_DISPLAY

    if (signal_value <= drone_detection_level)
    {
        for (int i = 0; i <= barSize; i++)
        {
            if (i < (drone_detection_level - signal_value) / dbPerChar)
            {
                osd.displayString(OSD_CHART_START_ROW - i, col, OSD_BAR_CHAR);
                // osd.displayString(5, col, "s:" + String(signal_value));
                // osd.displayString(
                //   6, col, String((drone_detection_level - signal_value) / dbPerChar));
            }
            else
            {
                osd.displayString(OSD_CHART_START_ROW - i, col, " ");
                // osd.displayString(15 - i - 1, col, " ");
                // break;
            }
        }
    }
    else
    {
        for (int i = 0; i <= barSize; i++)
        {
            // Clear bar
            osd.displayString(OSD_CHART_START_ROW - i, col, " ");
        }
    }

#else

    // Third line
    if (signal_value <= 9 && signal_value <= drone_detection_level)
    {
        osd.displayString(5, col, "s:" + String(signal_value));
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(12, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
    // Second line
    else if (signal_value < 19 && signal_value <= drone_detection_level)
    {
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(13, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
    // First line
    else
    {
        // Clean Up symbol
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, selectFreqChar(signal_value, drone_detection_level));
    }
#endif
}

// Start Sidebar
int sideBarRow = SIDEBAR_START_ROW;

void osdProcess()
{ // OSD enabled
    osdCyclesCount++;
    // memset(max_step_range, 33, 30);
    max_bin = 32;

    osd.displayString(12, OSD_CHART_START_COL, String(CONF_FREQ_BEGIN));
    osd.displayString(12, OSD_WIDTH - 10 + OSD_CHART_START_COL, String(CONF_FREQ_END));
    // Finding biggest in result
    //  Skiping 0 and 32 31 to avoid overflow
    for (int i = 1; i < MAX_POWER_LEVELS - 3; i++)
    {
        // filter
        if (result[i] > 0
#if FILTER_SPECTRUM_RESULTS
            && ((result[i + 1] != 0 /*&& result[i + 2] != 0*/) || result[i - 1] != 0)
#endif
        )
        {
            max_bin = i;
#ifdef PRINT_DEBUG
            Serial.print("MAX in bin:" + String(max_bin));
            Serial.println();
#endif
            break;
        }
    }
    // max_bin contains fist not 0 index of the bin
    if (max_step_range > max_bin && max_bin != 0)
    {
        max_step_range = max_bin;
// Store RSSI value for RSSI Method
#ifdef METHOD_RSSI
        // set to the min RSSI value
        max_step_range = 120;
        if (result[max_bin] != 0)
        {
            max_step_range = result[max_bin];
            // osd.displayString(rowDebug++, 0, String(max_step_range));
            // sleep(1);
        }
#endif
    }
    // Going to the next OSD step
    if (x % osd_steps == 0 && col < OSD_WIDTH)
    {
        // OSD SIDE BAR with frequency log
#ifdef OSD_SIDE_BAR
        {
#ifndef METHOD_RSSI
            if (max_step_range < 16)
            {
                osd.displayString(sideBarRow++, OSD_WIDTH - 7,
                                  String(CONF_FREQ_BEGIN + (col * osd_mhz_in_bin)) + "-" +
                                      String(max_step_range) + " ");
            }
#endif

#ifdef METHOD_RSSI
            String noChangeChar = "=";
            String upChar = ">";
            String downChar = "<";
            String printChar = "-";

            int freqOSD = CONF_FREQ_BEGIN + (col * osd_mhz_in_bin);
            String freqOSDString = "";

            if ((
// Show defined freq only
#ifndef SIDEBAR_ACCENT_ONLY
                    max_step_range <= SIDEBAR_DB_LEVEL
#else
                    false
#endif
                    || accentFreq[freqOSD] == true) &&
                max_step_range > 0)
            {
                if (prevDBvalue[freqOSD] != 0 &&
                    // 80 - 90
                    prevDBvalue[freqOSD] - max_step_range < -SIDEBAR_DB_DELTA)
                {
                    printChar = upChar;
                }
                else if (prevDBvalue[freqOSD] != 0 &&
                         // 90 - 80
                         prevDBvalue[freqOSD] - max_step_range > SIDEBAR_DB_DELTA)
                {
                    printChar = downChar;
                }
                else if (prevDBvalue[freqOSD] != 0 &&
                         // 90 - 92 && 90 - 88
                         abs(prevDBvalue[freqOSD] - max_step_range) < SIDEBAR_DB_DELTA)
                {
                    printChar = noChangeChar;
                }
                else
                {
                    printChar = printChar;
                }

                if (true || sideBarRow == SIDEBAR_START_ROW || freqOSD % 100 == 0)
                {
                    freqOSDString = String(freqOSD);
                }
                else
                {
                    freqOSDString = String(" ") + String(freqOSD).substring(1);
                }

                long int osd_start = millis();
                if (accentFreq[freqOSD] == false)
                {
                    osd.displayString(accentSize + sideBarRow++, SIDEBAR_POSITION,
                                      freqOSDString + printChar + String(max_step_range) +
                                          " ");
                }
                else
                {
                    osd.displayString(SIDEBAR_START_ROW + indexAccent, SIDEBAR_POSITION,
                                      freqOSDString + printChar + String(max_step_range) +
                                          " ");
                    indexAccent++;
                }
                long int osd_end = millis();
                // Serial.println("Time of EXecution: " + String((osd_end - osd_start)));
                prevDBvalue[freqOSD] = max_step_range;
            }
            else
            {
                // erase values without previous every N of the OSD cycle
                if (osdCyclesCount >= OSD_MAX_CLEAR_CYCLES)
                {
                    prevDBvalue[freqOSD] = 0;
                    // Clear sidebar 3 first values always on screen
                    for (int i = sideBarRow + 3; i <= SIDEBAR_END_ROW; i++)
                    {
                        osd.displayString(i++, SIDEBAR_POSITION, String("       "));
                    }
                    osdCyclesCount = 0;
                }
            }
            // MAx down sidebar
            if (sideBarRow >= SIDEBAR_END_ROW)
            {
                sideBarRow = SIDEBAR_START_ROW;
            }

#endif
        }
#endif
        // Test with Random Result...
        // max_step_range = rand() % 32;
#ifdef METHOD_RSSI
        // With THe RSSI method we can get real RSSI value not just a bin
#endif
        if (max_step_range > 0)
        {
            // PRINT SIGNAL CHAR ROW, COL, VALUE
            osdPrintSignalLevelChart(colOSD, max_step_range);
        }

#ifdef PRINT_DEBUG
        Serial.println("MAX:" + String(max_step_range));
#endif
        max_step_range = 32;
#ifdef METHOD_RSSI
        max_step_range = 120;
#endif
        colOSD++;
        col++;
    }
    if (osdCyclesCount >= OSD_MAX_CLEAR_CYCLES)
    {
        osdCyclesCount = 0;
    }
}
#endif

Config config;

#ifdef USING_LR1121
void setLRFreq(float freq)
{
    bool skipCalibration = false;
    /**
   if(!(((freq >= 150.0) && (freq <= 960.0)) ||
     ((freq >= 1900.0) && (freq <= 2200.0)) ||
     ((freq >= 2400.0) && (freq <= 2500.0)))) {
       return(RADIOLIB_ERR_INVALID_FREQUENCY);
   }**/

    // check if we need to recalibrate image
    // int16_t state;

    if (!true && (fabsf(freq - radio.freqMHz) >= RADIOLIB_LR11X0_CAL_IMG_FREQ_TRIG_MHZ))
    {
        state = radio.calibrateImageRejection(freq - 4, freq + 4);
        // RADIOLIB_ASSERT(state);
    }

    // set frequency
    state = radio.setRfFrequency((uint32_t)(freq * 1000000.0f));
    // RADIOLIB_ASSERT(state);
    radio.freqMHz = freq;
    radio.highFreq = (freq > 1000.0);
}
#endif

float getRSSI(void *param)
{
    Scan *r = (Scan *)param;
#if defined(USING_SX1280PA)
    // TODO: TEST new feature
    radio.getRSSI(false);
#elif defined(USING_LR1121)
    // Try getRssiInst
    float rssi;
    radio.getRssiInst(&rssi);
    // Serial.println("RSSI: " + String(rssi));
    // pass the replies
    return rssi;
#else
    return radio.getRSSI(false);
#endif
}

float getCAD(void *param)
{
    Scan *r = (Scan *)param;

    int16_t err = radio.scanChannel();
    if (err != RADIOLIB_ERR_NONE)
    {
        return -999;
    }

#ifdef USING_LR1121
    // LR1121 doesn't implement getRSSI(bool), getRSSI always
    // returns RSSI of the last packet
    return radio.getRSSI();
#else
    return radio.getRSSI(true);
#endif
}

Scan r;

#define WATERFALL_SENSITIVITY 0.05
DecoratedBarChart *bar;
WaterfallChart *waterChart;
StackedChart stacked(display, 0, 0, 0, 0);

UptimeClock *uptime;

int16_t initForScan(float freq)
{
    int16_t state;

#if defined(USING_SX1280PA)
    state = radio.beginGFSK(freq);
#elif defined(USING_LR1121)
    state = radio.beginGFSK(freq, 4.8F, 5.0F, 156.2F, 10, 16U, 1.6F);
    // RF Switch info Provided by LilyGo support:
    // https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/f2d3d995cba03c65a7031c73e212f106b03c95a2/examples/RadioLibExamples/Receive_Interrupt/Receive_Interrupt.ino#L279

    // LR1121
    // set RF switch configuration for Wio WM1110
    // Wio WM1110 uses DIO5 and DIO6 for RF switching
    static const uint32_t rfswitch_dio_pins[] = {RADIOLIB_LR11X0_DIO5,
                                                 RADIOLIB_LR11X0_DIO6, RADIOLIB_NC,
                                                 RADIOLIB_NC, RADIOLIB_NC};

    static const Module::RfSwitchMode_t rfswitch_table[] = {
        // mode                  DIO5  DIO6
        {LR11x0::MODE_STBY, {LOW, LOW}},  {LR11x0::MODE_RX, {HIGH, LOW}},
        {LR11x0::MODE_TX, {LOW, HIGH}},   {LR11x0::MODE_TX_HP, {LOW, HIGH}},
        {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
        {LR11x0::MODE_WIFI, {LOW, LOW}},  END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // LR1121 TCXO Voltage 2.85~3.15V
    radio.setTCXO(3.0);
    delay(1000);
#else
    state = radio.beginFSK(freq);
#endif

    int gotoAcounter = 0;
A:
#ifdef METHOD_RSSI
    // TODO: try RADIOLIB_SX126X_RX_TIMEOUT_INF
#ifdef USING_SX1280PA
    state = radio.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_NONE);
#elif USING_LR1121
    state = radio.startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_NONE);
#else
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
#endif

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        display.drawString(0, 64 - 10, "E:startReceive");
        display.display();
        delay(2000);
        Serial.println(state);
        gotoAcounter++;
        if (gotoAcounter < 5)
        {
            goto A;
        }
    }

#endif

    return state;
}

bool setFrequency(float curr_freq)
{
    r.current_frequency = curr_freq;
    LOG("setFrequency:%f\n", r.current_frequency);
    // Serial.println("setFrequency:" + String(curr_freq));

    int16_t state;
#ifdef USING_SX1280PA
    int16_t state1 =
        radio.setFrequency(r.current_frequency); // 1280 doesn't have calibration

    state = radio.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_INF);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:startReceive:" + String(state));
    }

    state = state1;
#elif USING_SX1276
    state = radio.setFrequency(r.current_frequency);
#elif USING_LR1121
    // state = radio.setRfFrequency((uint32_t)(r.current_frequency * 1000000.0f));
    //  TODO: make calibration, DONE!!
    //  ToDO: check how RF switch works continues scanning when init on low doesn't work
    //  for high freq
    setLRFreq(r.current_frequency);
#else
    state = radio.setFrequency(r.current_frequency,
                               true); // false = calibration is needed here
#endif
    if (state != RADIOLIB_ERR_NONE)
    {
        display.drawString(0, 64 - 10,
                           "E(" + String(state) +
                               "):setFrequency:" + String(r.current_frequency));
        Serial.println("E(" + String(state) +
                       "):setFrequency:" + String(r.current_frequency));
        display.display();
        delay(2);
        return false;
    }

    return true;
}

void init_radio()
{
    // initialize SX1262 FSK modem at the initial frequency
    both.println("Init radio");
#ifndef INIT_FREQ
    state = initForScan(CONF_FREQ_BEGIN);
#else
    state = initForScan(INIT_FREQ);
#endif
    if (state == RADIOLIB_ERR_NONE)
    {
        radioIsScan = true;
        Serial.println(F("success!"));
    }
    else
    {
        display.println("Error:" + String(state));
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(5);
        }
    }

#ifdef METHOD_SPECTRAL
    // upload a patch to the SX1262 to enable spectral scan
    // NOTE: this patch is uploaded into volatile memory,
    // and must be re-uploaded on every power up
    both.println("Upload SX1262 patch");

    // Upload binary patch into the SX126x device RAM. Patch is needed to e.g.,
    // enable spectral scan and must be uploaded again on every power cycle.
    RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
    // configure scan bandwidth and disable the data shaping
#endif

    both.println("Setting up radio");
#ifdef USING_SX1280PA
    // RADIOLIB_OR_HALT(radio.setBandwidth(RADIOLIB_SX128X_LORA_BW_406_25));
#elif USING_SX1276
    // 	Receiver bandwidth in kHz. Allowed values
    // are 2.6, 3.1, 3.9, 5.2, 6.3, 7.8, 10.4, 12.5, 15.6, 20.8, 25, 31.3, 41.7,
    // 50, 62.5, 83.3, 100, 125, 166.7, 200 and 250 kHz.
    RADIOLIB_OR_HALT(radio.setRxBandwidth(250));
#else
    RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));
#endif

    // and disable the data shaping
    state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:setDataShaping:" + String(state));
    }
    both.println("Starting scanning...");

    // calibrate only once ,,, at startup
    // TODO: check documentation (9.2.1) if we must calibrate in certain ranges
    setFrequency(CONF_FREQ_BEGIN);

    delay(100);

#ifdef USING_SX1262
    if (config.radio2.enabled && config.radio2.module.equalsIgnoreCase("SX1262"))
    {
        radio2 = new SX1262Module(config.radio2);
        state = radio2->beginScan(CONF_FREQ_BEGIN, BANDWIDTH, RADIOLIB_SHAPING_NONE);
        if (state == RADIOLIB_ERR_NONE)
        {
            both.println("Initialized additional module OK");
            radio2->setRxBandwidth(BANDWIDTH);
        }
        else
        {
            Serial.printf("Error initializing additional module: %d\n", state);
            if (state == RADIOLIB_ERR_CHIP_NOT_FOUND)
            {
                Serial.println("Radio2: CHIP NOT FOUND");
            }
        }
    }
#endif
}

struct frequency_scan_result
{
    uint64_t begin;
    uint64_t end;
    uint64_t last_epoch;
    int16_t rssi; // deliberately not a float; floats can pin task to wrong core forever

    ScanTaskResult dump;
    size_t readings_sz;
} frequency_scan_result;

TaskHandle_t logToSerial = NULL;
TaskHandle_t dumpToComms = NULL;

void eventListenerForReport(void *arg, Event &e)
{
    if (e.type == EventType::DETECTED)
    {
        if (e.epoch != frequency_scan_result.last_epoch)
        {
            frequency_scan_result.dump.sz = 0;
        }

        if (frequency_scan_result.dump.sz >= frequency_scan_result.readings_sz)
        {
            size_t old_sz = frequency_scan_result.readings_sz;
            frequency_scan_result.readings_sz = frequency_scan_result.dump.sz + 1;
            uint32_t *f = new uint32_t[frequency_scan_result.readings_sz];
            int16_t *r = new int16_t[frequency_scan_result.readings_sz];
            int16_t *r2 = radio2 ? new int16_t[frequency_scan_result.readings_sz] : NULL;

            if (old_sz > 0)
            {
                memcpy(f, frequency_scan_result.dump.freqs_khz,
                       old_sz * sizeof(uint32_t));
                memcpy(r, frequency_scan_result.dump.rssis, old_sz * sizeof(int16_t));
                if (radio2)
                {
                    memcpy(r2, frequency_scan_result.dump.rssis2,
                           old_sz * sizeof(int16_t));
                    delete[] frequency_scan_result.dump.rssis2;
                }

                delete[] frequency_scan_result.dump.freqs_khz;
                delete[] frequency_scan_result.dump.rssis;
            }

            frequency_scan_result.dump.freqs_khz = f;
            frequency_scan_result.dump.rssis = r;
            frequency_scan_result.dump.rssis2 = r2;
        }

        frequency_scan_result.dump.freqs_khz[frequency_scan_result.dump.sz] =
            e.detected.freq * 1000; // convert to kHz
        frequency_scan_result.dump.rssis[frequency_scan_result.dump.sz] =
            max(e.detected.rssi, -999.0f);
        if (radio2)
            frequency_scan_result.dump.rssis2[frequency_scan_result.dump.sz] =
                max(e.detected.rssi2, -999.0f);
        frequency_scan_result.dump.sz++;

        if (e.epoch != frequency_scan_result.last_epoch ||
            e.detected.rssi > frequency_scan_result.rssi)
        {
            frequency_scan_result.last_epoch = e.epoch;
            frequency_scan_result.rssi = e.detected.rssi;
        }

        return;
    }

    if (e.type == EventType::SCAN_TASK_COMPLETE)
    {
        // notify async communication that the data is ready
        if (logToSerial != NULL)
        {
            xTaskNotifyGive(logToSerial);
        }

        if (dumpToComms != NULL)
        {
            xTaskNotifyGive(dumpToComms);
        }
        return;
    }
}

ScanTask report_scans = ScanTask{
    count : 0, // 0 => report none; < 0 => report forever; > 0 => report that many
    delay : 0  // 0 => as and when it happens; > 0 => at least once that many ms
};

bool requested_host = true;

void dumpToCommsTask(void *parameter)
{
    uint64_t last_epoch = frequency_scan_result.last_epoch;

    for (;;)
    {
        int64_t delay = report_scans.delay;
        if (delay == 0)
        {
            delay = (1ull << 63) - 1;
        }

        ulTaskNotifyTake(true, pdMS_TO_TICKS(delay));
        if (report_scans.count == 0 || frequency_scan_result.last_epoch == last_epoch)
        {
            continue;
        }

        if (report_scans.count > 0)
        {
            report_scans.count--;
        }

        Message m;
        m.type = MessageType::SCAN_RESULT;
        m.payload.dump = frequency_scan_result.dump;
        if (requested_host)
        {
            HostComms->send(m);
        }
        else
        {
            if (Comms0 != NULL)
                Comms0->send(m);
            if (Comms1 != NULL)
                Comms1->send(m);
        }

        m.payload.dump.sz =
            0; // dump is shared, so should not delete arrays in destructor
    }
}

#ifdef LOG_DATA_JSON
void logToSerialTask(void *parameter)
{
    uint64_t last_epoch = frequency_scan_result.last_epoch;
    frequency_scan_result.rssi = -999;

    for (;;)
    {
        ulTaskNotifyTake(true, pdMS_TO_TICKS(config.log_data_json_interval));
        if (frequency_scan_result.begin != frequency_scan_result.end ||
            frequency_scan_result.last_epoch != last_epoch)
        {
            int16_t highest_value_scanned = frequency_scan_result.rssi;
            frequency_scan_result.rssi = -999;
            last_epoch = frequency_scan_result.last_epoch;
            if (highest_value_scanned == -999)
            {
                continue;
            }

#ifdef SEEK_ON_X
            SerialPort.printf("{\"low_range_freq\": %" PRIu64
                              ", \"high_range_freq\": %" PRIu64 ", "
                              "\"value\": \"%" PRIi16 "\"}\n",
                              frequency_scan_result.begin, frequency_scan_result.end,
                              highest_value_scanned);
#else
            Serial.printf("{\"low_range_freq\": %" PRIu64
                          ", \"high_range_freq\": %" PRIu64 ", "
                          "\"value\": \"%" PRIi16 "\"}\n",
                          frequency_scan_result.begin, frequency_scan_result.end,
                          highest_value_scanned);
#endif
        }
    }
}
#endif

void drone_sound_alarm(void *arg, Event &e);

void draw360Scale(int start = 0, int end = 360, int width = 128, int height = 64)
{
    float scale = 360 / width;
    int scaleLength = width; // Full width of the display
    int step = 90;           // Step size for labels (adjust for readability)
    int stepPixel = width / (end / step);
    int scaleY = (height / 2) + 15; // Vertical center for the scale

    // Draw the scale line
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawLine(0, scaleY, width, scaleY);

    // Add ticks and labels
    // 0   32    64   96  128 px
    // 0   90   180  270  360 degree
    // |____|____|____|_____|
    int n = 0;
    for (int x = 0; x <= width; x += stepPixel)
    {
        // int x = map(i, start, end, 0, scaleLength);
        Serial.println("Tick: " + String(x));
        if (x == 128)
        {
            x = x - 1;
        }
        // Draw tick mark
        display.drawVerticalLine(x, scaleY - 5, 10);
        // smaller tick
        display.drawVerticalLine(x + (int)(stepPixel / 2), scaleY - 3, 7);

        int degreeLegend = n * step;
        // Draw label hj
        // display.setFont(u8g2_font_6x10_tf); // Choose a readable font
        display.drawString(x, scaleY + 5, String(degreeLegend));
        n++;
    }

    // Optional: Add start and end labels
    display.drawString(0, scaleY + 5, String(start));
    display.display();
    // display.drawString(width - 20, scaleY + 15, String(end));
}

void configurePages()
{
    if (scan_pages_sz > 0)
        delete[] scan_pages;

    if (single_page_scan)
    {
        scan_pages_sz = 1;
        ScanPage scan_page = {
            start_mhz : CONF_FREQ_BEGIN,
            end_mhz : CONF_FREQ_END,
            page_sz : config.scan_ranges_sz
        };
        if (scan_page.page_sz > 0)
        {
            scan_page.scan_ranges = new ScanRange[scan_page.page_sz];
        }
        for (int i = 0; i < scan_page.page_sz; i++)
        {
            scan_page.scan_ranges[i] = config.scan_ranges[i];
        }
        scan_pages = new ScanPage[1]{scan_page};
        scan_page.page_sz =
            0; // make sure it doesn't free up the Scanranges that were just constructed
    }
    else
    {
        scan_pages_sz =
            (CONF_FREQ_END - CONF_FREQ_BEGIN + RANGE_PER_PAGE - 1) / RANGE_PER_PAGE;
        scan_pages = new ScanPage[scan_pages_sz];
        for (int j = 0; j < scan_pages_sz; j++)
        {
            ScanPage scan_page = {
                start_mhz : CONF_FREQ_BEGIN + j * RANGE_PER_PAGE,
                end_mhz : CONF_FREQ_BEGIN + (j + 1) * RANGE_PER_PAGE,
                page_sz : 0
            };
            for (int i = 0; i < config.scan_ranges_sz; i++)
            {
                if (config.scan_ranges[i].start_khz > scan_page.end_mhz * 1000 ||
                    config.scan_ranges[i].end_khz < scan_page.start_mhz * 1000)
                {
                    continue;
                }
                scan_page.page_sz++;
            }

            if (scan_page.page_sz > 0)
            {
                scan_page.scan_ranges = new ScanRange[scan_page.page_sz];
                for (int i = 0, r = 0; i < config.scan_ranges_sz; i++)
                {
                    if (config.scan_ranges[i].start_khz > scan_page.end_mhz * 1000 ||
                        config.scan_ranges[i].end_khz < scan_page.start_mhz * 1000)
                    {
                        continue;
                    }

                    scan_page.scan_ranges[r] = {
                        start_khz : max(config.scan_ranges[i].start_khz,
                                        scan_page.start_mhz * 1000),
                        end_khz :
                            min(config.scan_ranges[i].end_khz, scan_page.end_mhz * 1000),
                        step_khz : config.scan_ranges[i].step_khz
                    };
                    r++;
                }
            }
            scan_pages[j] = scan_page;
            scan_page.page_sz =
                0; // we copied over the values, make sure the array doesn't get freed
        }
    }
}

void configureDetection()
{
    if (config.scan_ranges_sz == 0)
    {
        if (config.detection_strategy.equalsIgnoreCase("RSSI_MAX"))
        {
            size_t sz = 10;
            uint32_t f_khz = FREQ_BEGIN * 1000;
            uint32_t ssz = (FREQ_END - FREQ_BEGIN) * 1000 / 10;
            uint32_t step =
                (float)(FREQ_END - FREQ_BEGIN) * 1000 / (STEPS * SCAN_RBW_FACTOR);

            uint32_t rx_b = FREQ_END * 1000 + 100000;
            uint32_t rx_e = rx_b + 500;
            if (RxComms != NULL)
            {
                rx_e = RxComms->loraCfg.bw;
                rx_b = RxComms->loraCfg.freq * 1000 - rx_e;
                rx_e = rx_b + 2 * rx_e;
                if (rx_e / ssz == rx_b / ssz && rx_e > f_khz && rx_b < FREQ_END * 1000)
                {
                    // entire exclusion range is in one bucket
                    sz++;
                }
            }

            config.scan_ranges_sz = sz;
            config.scan_ranges = new ScanRange[sz];
            for (int i = 0; i < sz; i++)
            {
                config.scan_ranges[i].step_khz = step;
                config.scan_ranges[i].start_khz = f_khz > rx_b ? max(f_khz, rx_e) : f_khz;

                bool starts_before = f_khz < rx_e;
                bool ends_after = f_khz + ssz > rx_b;

                if (starts_before && ends_after)
                {
                    config.scan_ranges[i].end_khz = rx_b;
                    i++;
                    config.scan_ranges[i].start_khz = rx_e;
                    config.scan_ranges[i].step_khz = step;
                }

                f_khz += ssz;
                config.scan_ranges[i].end_khz =
                    f_khz - step < rx_e ? min(f_khz - step, rx_b) : f_khz - step;
            }

            if (config.scan_ranges[sz - 1].end_khz > rx_e)
                config.scan_ranges[sz - 1].end_khz = FREQ_END * 1000;
        }
        else
        {
            config.scan_ranges_sz = 1;
            config.scan_ranges = new ScanRange[1];
            config.scan_ranges[0].start_khz = FREQ_BEGIN * 1000;
            config.scan_ranges[0].end_khz = FREQ_END * 1000;
            config.scan_ranges[0].step_khz =
                (float)(FREQ_END - FREQ_BEGIN) * 1000 / (STEPS * SCAN_RBW_FACTOR);
        }
    }

    if (config.samples <= 0)
    {
        config.samples = SAMPLES_RSSI;
    }

    CONF_SAMPLES = config.samples;

    CONF_FREQ_BEGIN = config.scan_ranges[0].start_khz / 1000;
    CONF_FREQ_END = config.scan_ranges[0].end_khz / 1000;
    for (int i = 1; i < config.scan_ranges_sz; i++)
    {
        CONF_FREQ_BEGIN = min(CONF_FREQ_BEGIN, config.scan_ranges[i].start_khz / 1000);
        CONF_FREQ_END = max(CONF_FREQ_END, config.scan_ranges[i].end_khz / 1000);
    }

    median_frequency = (CONF_FREQ_BEGIN + CONF_FREQ_END) / 2;

    samples = CONF_SAMPLES;

    RANGE_PER_PAGE = CONF_FREQ_END - CONF_FREQ_BEGIN; // FREQ_END - CONF_FREQ_BEGIN
    RANGE = (int)(CONF_FREQ_END - CONF_FREQ_BEGIN);
    range = RANGE;

    configurePages();

    if (bar != NULL)
    {
        bar->bar.min_x = CONF_FREQ_BEGIN;
        bar->bar.max_x = CONF_FREQ_END;
    }
}

void readConfigFile()
{
    // writeFile(LittleFS, "/text.txt", "{WIFI:{name:\"sdfsdf\",
    // Password:\"sdfsdf\"}");
    ssid = readParameterFromParameterFile(SSID);
    Serial.println("SSID: " + ssid);

    pass = readParameterFromParameterFile(PASS);
    Serial.println("PASS: " + pass);

    ip = readParameterFromParameterFile(IP);
    Serial.println("PASS: " + ip);

    gateway = readParameterFromParameterFile(GATEWAY);
    Serial.println("GATEWAY: " + gateway);

    fstart = readParameterFromParameterFile(FSTART);
    Serial.println("FSTART: " + fstart);

    fend = readParameterFromParameterFile(FEND);
    Serial.println("FEND: " + fend);

    smpls = readParameterFromParameterFile("samples");
    Serial.println("SAMPLES: " + smpls);

    String detection = String("RSSI");
    if (smpls.length() > 0)
        detection += "," + smpls;
    if (fstart.length() == 0)
        fstart = String(FREQ_BEGIN * 1000);
    if (fend.length() == 0)
        fend = String(FREQ_END * 1000);

    detection += ":" + fstart + ".." + fend + "/" + String(STEPS * SCAN_RBW_FACTOR);

    config.configureDetectionStrategy(detection);
    configureDetection();

    both.println("C FREQ BEGIN:" + String(CONF_FREQ_BEGIN));
    both.println("C FREQ END:" + String(CONF_FREQ_END));
    both.println("C SAMPLES:" + String(CONF_SAMPLES));
}

StatusBar *statusBar;

void setup(void)
{
    for (int i = 0; i < STEPS; i++)
    {
        xRSSI[i] = 999;
        max_x_rssi[i] = 999;
    }
#ifdef LOG_DATA_JSON
    Serial.begin(115200);

#ifdef SEEK_ON_X
    // Initialize custom Serial port on the hardware using pins
    // Parameters: baud rate, serial config, RX pin, TX pin
    SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
#endif
#endif

#ifdef LILYGO
    setupBoards(); // true for disable U8g2 display library
    delay(500);
    Serial.println("Setup LiLyGO board is done");
#endif

    // LED brightness
    heltec_led(25);
#ifdef OSD_ENABLED
    osd.init(OSD_SCK, OSD_MISO, OSD_MOSI);
    osd.clear();

    /* Write the custom character to the OSD, replacing the original character*/
    /* Expand 0xe0 to 0x0e0, the high 8 bits indicate page number and the low 8 bits
     * indicate the inpage address.*/
    osd.storeChar(0xe0, buf0);

    // Display Satellite icon in the left bottom corner
    osd.displayChar(14, 1, 0x10f);
    /*display String*/
    osd.displayString(14, 15, "    Lora SA");
    osd.displayString(2, 1, "   Spectral RF Analyzer");
#endif

    float resolution;
    bt_start = millis();
    wf_start = millis();

    config = Config::init();
#if defined(HAS_SDCARD)
    SD.end();
    SDCardSPI.end(); // end SPI before other uses, eg radio2 over SPI
#endif

    pinMode(LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(REB_PIN, OUTPUT);
    heltec_setup();

    if (!initUARTs(config))
    {
        Serial.println("Failed to initialize UARTs");
    }

    if (!initSPIs(config))
    {
        Serial.println("Failed to initialize SPIs");
    }

    if (!initWires(config))
    {
        Serial.println("Failed to initialize I2Cs");
    }

    r.comms_initialized = Comms::initComms(config);
    if (r.comms_initialized)
    {
        Serial.println("Comms initialized fine");
    }
    else
    {
        Serial.println("Comms did not initialize");
    }

#ifdef JOYSTICK_ENABLED
    calibrate_joy();
    pinMode(JOY_BTN_PIN, INPUT_PULLUP);
#endif
    UI_Init(&display);
    for (int i = 0; i < 200; i++)
    {
        button.update();
        delay(10);
        if (button.pressed())
        {
            r.sound_on = !r.sound_on;
            tone(BUZZER_PIN, 205, 100);
            delay(50);
            tone(BUZZER_PIN, 205, 100);
            break;
        }
    }

    display.clear();

#ifdef WEB_SERVER
    both.println("CLICK for WIFI settings.");

    for (int i = 0; i < 200; i++)
    {
        both.print(".");

        button.update();
        delay(10);
        if (button.pressedNow())
        {
            both.println("-----------");
            both.println("Starting WIFI-SERVER...");
            // Error here: E (15752) ledc: ledc_get_duty(745): LEDC is not initialized
            tone(BUZZER_PIN, 205, 100);
            delay(50);
            tone(BUZZER_PIN, 205, 500);
            tone(BUZZER_PIN, 205, 100);
            delay(50);

            serverStart();
            both.println("Ready to Connect: 192.168.4.1");
            delay(600);
            break;
        }
    }
    both.print("\n");

    both.println("Init File System");
    initLittleFS();

    readConfigFile();
#endif

#ifdef BT_MOBILE
    initBT();
#endif

#ifndef WEB_SERVER
    if (config.scan_ranges_sz == 0 && SCAN_RANGES.length() > 0)
    {
        config.configureDetectionStrategy(config.detection_strategy + ":" + SCAN_RANGES);
    }

    configureDetection();

    both.println("FREQ BEGIN:" + String(CONF_FREQ_BEGIN));
    both.println("FREQ END:" + String(CONF_FREQ_END));
    both.println("SAMPLES:" + String(CONF_SAMPLES));
#endif
    init_radio();

#ifndef LILYGO
    vbat = heltec_vbat();
    both.printf("V battery: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
    delay(1000);
#endif // end not LILYGO
#ifdef WIFI_SCANNING_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
#endif
#ifdef BT_SCANNING_ENABLED

#endif
    delay(400);
    display.clear();

    resolution = (float)RANGE / (STEPS * SCAN_RBW_FACTOR);

    single_page_scan = (RANGE_PER_PAGE == range);

#ifdef DISABLED_CODE
    // Adjust range if it is not even to RANGE_PER_PAGE
    if (!single_page_scan && range % RANGE_PER_PAGE != 0)
    {
        range = range + range % RANGE_PER_PAGE;
    }
#endif

    if (single_page_scan)
    {
        both.println("Single Page Screen MODE");
        both.println("Multi Screen View Press P - button");
        both.println("Multi Screen Res: " + String(resolution) + "Mhz/tick");
        both.println(
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_FACTOR)) +
            "MHz/tick");
        for (int i = 0; i < 500; i++)
        {
            button.update();
            delay(5);
            both.print(".");
            if (button.pressed())
            {
                RANGE_PER_PAGE = DEFAULT_RANGE_PER_PAGE;
                single_page_scan = false;
                tone(BUZZER_PIN, 205, 100);
                delay(50);
                tone(BUZZER_PIN, 205, 100);
                break;
            }
        }
    }
    else
    {
        both.println("Multi Page Screen MODE");
        both.println("Single screen View Press P - button");
        both.println("Single screen Resol: " + String(resolution) + "Mhz/tick");
        both.println(
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_FACTOR)) +
            "Mhz/tick");
        for (int i = 0; i < 500; i++)
        {
            button.update();
            delay(10);
            both.print(".");
            if (button.pressed())
            {
                RANGE_PER_PAGE = range;
                single_page_scan = true;
                tone(BUZZER_PIN, 205, 100);
                break;
            }
        }
    }

    configurePages();
    display.clear();
    init_fonts();
    Serial.println();

    // waterfall start line y-axis
    w = WATERFALL_START;
#ifdef OSD_ENABLED
    osd.clear();
#endif

#ifdef LOG_DATA_JSON
    xTaskCreate(logToSerialTask, "LOG_DATA_JSON", 2048, NULL, 1, &logToSerial);
#endif
    xTaskCreate(dumpToCommsTask, "DUMP_RESPONSE_PROCESS", 2048, NULL, 1, &dumpToComms);

    r.trigger_level = TRIGGER_LEVEL;
    stacked.reset(0, 0, display.width(), display.height());
    bar = new DecoratedBarChart(display, 0, 0, display.width(), 0, CONF_FREQ_BEGIN,
                                CONF_FREQ_END, LO_RSSI_THRESHOLD, HI_RSSI_THRESHOLD,
                                r.trigger_level);

    size_t b = stacked.addChart(bar);

    statusBar = new StatusBar(display, 0, 0, display.width(), r);

#if (WATERFALL_ENABLED == true)
    size_t *multiples = new size_t[6]{5, 3, 4, 15, 4, 3};
    WaterfallModel *model =
        new WaterfallModel((size_t)display.width(), 1000, 6, multiples);
    model->reset(millis(), display.width());

    delete[] multiples;

    waterChart = new WaterfallChart(display, 0, WATERFALL_START, display.width(), 0,
                                    CONF_FREQ_BEGIN, CONF_FREQ_END, r.trigger_level,
                                    WATERFALL_SENSITIVITY, model);

    size_t c = stacked.addChart(waterChart);
    stacked.setHeight(c, stacked.height - WATERFALL_START - statusBar->height);

    r.addEventListener(DETECTED, *waterChart);
#endif

    size_t d = stacked.addChart(statusBar);
    stacked.setHeight(b, stacked.height);

    r.addEventListener(DETECTED, bar->bar);
    r.addEventListener(SCAN_TASK_COMPLETE, stacked);
    r.addEventListener(DETECTED, drone_sound_alarm, &r);

    frequency_scan_result.readings_sz = 0;
    frequency_scan_result.dump.sz = 0;

    r.addEventListener(ALL_EVENTS, eventListenerForReport, NULL);

#ifdef COMPASS_ENABLED

    Serial.println("Compass Init Start");
    Wire1.end();
    Wire1.begin(46, 42);

    Serial.println("Compass BEGIN");
    Serial.println("HMC5883 Magnetometer Test");

    /* Initialise the sensor */
    if (!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();
    Serial.println("Compass Success!!!");

#endif

    if (wireDevices & QMC5883L || wire1Devices & QMC5883L)
    {
        compass = new QMC5883LCompass(wireDevices & QMC5883L ? Wire : Wire1);
    }

    if (compass)
    {
        if (!compass->begin())
        {
            Serial.println("Failed to initialize Compass");
        }

        String err = compass->selfTest();
        if (err.startsWith("OK\n"))
        {
            Serial.printf("Compass self-test passed: %s\n", err.c_str());
        }
        else
        {
            Serial.printf("Compass self-sets failed: %s\n", err.c_str());
        }
    }

#ifdef UPTIME_CLOCK
    uptime = new UptimeClock(display, millis());
#endif
}

// Formula to translate 33 bin to approximate RSSI value
int binToRSSI(int bin)
{
    // the first the strongest RSSI in bin value is 0
    return 11 + (bin * 4);
}

// is there an input using Hot Button or joystick
bool buttonInputRequested()
{
    if (button.pressedFor(100)
#ifdef JOYSTICK_ENABLED
        || joy_btn_click()
#endif
    )
    {
        button.update();
        if (button.pressedNow()
#ifdef JOYSTICK_ENABLED
            || joy_btn_click()
#endif
        )
        {
            return true;
        }
    }

    return false;
}

enum ButtonEvent
{
    NONE = 0,
    LONG_PRESS,
    SHORT_PRESS,
    TOO_SHORT,
    SUSPEND
};

ButtonEvent buttonPressEvent()
{
    button_pressed_counter = 0;
    // if long press stop
    while (button.pressedNow()
#ifdef JOYSTICK_ENABLED
           || joy_btn_click()
#endif
    )
    {
        delay(10);
        button_pressed_counter++;
        if (button_pressed_counter > 150)
        {
            digitalWrite(LED, HIGH);
            delay(150);
            digitalWrite(LED, LOW);
        }
    }
    if (button_pressed_counter > 150)
    {
        return LONG_PRESS;
    }

    if (button_pressed_counter > 50)
    {
        if (!joy_btn_clicked)
        {
            return SUSPEND;
        }
        return SHORT_PRESS;
    }
    button.update();

    return TOO_SHORT;
}

void drone_sound_alarm(void *arg, Event &e)
{
    if (e.type != DETECTED)
    {
        return;
    }

    Scan &r = *((Scan *)arg);
    if (!r.sound_on)
        return;

    int tone_freq_db = e.detected.detected_at * 2;
    int drone_detection_level = r.drone_detection_level;
    int detection_count = r.detection_count;

    // If level is set to sensitive,
    // start beeping every 10th frequency and shorter
    // it improves performance less short beep delays...
    if (drone_detection_level <= 25)
    {

        if (tone_freq_db != 205)
        {
            tone_freq_db = 285 - tone_freq_db;
        }

        if (r.detection_count == 1 && r.sound_on)
        {
            tone(BUZZER_PIN, tone_freq_db,
                 10); // same action ??? but first time
        }
        if (r.detection_count % 5 == 0 && r.sound_on)
        {
            tone(BUZZER_PIN, tone_freq_db,
                 10); // same action ??? but every 5th time
        }
    }
    else
    {
        if (r.detection_count % 20 == 0 && r.sound_on)
        {
            tone(BUZZER_PIN, 205,
                 10); // same action ??? but every 20th detection
        }
    }
}

void joystickMoveCursor(int joy_x_pressed)
{

    if (joy_x_pressed > 0)
    {
        cursor_x_position--;
        display.drawString(cursor_x_position, 0, String((int)r.current_frequency));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10);
        display.display();
        delay(10);
    }
    else if (joy_x_pressed < 0)
    {
        cursor_x_position++;
        display.drawString(cursor_x_position, 0, String((int)r.current_frequency));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10);
        display.display();
        delay(10);
    }
    if (cursor_x_position > DISPLAY_WIDTH || cursor_x_position < 0)
    {
        cursor_x_position = 0;
        display.drawString(cursor_x_position, 0, String((int)r.current_frequency));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10);
        display.display();
        delay(10);
    }
}

bool is_new_x_pixel(int x)
{
    if (x % SCAN_RBW_FACTOR == 0)
        return true;
    else
        return false;
}

/*
 * Given message type and config, modify from/to to route the message to the
 * right destination.
 */
void routeMessage(RoutedMessage &m)
{
    if (m.message == NULL)
    {
        return;
    }

    if (m.message->type == MessageType::SCAN_RESULT ||
        m.message->type == MessageType::SCAN_MAX_RESULT)
    {
        m.to.host = 1;
        return;
    }

    if (m.message->type == MessageType::CONFIG_TASK &&
        (m.message->payload.config.task_type == ConfigTaskType::GETSET_SUCCESS ||
         m.message->payload.config.task_type == ConfigTaskType::SET_FAIL))
    {
        m.to.addr = 0;
        m.to.host = 1;
        return;
    }

    if (config.is_host &&
        (m.message->type == MessageType::SCAN ||
         m.message->type == MessageType::CONFIG_TASK &&
             m.message->payload.config.key->equalsIgnoreCase("detection_strategy")))
    {
        m.to.lora = 1; // apply to self, and send over to lora
        return;
    }
}

int16_t sendMessage(RadioComms &c, Message &m);

void loraSendMessage(Message &m);

Result<int16_t, Message *> checkRadio(RadioComms &c);

void display_scan_result(ScanTaskResult &dump);

std::unordered_map<int, int16_t> findMaxRssi(int16_t *rssis, uint32_t *freqs_khz,
                                             int dump_sz, int level = 80);

void dumpHashMap(const std::unordered_map<unsigned int, RSSIData> &map)
{
    for (const auto &entry : map)
    {
        Serial.println("Key: " + String(entry.first) +
                       ", RSSI: " + String(entry.second.rssi) +
                       ", Time: " + String(entry.second.time) + " ms\n");
    }
}

void display_raw_scan(ScanTaskResult &dump)
{
    // display.setDisplayRotation(1);
    // display.println("Host Mode ->");
    dumpHashMap(historyRSSI);

    size_t dump_sz = dump.sz;
    int16_t *rssi = dump.rssis;
    uint32_t *fr = dump.freqs_khz;

    std::unordered_map<int, int16_t> maxMhzRssi =
        findMaxRssi(rssi, fr, dump_sz, abs(TRIGGER_LEVEL));

    int lx = 0;
    int ly = 0;
    int i = 0;
    String frSign = ":";
    for (const auto &pair : maxMhzRssi)
    {
        if (i == 0)
        {
            display.clear();
            display.drawString(100, 0, "R:" + String(dump.prssi));
        }

        int16_t rssi = pair.second;
        int16_t fr = (int)pair.first;

        if (historyRSSI.find(fr) != historyRSSI.end())
        {
            // Clear old RSSI
            if (millis() - historyRSSI[fr].time > 60 * 1000)
            {
                historyRSSI.erase(fr);
            }
            else
            {
                if (abs(historyRSSI[fr].rssi) > abs(rssi))
                {
                    frSign = "<";
                }
                else if (abs(historyRSSI[fr].rssi) < abs(rssi))
                {
                    frSign = ">";
                }
                else
                {
                    frSign = "=";
                }
            }
        }

        // Save previous value
        historyRSSI[fr] = {(short)abs(rssi), millis()};
        Serial.println("RSSI:" + String(historyRSSI[fr].rssi) + "/" +
                       String(historyRSSI[fr].time) + ":" + String(millis()));

        // screen overflow protection
        if (lx < 130)
        {

            display.drawString(lx, ly, String(fr) + frSign + String(rssi));

            // go to next line
            ly = ly + 10;
            if (ly > 55)
            {
                ly = 0;
                // go to next column
                lx = lx + 45;
            }
        }
        i++;
    }

    display.display();
}
/*
 * If m.to is LOOP, the message is directed at this module; enact the message.
 * If m.to is not LOOP, send the message via the respective interface.
 */
void sendMessage(RoutedMessage &m)
{
    if (m.message == NULL)
    {
        return;
    }

    Message *msg = m.message;

    if (m.to.loop)
    {
        switch (msg->type)
        {
        case MessageType::SCAN:
            report_scans = msg->payload.scan;
            requested_host = !!m.from.host;
            break;
        case MessageType::CONFIG_TASK:
        {
            ConfigTaskType ctt = msg->payload.config.task_type;

            // sanity check; GETSET_SUCCESS and SET_FAIL should get routed to HOST
            if (ctt == ConfigTaskType::GET || ctt == ConfigTaskType::SET)
            {
                // must be GET or SET - both require sending back a response
                RoutedMessage resp;
                resp.to.addr = m.from.addr;
                resp.to.loop = 0;

                resp.from.addr = 0;
                resp.from.loop = 1;
                resp.message = new Message();
                resp.message->type = msg->type;

                String old_v = config.getConfig(*msg->payload.config.key);
                bool success = true;
                if (ctt == ConfigTaskType::SET)
                {
                    success = config.updateConfig(*msg->payload.config.key,
                                                  *msg->payload.config.value);

                    if (success &&
                        msg->payload.config.key->equalsIgnoreCase("detection_strategy"))
                    {
                        configureDetection(); // redo the pages and scan ranges
                    }
                }

                resp.message->payload.config.key = new String(*msg->payload.config.key);
                if (success)
                {
                    resp.message->payload.config.task_type = GETSET_SUCCESS;
                    resp.message->payload.config.value = new String(old_v);
                }
                else
                {
                    resp.message->payload.config.task_type = SET_FAIL;
                }

                sendMessage(resp);

                delete resp.message;
            }
        }
        break;
        case SCAN_RESULT:
        case SCAN_MAX_RESULT:
            if (config.is_host)
            {
#ifdef DISPLAY_RAW_SCAN
                display_raw_scan(m.message->payload.dump);
#else
                display_scan_result(m.message->payload.dump);
#endif
            }
            break;
        case HEADING:
            droneHeading.setHeading(millis(), m.message->payload.heading.heading);
            break;
        }
    }

    if (m.to.host)
    {
        HostComms->send(*m.message);
    }

    if (m.to.uart0)
    {
        Comms0->send(*m.message);
    }

    if (m.to.uart1)
    {
        Comms1->send(*m.message);
    }

    if (m.to.lora)
    {
        if (config.is_host && TxComms != NULL)
        {
            checkRadio(*TxComms); // waiting for peer to squak first, so message
                                  // sending will land on the receiving cycle
        }

        loraSendMessage(*m.message);
    }
}

RoutedMessage checkComms()
{
    RoutedMessage mess;
    mess.from.addr = 0;
    mess.to.addr = 0;
    mess.to.loop = 1;
    mess.message = NULL;

    while (HostComms->available() > 0)
    {
        Message *m = HostComms->receive();
        if (m == NULL)
            continue;

        mess.from.host = 1;
        mess.message = m;
        return mess;
    }

    while (Comms0->available() > 0)
    {
        Message *m = Comms0->receive();
        Serial.println("Comms0: was available, but didn't receive");
        if (m == NULL)
            continue;

        mess.from.uart0 = 1;
        mess.message = m;
        return mess;
    }

    while (Comms1->available() > 0)
    {
        Message *m = Comms1->receive();
        Serial.println("Comms1: was available, but didn't receive");
        if (m == NULL)
            continue;

        mess.from.uart1 = 1;
        mess.message = m;
        return mess;
    }

    // NB: swapping the use of Tx and Rx comms, so a pair of modules
    //     with identical rx/tx_lora config can talk
    RadioComms *rx = config.is_host ? TxComms : RxComms;

    if (rx != NULL && (config.is_host || config.lora_enabled))
    {
        Result<int16_t, Message *> res = checkRadio(*rx);

        if (!res.is_ok)
        {
            int16_t status = res.not_ok;
            if (status != RADIOLIB_ERR_NONE)
            {
                Serial.printf("Error getting a message: %d\n", status);
            }
        }
        else
        {
            mess.from.lora = 1;
            mess.message = res.ok;
        }

        return mess;
    }

    mess.from.loop = 1;
    return mess;
}

// MAX Frequency RSSI BIN value of the samples
int max_rssi_x = 999;

void doScan();

void reportScan();

#ifdef COMPASS_ENABLED
float getCompassHeading()
{
    /* code */

    /* Get a new sensor event */
    sensors_event_t event2;
    mag.getEvent(&event2);

#ifdef COMPASS_DEBUG
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("X: ");
    Serial.print(event2.magnetic.x);
    Serial.print("  ");
    Serial.print("Y: ");
    Serial.print(event2.magnetic.y);
    Serial.print("  ");
    Serial.print("Z: ");
    Serial.print(event2.magnetic.z);
    Serial.print("  ");
    Serial.println("uT");
#endif

    // Hold the module so that Z is pointing 'up' and you can measure the heading with
    // x&y Calculate heading when the magnetometer is level, then correct for signs of
    // axis. float heading = atan2(event.magnetic.y, event.magnetic.x); Use Y as the
    // forward axis float heading = atan2(event.magnetic.x, event.magnetic.y);
    /// If Z-axis is forward and Y-axis points upward:
    // float heading = atan2(event.magnetic.x, event.magnetic.y);
    //  If Z-axis is forward and X-axis points upward:
    //  float heading = atan2(event.magnetic.y, -event.magnetic.x);

    // heading based on the magnetic readings from the Z-axis (forward) and the X-axis
    // (perpendicular to Z, horizontal).
    // float heading = atan2(event.magnetic.z, event.magnetic.x);

    // Dynamicly Calibrated out

    // Read raw magnetometer data
    float x = event2.magnetic.x;
    float y = event2.magnetic.y;
    float z = event2.magnetic.z;

    // Update min/max values dynamically
    x_min = min(x_min, x);
    x_max = max(x_max, x);
    y_min = min(y_min, y);
    y_max = max(y_max, y);
    z_min = min(z_min, z);
    z_max = max(z_max, z);

    Serial.println("x_min:" + String(x_min) + " x_max: " + String(x_max) +
                   " y_min: " + String(y_min));

    // Calculate offsets and scales in real-time
    float x_offset = (x_max + x_min) / 2;
    float y_offset = (y_max + y_min) / 2;
    float z_offset = (z_max + z_min) / 2;

    float x_scale = (x_max - x_min) / 2;
    float y_scale = (y_max - y_min) / 2;
    float z_scale = (z_max - z_min) / 2;

    // Apply calibration to raw data
    float calibrated_x = (x - x_offset) / x_scale;
    float calibrated_y = (y - y_offset) / y_scale;
    float calibrated_z = (z - z_offset) / z_scale;

    // Calculate heading using Z-axis forward, X-axis horizontal
    float heading = atan2(calibrated_z, calibrated_x);

    // Once you have your heading, you must then add your 'Declination Angle', which
    // is the 'Error' of the magnetic field in your location. Find yours here:
    // http://www.magnetic-declination.com/ Mine is: -13* 2' W, which is ~13 Degrees,
    // or (which we need) 0.22 radians If you cannot find your Declination, comment
    // out these two lines, your compass will be slightly off.
    float declinationAngle = 0.22;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if (heading < 0)
        heading += 2 * PI;

    // Check for wrap due to addition of declination.
    if (heading > 2 * PI)
        heading -= 2 * PI;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180 / M_PI;
    return headingDegrees;
}
#endif

float historicalCompassRssi[STEPS] = {999};
int compassCounter = 0;
// max_x_rssi[i] = 999;
int maxRssiHist = 9999;
int maxRssiHistX = 0;
float maxRssiHeading = 0;
int oldX = 0;

void loop(void)
{
    r.led_flag = false;

    r.detection_count = 0;
    drone_detected_frequency_start = 0;

    for (RoutedMessage mess = checkComms(); mess.message != NULL; mess = checkComms())
    {
        routeMessage(mess);
        sendMessage(mess);
        delete mess.message;
    }

    if (compass != NULL)
    {
        int16_t heading = compass->heading();
        Serial.printf("Heading: %" PRIi16 "\n", heading);
    }

    if (!config.is_host)
    {
        doScan();
        reportScan();
    }
#ifdef COMPASS_ENABLED
#if defined(COMPASS_FREQ)
    delay(1000);
    display.clear();
#endif // COMPAS_FREQ
    // Redraw Chart scale line
    statusBar->ui_initialized = false;
    int t = 1;
#ifdef COMPASS_RSSI
    t = 100;

#endif
    draw360Scale(0, 360, 128, 64);
    while (t > 0)
    {
        // ToDO: fix go to;
    compass:
        float headingDegrees = getCompassHeading();
        // Serial.println("Heading (degrees): " + String(headingDegrees));
#ifndef COMPASS_FREQ
        t = 0;
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(80, 0, String((int)headingDegrees));
        display.display();
#endif
#ifdef COMPASS_FREQ

        if (compassCounter == 0)
        {
            display.clear();
            draw360Scale(0, 360, 128, 64);
        }

        float rssi = -122;
        float rssiMax = -999;
        if (headingDegrees >= 0 && headingDegrees <= 360)
        {
            float startFreq = COMPASS_FREQ - 1.0; // Start 2 MHz left
            float endFreq = COMPASS_FREQ + 1.0;   // End 2 MHz right
            float step = 0.5;                     // Step size in MHz
#ifdef COMPASS_RSSI
            for (int i = 0; i < SAMPLES_RSSI; i++)
            {

                for (float freq = startFreq; freq <= endFreq; freq += step)
                {
                    setFrequency(freq);
                    // Serial.println("COMPASS FREQ SET: " + String(freq));
                    draw360Scale(0, 360, 128, 64);
                    // heltec_delay(5);
#ifdef USING_LR1121
                    radio.getRssiInst(&rssi);
#else
                    rssi = getRssi(false);
#endif
                    float headingDegreesAfter = getCompassHeading();
                    float compassDiff = abs(headingDegreesAfter - headingDegrees);
                    if (compassDiff >= 3)
                    {
                        goto compass;
                    }
                    if (rssi > rssiMax)
                    {
                        rssiMax = rssi;
                    }
                }
            }
#else  // end COMPASS RSSI
            int rssiMax = 999;
            /*for (const auto &pair : freqX)
            {
                Serial.println("freq: " + String(pair.first) +
                               ", x: " + String(pair.second));
            }
            for (int i = 0; i < STEPS; i++)
            {
                Serial.println("X: " + String(i) + ", RSSI: " + String(xRSSI[i]));
            }
            */
            for (int i = 0; i < STEPS; i++)
            {
                Serial.println("X pix: " + String(i) +
                               ", RSSI: " + String(max_x_rssi[i]));
            }
            auto it = freqX.find(COMPASS_FREQ);
            if (it != freqX.end())
            {
                int x = it->second;
                Serial.println("RSSI X: " + String(x));
                rssiMax = max_x_rssi[x];
                // Add side pixels to the max +1-1Mhz
                if (x - 1 > 0 && max_x_rssi[x - 1] < rssiMax)
                {
                    rssiMax = max_x_rssi[x - 1];
                }
                if (x + 1 < STEPS && max_x_rssi[x + 1] < rssiMax)
                {
                    rssiMax = max_x_rssi[x + 1];
                }
            }
            else
            {
                rssiMax = 120;
            }
#endif // end COMPASS RSSI
            button.update();
            int gain = 20;
            // Serial.println("RSSI: " + String(rssiMax));
            rssiMax = abs((int)rssiMax);
            float xResolution = (float)((float)360 / (float)128);
            int newX = (float)((float)headingDegrees / (float)xResolution);

            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.setColor(BLACK);
            display.drawString(oldX, 50, "^");
            display.setColor(WHITE);
            display.drawString(newX, 50, "^");
            oldX = newX;
            // Serial.println("COMPASS (" + String(headingDegrees) + " / " +
            //                String(xResolution) + ") PIXEL: " + String(newX));
            //  Showing not max but some live avarage
            //  if (historicalCompassRssi[newX] > rssiMax)
            {
                // Remove old historical data
                display.setColor(BLACK);
                display.drawVerticalLine(newX, 10, 40);
                display.setColor(WHITE);
                if (historicalCompassRssi[newX] != 999 &&
                    historicalCompassRssi[newX] != 0 &&
                    abs(abs(historicalCompassRssi[newX]) - abs(rssiMax)) < 4)
                {
                    historicalCompassRssi[newX] =
                        (rssiMax + historicalCompassRssi[newX]) / 2;
                }
                else
                {
                    historicalCompassRssi[newX] = rssiMax;
                }
            }
            //  Historical compass RSSI
            for (int i = 0; i < STEPS; i++)
            {
                display.setColor(BLACK);
                display.drawString(i, 10, ".");
                display.setColor(WHITE);

                // Serial.println("Compass x: " + String(historicalCompassRssi[i]));
                if (historicalCompassRssi[i] != 999 && historicalCompassRssi[i] != 0)
                {
                    int length2 = 80 - abs(historicalCompassRssi[i]);
                    if (length2 > 0)
                    {
                        display.drawVerticalLine(i, 64 / 2 + 15 - length2, length2);
                    }
                    if (abs(historicalCompassRssi[i]) <= maxRssiHist)
                    {
                        display.setColor(BLACK);
                        display.drawVerticalLine(maxRssiHistX, 10, 40);
                        display.drawString(maxRssiHistX, 10, "^");
                        display.setColor(WHITE);
                        maxRssiHist = (maxRssiHist + abs(historicalCompassRssi[i])) / 2;
                        maxRssiHeading = headingDegrees;
                        maxRssiHistX = i;
                    }

                    // Experemental feature alowing fake max go out
                    if (i == maxRssiHistX)
                    {
                        // we must push values out of max
                        if (abs(abs(maxRssiHist) - abs(historicalCompassRssi[i]) < 4))
                        {
                            maxRssiHist =
                                (maxRssiHist + abs(historicalCompassRssi[i])) / 2;
                        }
                        else
                        {
                            maxRssiHist = historicalCompassRssi[i];
                        }
                    }

                    if (maxRssiHist == abs(historicalCompassRssi[i]))
                    {
                        display.drawString(i, 10, ".");
                    }
                }
            }
            // Draw max Position
            display.drawVerticalLine(maxRssiHistX, 10, 40);
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(maxRssiHistX, 10, "^");

            display.setTextAlignment(TEXT_ALIGN_CENTER);

            float coeficient = ((float)(64 / 2) + 15) / (130.0f + 20.0f);
            // Curent compass reading
            // show only above 80
            int length = 80 - abs(rssiMax);
            display.setColor(WHITE);
            if (length > 0)
            {
                display.drawVerticalLine(newX, 64 / 2 + 15 - length, length);
            }
            // Serial.println("Compass x length: " + String(length));

            display.setColor(BLACK);
            display.fillRect(0, 0, 128, 10);
            display.setColor(WHITE);
            // Direction to turn drone
            if (maxRssiHistX < newX)
            {
                display.setTextAlignment(TEXT_ALIGN_LEFT);
                display.drawString(0, 0, "<<");
            }
            else if (maxRssiHistX > newX)
            {
                display.setTextAlignment(TEXT_ALIGN_RIGHT);
                display.drawString(128, 0, ">>");
            }
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(60, 0,
        "r:" + String((int)rssiMax) + "h:" + String((int)headingDegrees) + 
        "|r:" + String((int)maxRssiHist) +"m:" + String((int)maxRssiHeading)
                                    // DEBUG STUFF
                                   /*String((int)headingDegrees) + "x:" + String(newX) +
                                   "c:" + String(xResolution) + "l:" + String(length)*/);
#ifdef BT_MOBILE
            sendBTData(headingDegrees, rssiMax); // Send data to BLE client
#endif

            display.display();
            compassCounter++;
            // Null counter
            for (int timer = 0; timer < 20; timer++)
            {
                button.update();

                if (button.pressed())
                {
                    compassCounter = 0;
                    for (int i = 0; i < STEPS; i++)
                    {
                        historicalCompassRssi[i] = 999;
                    }
                    initForScan(FREQ_BEGIN);
                    // Restart BT advert
                    pAdvertising->start();
                    maxRssiHist = 9999;
                    maxRssiHistX = 130;
                    maxRssiHeading = 0;
                    display.clear();
                }
#ifndef COMPASS_RSSI
                heltec_delay(50);
#else
                t = 20;
#endif
            }
            for (int i = 0; i < STEPS; i++)
            {
                max_x_rssi[i] = 999;
            }
        }
#endif
        t--;
    }
#if defined(COMPASS_FREQ)
    display.clear();
#endif // COMPASS_FREQ

#endif // end COMPASS_ENABLED
}

void doScan()
{
    if (!radioIsScan)
    {
        radioIsScan = true;
#ifdef INIT_FREQ
        CONF_FREQ_BEGIN = INIT_FREQ;
#endif
        /*#ifdef COMPASS_FREQ
                CONF_FREQ_BEGIN = COMPASS_FREQ;
                CONF_FREQ_END = COMPASS_FREQ + 1;
        #endif*/
        initForScan(CONF_FREQ_BEGIN);
        state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    }

    // reset scan time
    if (config.print_profile_time)
    {
        scan_time = 0;
        loop_start = millis();
    }
    r.epoch++;

    if (!ANIMATED_RELOAD || !single_page_scan)
    {
        // clear the scan plot rectangle
        UI_clearPlotter();
        UI_clearTopStatus();
    }

    // do the scan
    range = CONF_FREQ_END - CONF_FREQ_BEGIN;
    if (RANGE_PER_PAGE > range)
    {
        RANGE_PER_PAGE = range;
    }

    r.fr_begin = CONF_FREQ_BEGIN;
    r.fr_end = r.fr_begin;

    for (scan_page = 0; scan_page < scan_pages_sz; scan_page++)
    {
        ScanPage &page = scan_pages[scan_page];
        r.fr_begin = page.start_mhz;
        r.fr_end = page.end_mhz;
        range = r.fr_end - r.fr_begin;
        median_frequency = (page.start_mhz + page.end_mhz) / 2;

#ifdef DISABLED_CODE
        if (!ANIMATED_RELOAD || !single_page_scan)
        {
            // clear the scan plot rectangle
            UI_clearPlotter();
        }
#endif

        drone_detected_frequency_start = 0;
        display.setTextAlignment(TEXT_ALIGN_RIGHT);

        for (int i = 0; i < MAX_POWER_LEVELS; i++)
        {
            max_bins_array_value[i] = 0;
        }

        // horizontal (x axis) Frequency loop
        osd_x = 1, osd_y = 2, col = 0, max_bin = 0;
        colOSD = OSD_CHART_START_COL;
        indexAccent = 0;
        // x loop
        for (int range = 0, step = 0; range < page.page_sz; range += (step == 0))
        {
            // the logic is:
            // 1. go through each scan_range in the order that they are declared
            //    in the page
            // 2. start with scan_range.start and always end with scan_range.end
            //    if adding step lands us a little short of end, there will be
            //    extra iteration to scan actual end frequency
            // 3. the next iteration after scanning end frequency will be next range
            // 4. x is derived from the frequency we are going to scan with respect to
            //    the page size
            ScanRange scan_range = page.scan_ranges[range];
            uint64_t curr_freq = scan_range.start_khz + scan_range.step_khz * step;
            if (curr_freq > scan_range.end_khz)
                curr_freq = scan_range.end_khz;

            // for now support legacy calculation of x that relies on SCAN_RBW_FACTOR
            x = (curr_freq - page.start_mhz * 1000) * STEPS * SCAN_RBW_FACTOR /
                ((page.end_mhz - page.start_mhz) * 1000);

            new_pixel = step == 0 || is_new_x_pixel(x);
            if (curr_freq == scan_range.end_khz)
            {
                step = 0; // trigger switch to the next range on the next iteration
            }
            else
            {
                step++;
            }

            if (ANIMATED_RELOAD && SCAN_RBW_FACTOR == 1)
            {
                UI_drawCursor(x);
            }
            if (new_pixel && ANIMATED_RELOAD && SCAN_RBW_FACTOR > 1)
            {
                UI_drawCursor((int)(x / SCAN_RBW_FACTOR));
            }

#ifdef PRINT_PROFILE_TIME
            scan_start_time = millis();
#endif
            // Real display pixel x - axis.
            // Because of the SCAN_RBW_FACTOR x is not a display coordinate anymore
            // x > STEPS on SCAN_RBW_FACTOR
            int display_x = x / SCAN_RBW_FACTOR;
            freqX[(int)r.current_frequency] = display_x;
            setFrequency(curr_freq / 1000.0);
            if (radio2 != NULL)
            {
                state = radio2->setFrequency(curr_freq / 1000.0);
                if (state != RADIOLIB_ERR_NONE)
                {
                    Serial.printf("Radio2: Failed to set freq: %d\n", state);
                }
            }

            LOG("Step:%d Freq: %f\n", x, r.current_frequency);
            // SpectralScan Method
#ifdef METHOD_SPECTRAL
            {
                // start spectral scan third parameter is a sleep interval
                radio.spectralScanStart(SAMPLES, 1);
                // wait for spectral scan to finish
                radio_error_count = 0;
                while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE)
                {
                    Serial.println("radio.spectralScanGetStatus ERROR: ");
                    Serial.println(radio.spectralScanGetStatus());
                    display.drawString(0, 64 - 20,
                                       "E:specScSta:" +
                                           String(radio.spectralScanGetStatus()));
                    display.display();
                    heltec_delay(ONE_MILLISEC * 2);
                    radio_error_count++;
                    if (radio_error_count > 10)
                        continue;
                }

                // read the results Array to which the results will be saved
                state = radio.spectralScanGetResult(result);
                display.drawString(0, 64 - 10, "scanGetResult:" + String(state));
            }

#endif
#ifdef METHOD_RSSI
            // Spectrum analyzer using getRSSI
            float rssi2 = -999;

            {
                LOG("METHOD RSSI");

                float (*g)(void *);
                samples = CONF_SAMPLES;

                if (config.detection_strategy.equalsIgnoreCase("RSSI") ||
                    config.detection_strategy.equalsIgnoreCase("RSSI_MAX"))
                    g = &getRSSI;
                else if (config.detection_strategy.equalsIgnoreCase("CAD"))
                {
                    g = &getCAD;
                    samples = min(1,
                                  CONF_SAMPLES); // TODO: do we need to support values
                                                 // other than 1
                }
                else
                    g = &getRSSI;
                uint16_t max_rssi = 120;

                // Scan if not in the ignore list
                if (ignoredFreq.find((int)r.current_frequency) == ignoredFreq.end())
                {
                    max_rssi = r.rssiMethod(g, &r, samples, result,
                                            RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);
                    /*Serial.print("xx: " + String(display_x));
                    Serial.println(" RSSI: " + String(max_rssi));*/
                    if (max_rssi != 0 && xRSSI[display_x] > (int)max_rssi)
                    {
                        xRSSI[display_x] = (int)max_rssi;
                    }

                    for (int i = 0; radio2 != NULL && i < samples; i++)
                    {
                        rssi2 = max(rssi2, radio2->getRSSI());
                    }
                }
                else
                {
                    // if ignored default RSSI value -120dB
                    max_rssi = 120;
                }
                // 0 is default after clear {999}
                if (max_x_rssi[display_x] == 0 ||
                    (max_x_rssi[display_x] > max_rssi && max_rssi != 0))
                {
                    max_x_rssi[display_x] = max_rssi;
                }
            }
#endif // SCAN_METHOD == METHOD_RSSI

            // if this code is not executed LORA radio doesn't work
            // basically SX1262 requires delay
            // osd.displayString(12, 1, String(CONF_FREQ_BEGIN));
            // osd.displayString(12, 30 - 8, String(FREQ_END));
            // delay(2);

#ifdef OSD_ENABLED
            osdProcess();
#endif
#ifdef JOYSTICK_ENABLED
            if (display_x == cursor_x_position)
            {
                display.setColor(BLACK);
                display.fillRect(display_x - 20, 3, 36, 11);
                display.setColor(WHITE);
            }
#endif
            Event event = r.detect(result, filtered_result,
                                   RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE, samples);
            event.time_ms = millis();
            event.detected.rssi2 = rssi2;

            size_t detected_at = event.detected.detected_at;
            if (max_rssi_x > detected_at)
            {
                // MAx bin Value not RSSI
                max_rssi_x = detected_at;
            }

            detected_y[display_x] = false;

            r.drone_detection_level = drone_detection_level;

            if (event.detected.trigger)
            {
                // check if we should alarm about a drone presence
                if (detected_y[display_x] == false) // detection threshold match
                {
                    // Set LED to ON (filtered in UI component)
                    r.led_flag = true;
                    if (drone_detected_frequency_start == 0)
                    {
                        // mark freq start
                        drone_detected_frequency_start = r.current_frequency;
                    }

                    // mark freq end ... will shift right to last detected range
                    drone_detected_frequency_end = r.current_frequency;

#ifdef LOG_DATA_JSON
                    frequency_scan_result.begin = drone_detected_frequency_start;
                    frequency_scan_result.end = drone_detected_frequency_end;
#endif
                    if (DRAW_DETECTION_TICKS == true)
                    {
// draw vertical line on top of display for "drone detected"
// frequencies
#ifdef METHOD_SPECTRAL
                        if (!detected_y[display_x])
                        {
                            display.drawLine(display_x, 1, display_x, 4);
                            detected_y[display_x] = true;
                        }
#endif
                    }
                }
            }

            r.fireEvent(event);

#ifdef JOYSTICK_ENABLED
            // Draw joystick cursor and Frequency RSSI value
            if (display_x == cursor_x_position)
            {
                display.drawString(display_x - 1, 0, String((int)r.current_frequency));
                display.drawLine(display_x, 1, display_x, 12);
                // if method scan RSSI we can get exact RSSI value
                display.drawString(display_x + 17, 0, "-" + String((int)max_rssi_x * 4));
            }
#endif

#ifdef PRINT_PROFILE_TIME
            scan_time += (millis() - scan_start_time);
#endif
#ifdef PRINT_DEBUG
            Serial.println("....\n");
#endif
            if (r.animated)
            {
                display.display();
            }

            if (buttonInputRequested())
            {
                display.setTextAlignment(TEXT_ALIGN_CENTER);
                display.drawString(display.width() / 2, 0, String(r.current_frequency));
                display.display();

                ButtonEvent e = buttonPressEvent();

                if (e == LONG_PRESS)
                {
                    // Remove Curent Frequency Text
                    display.setTextAlignment(TEXT_ALIGN_CENTER);
                    display.setColor(BLACK);
                    display.drawString(display.width() / 2, 0,
                                       String(r.current_frequency));
                    display.setColor(WHITE);
                    display.display();

                    break;
                }

                if (e == SUSPEND)
                {
                    // Visually confirm it's off so user releases button
                    display.displayOff();
                    // Deep sleep (has wait for release so we don't wake up
                    // immediately)
                    heltec_deep_sleep();
                    break;
                }

                if (e == SHORT_PRESS)
                    break;

                if (e == TOO_SHORT)
                {
                    String v = String(r.trigger_level) + " dB";
                    uint16_t w = display.getStringWidth(v);
                    display.setTextAlignment(TEXT_ALIGN_RIGHT);
                    // erase old drone detection level value
                    display.setColor(BLACK);
                    display.fillRect(display.width() - w, 0, 13, w);
                    display.setColor(WHITE);

                    // dt is roughly single-pixel increment
                    float dt =
                        bar->bar.height == 0
                            ? 0.0
                            : (LO_RSSI_THRESHOLD - HI_RSSI_THRESHOLD) / bar->bar.height;
                    r.trigger_level += dt;
                    if (r.trigger_level <= LO_RSSI_THRESHOLD)
                    {
                        r.trigger_level = HI_RSSI_THRESHOLD;
                    }

                    // print new value
                    display.drawString(display.width(), 0, v);
                    tone(BUZZER_PIN, 104, 150);

                    bar->bar.redraw_all = true;
                }
            }

            // wait a little bit before the next scan,
            // otherwise the SX1262 hangs
            // Add more logic before instead of long delay...
            int delay_cnt = 1;
#ifdef METHOD_SPECTRAL
            if (false && state != RADIOLIB_ERR_NONE)
            {
                if (delay_cnt == 1)
                {
                    Serial.println("E:getResult");
                    display.drawString(0, 64 - 10, "E:getResult");
                    // trying to use display as delay..
                    display.display();
                }
                else
                {
                    heltec_delay(ONE_MILLISEC * 2);
                    Serial.println("E:getStatus");
                    display.drawString(0, 64 - 10, "E:getResult");
                    // trying to use display as delay..
                    display.display();
                }

                Serial.println("spectralScanGetStatus ERROR(" +
                               String(radio.spectralScanGetStatus()) +
                               ") hard delay(2) - " + String(delay_cnt));
                // if error than speed is slow animating chart
                ANIMATED_RELOAD = true;
                delay(50);
                delay_cnt++;
            }
#endif
            // TODO: move osd logic here as a daley ;)
            // Loop is needed if heltec_delay(1) not used
            heltec_loop();
// Move joystick
#ifdef JOYSTICK_ENABLED
            int joy_x_pressed = get_joy_x(true);
            joystickMoveCursor(joy_x_pressed);
#endif
        }
        w++;
        if (w > ROW_STATUS_TEXT + 1)
        {
            w = WATERFALL_START;
        }

        {
            Event event(r, SCAN_TASK_COMPLETE, millis());
            r.fireEvent(event);
        }
        // Render display data here

#ifdef UPTIME_CLOCK
        uptime->draw(millis());
#endif
        display.display();
#ifdef OSD_ENABLED
        // Sometimes OSD prints entire screen with the digits.
        // We need clean the screen to fix it.
        // We can do it every time but to optimise doing every N times
        if (global_counter != 0 && global_counter % 10 == 0)
        {
#if !defined(BT_SCANNING_ENABLED) && !defined(WIFI_SCANNING_ENABLED)
            osd.clear();
            osd.displayChar(14, 1, 0x10f);
            global_counter = 0;
#endif
        }
        ANIMATED_RELOAD = false;
        global_counter++;
#endif
    }
#ifdef PRINT_DEBUG
// Serial.println("----");
#endif

    joy_btn_clicked = false;

    if (config.print_profile_time)
    {
#ifdef PRINT_PROFILE_TIME
        loop_time = millis() - loop_start;
        Serial.printf("LOOP: %lld ms; SCAN: %lld ms;\n  ", loop_time, scan_time);
#endif
    }

// No WiFi and BT Scan Without OSD
#ifdef OSD_ENABLED
#ifdef WIFI_SCANNING_ENABLED
    if ((millis() - wf_start) > WF_SCAN_DELAY)
    {
        scanWiFi();
        wf_start = millis();
        // prevent BT scanning after scanning WF
        bt_start = millis();
    }
#endif
#ifdef BT_SCANNING_ENABLED
    if ((millis() - bt_start) > BT_SCAN_DELAY)
    {

        scanBT();
        bt_start = millis();
    }
#endif
    sideBarRow = SIDEBAR_START_ROW;
#endif
}

std::unordered_map<int, int16_t> findMaxRssi(int16_t *rssis, uint32_t *freqs_khz,
                                             int dump_sz, int level)
{
    std::unordered_map<int, int16_t> maxRssiPerMHz; // Map to store max RSSI per MHz

    for (int i = 0; i < dump_sz; i++)
    {
        int16_t rssi = rssis[i];
        int freq_mhz = (int)freqs_khz[i] / 1000; // Convert kHz to MHz

        // Update the maximum RSSI for this MHz frequency
        if (maxRssiPerMHz.find(freq_mhz) == maxRssiPerMHz.end() ||
            maxRssiPerMHz[freq_mhz] < rssi)
        {
            if (abs(rssi) <= level || frAlwaysShow.find(freq_mhz) != frAlwaysShow.end())
            {
                maxRssiPerMHz[freq_mhz] = rssi;
            }
        }
    }
    return maxRssiPerMHz;
}

bool lock = false;
Result<int16_t, Message *> checkRadio(RadioComms &comms)
{
    radioIsScan = false;
    Result<int16_t, Message *> ret;
    ret.is_ok = false;

    ret.not_ok = comms.configureRadio();
    if (ret.not_ok != RADIOLIB_ERR_NONE)
        return ret;

    ret.is_ok = true;

    Message *msg = comms.receive(
        config.is_host
            ? 2000
            : 200); // 200ms should be enough to receive 500 bytes at SF 7 and BW 500
    ret.ok = msg;

    return ret;
}

int16_t sendMessage(RadioComms &comms, Message &msg)
{
    radioIsScan = false;
    int16_t status = comms.configureRadio();
    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Failed to configure Radio: %d\n", status);
        return status;
    }

    if (false)
    {
        lock = true;

        lock = false;
    }

    status = comms.send(msg);

    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Failed to send message of type %d: %d\n", msg.type, status);
        return status;
    }

    return status;
}

void loraSendMessage(Message &msg)
{
    RadioComms *tx = config.is_host ? RxComms : TxComms;
    if (tx == NULL)
    {
        return;
    }

    sendMessage(*tx, msg);
}

void reportScan()
{
    if (!config.lora_enabled)
        return;

    Message m;
    m.type = SCAN_RESULT;
    m.payload.dump.sz = 0;

    if (config.detection_strategy.equalsIgnoreCase("RSSI"))
    {
        size_t sz = frequency_scan_result.dump.sz;
        m.payload.dump.sz = sz;
        m.payload.dump.freqs_khz = new uint32_t[sz];
        m.payload.dump.rssis = new int16_t[sz];

        memcpy(m.payload.dump.freqs_khz, frequency_scan_result.dump.freqs_khz,
               sizeof(uint32_t) * sz);
        memcpy(m.payload.dump.rssis, frequency_scan_result.dump.rssis,
               sizeof(int16_t) * sz);
    }
    else if (config.detection_strategy.equalsIgnoreCase("RSSI_MAX"))
    {
        m.type = SCAN_MAX_RESULT;

        size_t sz = config.scan_ranges_sz;
        m.payload.dump.sz = sz;
        m.payload.dump.freqs_khz = new uint32_t[sz];
        m.payload.dump.rssis = new int16_t[sz];

        for (int i = 0; i < sz; i++)
        {
            int16_t rssi = -999;
            for (int j = 0; j < frequency_scan_result.dump.sz; j++)
            {
                uint32_t f = frequency_scan_result.dump.freqs_khz[j];

                if (config.scan_ranges[i].start_khz > f ||
                    config.scan_ranges[i].end_khz < f)
                    continue;

                rssi = max(rssi, frequency_scan_result.dump.rssis[j]);
            }

            m.payload.dump.freqs_khz[i] =
                (config.scan_ranges[i].start_khz + config.scan_ranges[i].end_khz) / 2;
            m.payload.dump.rssis[i] = rssi;
        }
    }
    else
    {
        return;
    }

    loraSendMessage(m);
}

void display_scan_result(ScanTaskResult &dump)
{
    if (bar == NULL)
        return;
    // assuming this module and the peer are in sync w.r.t. scan ranges
    if (config.detection_strategy.equalsIgnoreCase("RSSI"))
    {
        for (int i = 0; i < dump.sz; i++)
            bar->bar.updatePoint(dump.freqs_khz[i] / 1000, dump.rssis[i]);

        bar->draw();
        display.display();

        return;
    }

    if (config.detection_strategy.equalsIgnoreCase("RSSI_MAX"))
    {
        float step = (bar->bar.max_x - bar->bar.min_x) / bar->bar.width;

        bar->bar.clear();
        bar->draw_labels = true;

        for (int i = 0; i < config.scan_ranges_sz; i++)
        {
            int j;
            for (j = 0; j < dump.sz; j++)
            {
                if (config.scan_ranges[i].start_khz <= dump.freqs_khz[j] &&
                    config.scan_ranges[i].end_khz >= dump.freqs_khz[j])
                    break;
            }

            int16_t rssi = j < dump.sz ? dump.rssis[j] : bar->bar.min_y;

            for (float f = config.scan_ranges[i].start_khz / 1000;
                 f <= config.scan_ranges[i].end_khz / 1000; f += step)
                bar->bar.updatePoint(f, rssi);
        }

        bar->draw();
        display.display();

        return;
    }
}
