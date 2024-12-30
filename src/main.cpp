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

// We are not using default display library but Adafruit insread
#define HELTEC_NO_DISPLAY_INSTANCE
#define HELTEC_NO_DISPLAY

#include "FS.h"
#include <Arduino.h>
#ifdef LOG_DATA_JSON
#include <ArduinoJson.h>
#endif
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <File.h>
#include <LittleFS.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unordered_map>

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

#if defined(HELTEC) && defined(HELTEC_NO_DISPLAY)
#define DISPLAY_ADDR 0x3C
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
/**
 * @class PrintSplitter
 * @brief A class that splits the output of the Print class to two different
 *        Print objects.
 *
 * The PrintSplitter class is used to split the output of the Print class to two
 * different Print objects. It overrides the write() function to write the data
 * to both Print objects.
 */
class PrintSplitter2 : public Print
{
  public:
    PrintSplitter2(Print &_a, Print &_b) : a(_a), b(_b) {}
    size_t write(uint8_t c)
    {
        a.write(c);
        return b.write(c);
    }
    size_t write(const char *str)
    {
        a.write(str);
        return b.write(str);
    }

  private:
    Print &a;
    Print &b;
};

#define DISPLAY_GEOMETRY GEOMETRY_128_64

#define SCREEN_ADDRESS 0x3C
// Wire.begin(SDA_OLED, SCL_OLED);
Adafruit_SSD1306 display(128, 64, &Wire, RST_OLED);
PrintSplitter2 splitBoth(Serial, display);

#endif

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
#define SIDEBAR_START_COL 2
#define SIDEBAR_END_COL 10
// #define SIDEBAR_ACCENT_ONLY 1
#define SIDEBAR_DB_LEVEL 80 // Absolute value without minus
#define SIDEBAR_DB_DELTA 2  // detect changes <> threshold

// SPI pins
#define OSD_CS 47
#define OSD_MISO 33
#define OSD_MOSI 34
#define OSD_SCK 26
#endif

#define OSD_WIDTH 30
#define OSD_HEIGHT 16
#define OSD_CHART_WIDTH 15
#define OSD_CHART_HEIGHT 5
#define OSD_X_START 1
#define OSD_Y_START 16

// TODO: Calculate dynamically:
// osd_steps = osd_mhz_in_bin / (FM range / LORA radio x Steps)
int osd_mhz_in_bin = 5;
int osd_steps = 12;
int global_counter = 0;

#ifdef OSD_ENABLED
DFRobot_OSD osd(OSD_CS);
#endif

#include "global_config.h"
#include "ui.h"

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

size_t scan_pages_sz = 0;
ScanPage *scan_pages;
size_t scan_page = 0;

// MHZ per page
// to put everything into one page set RANGE_PER_PAGE = FREQ_END - 800
uint64_t RANGE_PER_PAGE; // FREQ_END - CONF_FREQ_BEGIN

uint64_t CONF_FREQ_END, CONF_FREQ_BEGIN; // To Enable Multi Screen scan
//  uint64_t RANGE_PER_PAGE = 50;
//  Default Range on Menu Button Switch

// multiplies STEPS * N to increase scan resolution.
#define SCAN_RBW_FACTOR 2

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

// #define PRINT_DEBUG

#define PRINT_PROFILE_TIME
#ifdef PRINT_PROFILE_TIME
uint64_t loop_start = 0;
uint64_t loop_time = 0;
uint64_t scan_time = 0;
uint64_t scan_start_time = 0;
#endif

// Define which UART port to use (0, 1, or 2)
#define SERIAL_PORT 1

// Define which pins to use
#define TX_PIN 12
#define RX_PIN 16 // not used

// Create HardwareSerial instance (UART 1 or 2)
HardwareSerial SerialPort(SERIAL_PORT);

// #define WEB_SERVER true

uint64_t x, y, w = WATERFALL_START, i = 0;
int osd_x = 1, osd_y = 2, col = 0, max_bin = 32;

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
    // Third line
    if (signal_value <= 9 && signal_value <= drone_detection_level)
    {
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
}

// Start Sidebar
int sideBarCol = SIDEBAR_START_COL;

std::unordered_map<int, bool> accentFreq = {{950, true}, {915, true}};

void osdProcess()
{ // OSD enabled
    osdCyclesCount++;
    // memset(max_step_range, 33, 30);
    max_bin = 32;

    osd.displayString(12, 1, String(CONF_FREQ_BEGIN));
    osd.displayString(12, OSD_WIDTH - 8, String(CONF_FREQ_END));
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
        max_step_range = result[max_bin];
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
                osd.displayString(sideBarCol++, OSD_WIDTH - 7,
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

                if (true || sideBarCol == SIDEBAR_START_COL || freqOSD % 100 == 0)
                {
                    freqOSDString = String(freqOSD);
                }
                else
                {
                    freqOSDString = String(" ") + String(freqOSD).substring(1);
                }

                long int osd_start = millis();
                osd.displayString(sideBarCol++, OSD_WIDTH - 7,
                                  freqOSDString + printChar + String(max_step_range) +
                                      " ");
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
                    for (int i = sideBarCol + 3; i <= SIDEBAR_END_COL; i++)
                    {
                        osd.displayString(i++, OSD_WIDTH - 7, String("       "));
                    }
                    osdCyclesCount = 0;
                }
            }
            // MAx down sidebar
            if (sideBarCol >= SIDEBAR_END_COL)
            {
                sideBarCol = SIDEBAR_START_COL;
            }

#endif
        }
#endif
        // Test with Random Result...
        // max_step_range = rand() % 32;
#ifdef METHOD_RSSI
        // With THe RSSI method we can get real RSSI value not just a bin
#endif
        // PRINT SIGNAL CHAR ROW, COL, VALUE
        osdPrintSignalLevelChart(col, max_step_range);

#ifdef PRINT_DEBUG
        Serial.println("MAX:" + String(max_step_range));
#endif
        max_step_range = 32;
        col++;
    }
    if (osdCyclesCount >= OSD_MAX_CLEAR_CYCLES)
    {
        osdCyclesCount = 0;
    }
}
#endif

Config config;

float getRSSI(void *param)
{
    Scan *r = (Scan *)param;
#if defined(USING_SX1280PA)
    // radio.startReceive();
    // get instantaneous RSSI value
    // When PR will be merged we can use radi.getRSSI(false);
    uint8_t data[3] = {0, 0, 0}; // RssiInst, Status, RFU
    radio.mod->SPIreadStream(RADIOLIB_SX128X_CMD_GET_RSSI_INST, data, 3);
    return ((float)data[0] / (-2.0));

#elif defined(USING_LR1121)
    // Try getRssiInst
    float rssi;
    radio.getRssiInst(&rssi);
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
    state = radio.beginGFSK(freq, 4.8F, 5.0F, 156.2F, 10, 16U, 1.7F);
#else
    state = radio.beginFSK(freq);
#endif

    return state;
}

bool setFrequency(float curr_freq)
{
    r.current_frequency = curr_freq;
    LOG("setFrequency:%f\n", r.current_frequency);

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
    state = radio.setFrequency(freq);
#else
    state = radio.setFrequency(r.current_frequency,
                               true); // false = calibration is needed here
#endif
    if (state != RADIOLIB_ERR_NONE)
    {
        display.setCursor(0, 64 - 10);
        display.print("E(" + String(state) +
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
    splitBoth.println("Init radio");
    state = initForScan(CONF_FREQ_BEGIN);

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
    splitBoth.println("Upload SX1262 patch");

    // Upload binary patch into the SX126x device RAM. Patch is needed to e.g.,
    // enable spectral scan and must be uploaded again on every power cycle.
    RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
    // configure scan bandwidth and disable the data shaping
#endif

    splitBoth.println("Setting up radio");
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
    splitBoth.println("Starting scanning...");

    // calibrate only once ,,, at startup
    // TODO: check documentation (9.2.1) if we must calibrate in certain ranges
    setFrequency(CONF_FREQ_BEGIN);

    delay(50);
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

            if (old_sz > 0)
            {
                memcpy(f, frequency_scan_result.dump.freqs_khz,
                       old_sz * sizeof(uint32_t));
                memcpy(r, frequency_scan_result.dump.rssis, old_sz * sizeof(int16_t));

                delete[] frequency_scan_result.dump.freqs_khz;
                delete[] frequency_scan_result.dump.rssis;
            }

            frequency_scan_result.dump.freqs_khz = f;
            frequency_scan_result.dump.rssis = r;
        }

        frequency_scan_result.dump.freqs_khz[frequency_scan_result.dump.sz] =
            e.detected.freq * 1000; // convert to kHz
        frequency_scan_result.dump.rssis[frequency_scan_result.dump.sz] =
            max(e.detected.rssi, -999.0f);
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
    JsonDocument doc;
    char jsonOutput[200];
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

            doc["low_range_freq"] = frequency_scan_result.begin;
            doc["high_range_freq"] = frequency_scan_result.end;
            doc["value"] = String(highest_value_scanned);

            serializeJson(doc, jsonOutput);
            SerialPort.println(jsonOutput);
        }
    }
}
#endif

void drone_sound_alarm(void *arg, Event &e);

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
        config.scan_ranges_sz = 1;
        config.scan_ranges = new ScanRange[1];
        config.scan_ranges[0].start_khz = FREQ_BEGIN * 1000;
        config.scan_ranges[0].end_khz = FREQ_END * 1000;
        config.scan_ranges[0].step_khz =
            (float)(FREQ_END - FREQ_BEGIN) * 1000 / (STEPS * SCAN_RBW_FACTOR);
    }

    if (config.samples <= 0)
    {
        config.samples = SAMPLES_RSSI;
    }

    CONF_SAMPLES = config.samples;

    CONF_FREQ_BEGIN = config.scan_ranges[0].start_khz / 1000;
    CONF_FREQ_END = config.scan_ranges[0].end_khz / 1000;
    for (int i = 0; i < config.scan_ranges_sz; i++)
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

    splitBoth.println("C FREQ BEGIN:" + String(CONF_FREQ_BEGIN));
    splitBoth.println("C FREQ END:" + String(CONF_FREQ_END));
    splitBoth.println("C SAMPLES:" + String(CONF_SAMPLES));
}

void setup(void)
{

#ifdef LOG_DATA_JSON
    Serial.begin(115200);

    // Initialize custom Serial port
    // Parameters: baud rate, serial config, RX pin, TX pin
    SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
#endif

#ifdef LILYGO
    setupBoards(); // true for disable U8g2 display library
    delay(500);
    Serial.println("Setup LiLybeginSDCardGO board is done");
#else
    // Serial0.begin(115200); // Initialize UART0
    // Serial0.println("Hello, Serial0 (UART0)!");
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
    float vbat;
    float resolution;
    bt_start = millis();
    wf_start = millis();

    config = Config::init();
    r.comms_initialized = Comms::initComms(config);
    if (r.comms_initialized)
    {
        Serial.println("Comms initialized fine");
    }
    else
    {
        Serial.println("Comms did not initialize");
    }

    pinMode(LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(REB_PIN, OUTPUT);
    heltec_setup();
#if defined(HELTEC) && defined(HELTEC_NO_DISPLAY)
    pinMode(RST_OLED, OUTPUT);
    digitalWrite(RST_OLED, HIGH);
    delay(1);
    digitalWrite(RST_OLED, LOW);
    delay(20);
    digitalWrite(RST_OLED, HIGH);
    Wire.begin((int)SDA_OLED, (int)SCL_OLED);
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false);
#endif

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

    display.clearDisplay();

#ifdef WEB_SERVER
    splitBoth.println("CLICK for WIFI settings.");

    for (int i = 0; i < 200; i++)
    {
        splitBoth.print(".");

        button.update();
        delay(10);
        if (button.pressedNow())
        {
            splitBoth.println("-----------");
            splitBoth.println("Starting WIFI-SERVER...");
            // Error here: E (15752) ledc: ledc_get_duty(745): LEDC is not initialized
            tone(BUZZER_PIN, 205, 100);
            delay(50);
            tone(BUZZER_PIN, 205, 500);
            tone(BUZZER_PIN, 205, 100);
            delay(50);

            serverStart();
            splitBoth.println("Ready to Connect: 192.168.4.1");
            delay(600);
            break;
        }
    }
    splitBoth.print("\n");

    splitBoth.println("Init File System");
    initLittleFS();

    readConfigFile();
#endif

#ifndef WEB_SERVER
    if (config.scan_ranges_sz == 0 && SCAN_RANGES.length() > 0)
    {
        config.configureDetectionStrategy(config.detection_strategy + ":" + SCAN_RANGES);
    }

    configureDetection();

    splitBoth.println("FREQ BEGIN:" + String(CONF_FREQ_BEGIN));
    splitBoth.println("FREQ END:" + String(CONF_FREQ_END));
    splitBoth.println("SAMPLES:" + String(CONF_SAMPLES));
#endif
    init_radio();

#ifndef LILYGO
    vbat = heltec_vbat();
    splitBoth.printf("V battery: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
    delay(1000);
#endif // end not LILYGO
#ifdef WIFI_SCANNING_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
#endif
#ifdef BT_SCANNING_ENABLED

#endif
    delay(400);
    display.clearDisplay();

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
        splitBoth.println("Single Page Screen MODE");
        splitBoth.println("Multi Screen View Press P - button");
        splitBoth.println("Multi Screen Res: " + String(resolution) + "Mhz/tick");
        splitBoth.println(
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_FACTOR)) +
            "MHz/tick");
        for (int i = 0; i < 500; i++)
        {
            button.update();
            delay(5);
            splitBoth.print(".");
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
        splitBoth.println("Multi Page Screen MODE");
        splitBoth.println("Single screen View Press P - button");
        splitBoth.println("Single screen Resol: " + String(resolution) + "Mhz/tick");
        splitBoth.println(
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_FACTOR)) +
            "Mhz/tick");
        for (int i = 0; i < 500; i++)
        {
            button.update();
            delay(10);
            splitBoth.print(".");
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
    display.clearDisplay();
    Serial.println();

#ifdef METHOD_RSSI
    // TODO: try RADIOLIB_SX126X_RX_TIMEOUT_INF
#ifdef USING_SX1280PA
    state = radio.startReceive(RADIOLIB_SX128X_RX_TIMEOUT_NONE);
#else
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
#endif

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        display.setCursor(0, 64 - 10);
        display.print("E:startReceive");
        display.display();
        delay(500);
        Serial.println(state);
    }
#endif
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

    Chart *statusBar = new StatusBar(display, 0, 0, display.width(), r);

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
    r.addEventListener(DETECTED, drone_sound_alarm, &r);
    r.addEventListener(SCAN_TASK_COMPLETE, stacked);

    frequency_scan_result.readings_sz = 0;
    frequency_scan_result.dump.sz = 0;

    r.addEventListener(ALL_EVENTS, eventListenerForReport, NULL);

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
        display.setCursor(cursor_x_position, 0);
        display.print(String((int)r.current_frequency));
        display.setCursor(cursor_x_position, 1);
        display.drawFastHLine(cursor_x_position, 10, 1, WHITE);
        display.display();
        delay(10);
    }
    else if (joy_x_pressed < 0)
    {
        cursor_x_position++;
        display.setCursor(cursor_x_position, 0);
        display.print(String((int)r.current_frequency));
        display.setCursor(cursor_x_position, 1);
        display.drawFastHLine(cursor_x_position, 10, 1, WHITE);
        display.display();
        delay(10);
    }
    if (cursor_x_position > DISPLAY_WIDTH || cursor_x_position < 0)
    {
        cursor_x_position = 0;
        display.setCursor(cursor_x_position, 0);
        display.print(String((int)r.current_frequency));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10, WHITE);
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

void checkComms()
{
    while (HostComms->available() > 0)
    {
        Message *m = HostComms->receive();
        if (m == NULL)
            continue;

        switch (m->type)
        {
        case MessageType::SCAN:
            report_scans = m->payload.scan;
            requested_host = true;
            Serial.println("Host: forwarding message SCAN to peer");
            Comms0->send(*m); // forward to peer
            Comms1->send(*m); // forward to peer
            break;
        case MessageType::CONFIG_TASK:
            if (m->payload.config.is_set)
            {
                String v = config.getConfig(*m->payload.config.key);
                bool r =
                    config.updateConfig(*m->payload.config.key, *m->payload.config.value);
                Serial.printf("SET config (%s): %s = %s (was: %s)\n", r ? "OK" : "failed",
                              m->payload.config.key->c_str(),
                              m->payload.config.value->c_str(), v.c_str());
            }
            else
            {
                Serial.printf("GET config: %s = %s\n", m->payload.config.key->c_str(),
                              config.getConfig(*m->payload.config.key).c_str());
            }
            break;
        }
        delete m;
    }

    while (Comms0->available() > 0)
    {
        Message *m = Comms0->receive();
        Serial.println("Comms0: was available, but didn't receive");
        if (m == NULL)
            continue;

        switch (m->type)
        {
        case MessageType::SCAN:
            report_scans = m->payload.scan; // receive from peer
            requested_host = false;
            break;

        case MessageType::SCAN_RESULT:
            HostComms->send(*m); // forward from peer
            break;
        }
        delete m;
    }

    while (Comms1->available() > 0)
    {
        Message *m = Comms1->receive();
        Serial.println("Comms1: was available, but didn't receive");
        if (m == NULL)
            continue;

        switch (m->type)
        {
        case MessageType::SCAN:
            report_scans = m->payload.scan; // receive from peer
            requested_host = false;
            break;

        case MessageType::SCAN_RESULT:
            HostComms->send(*m); // forward from peer
            break;
        }
        delete m;
    }
}

// MAX Frequency RSSI BIN value of the samples
int max_rssi_x = 999;

void doScan();

void reportScan(RadioComms &c);

int16_t checkRadio(RadioComms &c);

void loop(void)
{
    r.led_flag = false;

    r.detection_count = 0;
    drone_detected_frequency_start = 0;

    checkComms();

    if (config.is_host)
    {
        if (TxComms != NULL)
        {
            // NB: swapping the use of Tx and Rx comms, so a pair of modules
            //     with identical rx/tx_lora config can talk
            int16_t status = checkRadio(*TxComms);
            if (status != RADIOLIB_ERR_NONE)
            {
                Serial.printf("Error getting a message: %d\n", status);
            }
        }
    }
    else
    {
        doScan();
        if (TxComms != NULL)
            reportScan(*TxComms);
        if (RxComms != NULL)
            checkRadio(*RxComms);
    }
}

void doScan()
{
    if (!radioIsScan)
    {
        radioIsScan = true;
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
        // display.setTextAlignment(TEXT_ALIGN_RIGHT);

        for (int i = 0; i < MAX_POWER_LEVELS; i++)
        {
            max_bins_array_value[i] = 0;
        }

        // horizontal (x axis) Frequency loop
        osd_x = 1, osd_y = 2, col = 0, max_bin = 0;

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

            setFrequency(curr_freq / 1000.0);

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
            {
                LOG("METHOD RSSI");

                float (*g)(void *);
                samples = CONF_SAMPLES;

                if (config.detection_strategy.equalsIgnoreCase("RSSI"))
                    g = &getRSSI;
                else if (config.detection_strategy.equalsIgnoreCase("CAD"))
                {
                    g = &getCAD;
                    samples = min(
                        1,
                        CONF_SAMPLES); // TODO: do we need to support values other than 1
                }
                else
                    g = &getRSSI;

                uint16_t max_rssi = r.rssiMethod(g, &r, samples, result,
                                                 RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);

                if (max_x_rssi[display_x] > max_rssi)
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
                // display.setTextAlignment(TEXT_ALIGN_CENTER);
                int16_t x1, y1;
                uint16_t w, h0;
                String s = String(r.current_frequency);
                // Measure the text dimensions
                display.getTextBounds(s.c_str(), 0, 0, &x1, &y1, &w, &h0);
                display.setCursor((display.width() / 2) - w, 0);
                display.print(s);
                display.display();

                ButtonEvent e = buttonPressEvent();

                if (e == LONG_PRESS)
                {
                    // Remove Curent Frequency Text
                    // display.setTextAlignment(TEXT_ALIGN_CENTER);
                    int16_t x1, y1;
                    uint16_t w, h0;
                    String s = String(r.current_frequency);
                    // Measure the text dimensions
                    display.getTextBounds(s.c_str(), 0, 0, &x1, &y1, &w, &h0);
                    display.setTextColor(BLACK);
                    display.setCursor((display.width() / 2) - w, 0);
                    display.print(s);
                    display.setTextColor(WHITE);
                    display.display();

                    break;
                }

                if (e == SUSPEND)
                {
                    // Visually confirm it's off so user releases button
                    display.clearDisplay();
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
                    int16_t x0, y0;
                    uint16_t w, h0;

                    // Measure the text dimensions
                    display.getTextBounds(v.c_str(), 0, 0, &x0, &y0, &w, &h0);

                    // display.setTextAlignment(TEXT_ALIGN_RIGHT);
                    //  erase old drone detection level value
                    display.setTextColor(BLACK);
                    display.fillRect(display.width() - w, 0, 13, w, BLACK);
                    display.setTextColor(WHITE);

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
                    display.setCursor(display.width(), 0);
                    display.print(v);
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
    sideBarCol = SIDEBAR_START_COL;
#endif
}

int16_t checkRadio(RadioComms &comms)
{
    radioIsScan = false;
    int16_t status = comms.configureRadio();
    if (status != RADIOLIB_ERR_NONE)
        return status;

    Message *msg = comms.receive(
        config.is_host
            ? 2000
            : 200); // 200ms should be enough to receive 500 bytes at SF 7 and BW 500
    if (msg == NULL)
    {
        return status;
    }

    if (msg->type == SCAN_RESULT)
    {
        display.clearDisplay();
        // display.setDisplayRotation(1);
        display.println("Host Mode ->");

        size_t dump_sz = msg->payload.dump.sz;

        for (int i = 0; i < dump_sz; i++)
        {
            int16_t rssi = msg->payload.dump.rssis[i];
            int16_t fr = msg->payload.dump.freqs_khz[i];
            display.println(String(fr) + ":" + String(rssi));
        }
        HostComms->send(*msg);
    }
    else
    {
        Serial.printf("Received a message of unsupported type: %d\n", msg->type);
    }

    delete msg;

    return status;
}

void reportScan(RadioComms &comms)
{
    radioIsScan = false;
    int16_t status = comms.configureRadio();
    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Failed to configure Radio: %d\n", status);
        return;
    }

    Message m;
    m.type = SCAN_RESULT;
    m.payload.dump = frequency_scan_result.dump;
    status = comms.send(m);

    m.payload.dump.sz = 0; // dump is shared, so should not delete underlying arrays

    if (status != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Failed to send scan result: %d\n", status);
        return;
    }
}
