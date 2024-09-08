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

// #define HELTEC_NO_DISPLAY

#include <Arduino.h>

//  #define OSD_ENABLED true
//  #define WIFI_SCANNING_ENABLED true
//  #define BT_SCANNING_ENABLED true

#ifndef LILYGO
#include <heltec_unofficial.h>
// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"
#elif defined(LILYGO)
// LiLyGO device does not support the auto download mode, you need to get into the
// download mode manually. To do so, press and hold the BOOT button and then press the
// RESET button once. After that release the BOOT button. Or OFF->ON together with BOOT

// Default LilyGO code
#include "utilities.h"
// Our Code
#include "LiLyGo.h"

#endif // end LILYGO

#define BT_SCAN_DELAY 60 * 1 * 1000
#define WF_SCAN_DELAY 60 * 2 * 1000
long noDevicesMillis = 0, cycleCnt = 0;
bool present = false;
bool scanFinished = true;

// time to scan BT
#define BT_SCAN_TIME 10

uint64_t wf_start = 0;
uint64_t bt_start = 0;

#define MAX_POWER_LEVELS 33
#ifdef OSD_ENABLED
#include "DFRobot_OSD.h"
#define OSD_SIDE_BAR true

static constexpr uint16_t levels[10] = {
    0x105, // 0
    0x10E, // 1
    0x10D, // 2
    0x10C, // 3
    0x10B, // 4
    0x10A, // 5
    0x109, // 6
    0x108, // 7
    0x107, // 8
    0x106, // 9
};

static constexpr uint16_t power_level[MAX_POWER_LEVELS + 1] = {
    0x10E, // 0
    0x10E, // 1
    0x10D, // 2
    0x10C, // 3
    0x10B, // 4
    0x10A, // 5
    0x109, // 6
    0x108, // 7
    0x107, // 8
    0x106, // 9 not using 106 to accent rise
    // new line
    0x10E, // 10
    0x10D, // 11
    0x10C, // 12
    0x10B, // 13
    0x10A, // 14
    0x109, // 15
    0x108, // 16
    0x107, // 17
    0x106, // 18 not using 106
    // new line
    0x10E, // 19
    0x10D, // 20
    0x10C, // 21
    0x10B, // 22
    0x10A, // 23
    0x109, // 24
    0x108, // 25
    0x107, // 26
    0x106, // 27
    0x105, // 28 ---
    0x105, // 29
    0x105, // 30
    0x105, // 31
    0x105, // 32
    0x105  // 33
};

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

/*Define Custom characters Example*/
static const int buf0[36] = {0x02, 0x80, 0x02, 0x40, 0x7F, 0xE0, 0x42, 0x00,
                             0x42, 0x00, 0x7A, 0x40, 0x4A, 0x40, 0x4A, 0x80,
                             0x49, 0x20, 0x5A, 0xA0, 0x44, 0x60, 0x88, 0x20};

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

#define SCAN_METHOD
// #define METHOD_SPECTRAL // Spectral scan method
#define METHOD_RSSI // Uncomment this and comment METHOD_SPECTRAL fot RSSI

// Output Pixel Formula
// 1 = rssi / 4, 2 = (rssi / 2) - 22 or 20
constexpr int RSSI_OUTPUT_FORMULA = 2;

// Feature to scan diapasones. Other frequency settings will be ignored.
// int SCAN_RANGES[] = {850890, 920950};
int SCAN_RANGES[] = {};

// MHZ per page
// to put everything into one page set RANGE_PER_PAGE = FREQ_END - 800
uint64_t RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN

// multiplies STEPS * N to increase scan resolution.
#define SCAN_RBW_FACTOR 2

constexpr int OSD_PIXELS_PER_CHAR = (STEPS * SCAN_RBW_FACTOR) / OSD_CHART_WIDTH;

// To Enable Multi Screen scan
//  uint64_t RANGE_PER_PAGE = 50;
//  Default Range on Menu Button Switch

#define DEFAULT_RANGE_PER_PAGE 50

// Print spectrum values pixels at once or by line
bool ANIMATED_RELOAD = false;

// TODO: Ignore max power lines
#define UP_FILTER 5
// Trim low signals - nose level
#define START_LOW 6
// Remove reading without neighbors
#define FILTER_SPECTRUM_RESULTS true
#define FILTER_SAMPLES_MIN
constexpr bool DRAW_DETECTION_TICKS = true;

// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
// if more than 100 it can freeze
#define SAMPLES 35 //(scan time = 1294)
// number of samples for RSSI method
#define SAMPLES_RSSI 20 // 21 //

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / (STEPS * SCAN_RBW_FACTOR))

uint64_t range = (int)(FREQ_END - FREQ_BEGIN);
uint64_t fr_begin = FREQ_BEGIN;
uint64_t fr_end = FREQ_BEGIN;

uint64_t iterations = RANGE / RANGE_PER_PAGE;

// uint64_t range_frequency = FREQ_END - FREQ_BEGIN;
uint64_t median_frequency = FREQ_BEGIN + FREQ_END - FREQ_BEGIN / 2;

// #define DISABLE_PLOT_CHART false   // unused

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t result_display_set[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t result_detections[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

int max_bins_array_value[MAX_POWER_LEVELS];
int max_step_range = 32;

// Waterfall array
bool waterfall[STEPS], detected_y[STEPS]; // 20 - ??? steps of the waterfall

// global variable

// Used as a Led Light and Buzzer/count trigger
bool first_run, new_pixel, detected_x = false;
// drone detection flag
bool detected = false;
uint64_t drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL;
uint64_t drone_detected_frequency_start = 0;
uint64_t drone_detected_frequency_end = 0;
uint64_t detection_count = 0;
bool single_page_scan = false;
bool SOUND_ON = false;

// #define PRINT_DEBUG
#define PRINT_PROFILE_TIME

#ifdef PRINT_PROFILE_TIME
uint64_t loop_start = 0;
uint64_t loop_time = 0;
uint64_t scan_time = 0;
uint64_t scan_start_time = 0;
#endif

uint64_t x, y, range_item, w = WATERFALL_START, i = 0;
int osd_x = 1, osd_y = 2, col = 0, max_bin = 32;
uint64_t ranges_count = 0;

float freq = 0;
int rssi = 0;
int state = 0;

#ifdef METHOD_SPECTRAL
constexpr int samples = SAMPLES;
#endif
#ifdef METHOD_RSSI
constexpr int samples = SAMPLES_RSSI;
#endif

uint8_t result_index = 0;
uint8_t button_pressed_counter = 0;
uint64_t loop_cnt = 0;

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

void osdProcess()
{ // OSD enabled

    // memset(max_step_range, 33, 30);
    max_bin = 32;

    osd.displayString(12, 1, String(FREQ_BEGIN));
    osd.displayString(12, OSD_WIDTH - 8, String(FREQ_END));
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
        max_bins_array_value[col] = result[max_bin];
#endif
    }
    // Going to the next OSD step
    if (x % osd_steps == 0 && col < OSD_WIDTH)
    {
        // OSD SIDE BAR with frequency log
#ifdef OSD_SIDE_BAR
        {
            osd.displayString(col, OSD_WIDTH - 7,
                              String(FREQ_BEGIN + (col * osd_mhz_in_bin)) + "-" +
                                  String(max_step_range) + " ");
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
}
#endif

void init_radio()
{
    // initialize SX1262 FSK modem at the initial frequency
    both.println("Init radio");
    state == radio.beginFSK(FREQ_BEGIN);

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
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
    RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));

    // and disable the data shaping
    RADIOLIB_OR_HALT(radio.setDataShaping(RADIOLIB_SHAPING_NONE));
    both.println("Starting scanning...");

    // calibrate only once ,,, at startup
    // TODO: check documentation (9.2.1) if we must calibrate in certain ranges
    radio.setFrequency(FREQ_BEGIN, true);
    delay(50);
}

void setup(void)
{
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
    loop_cnt = 0;
    bt_start = millis();
    wf_start = millis();

    pinMode(LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(REB_PIN, OUTPUT);
    heltec_setup();
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
            SOUND_ON = !SOUND_ON;
            tone(BUZZER_PIN, 205, 100);
            delay(50);
            tone(BUZZER_PIN, 205, 100);
            break;
        }
    }

    init_radio();
#ifndef LILYGO
    vbat = heltec_vbat();
    both.printf("V battery: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
#endif // end not LILYGO
#ifdef WIFI_SCANNING_ENABLED
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
#endif
#ifdef BT_SCANNING_ENABLED

#endif
    delay(400);
    display.clear();

    resolution = RANGE / (STEPS * SCAN_RBW_FACTOR);

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
    display.clear();
    Serial.println();

#ifdef METHOD_RSSI
    // TODO: try RADIOLIB_SX126X_RX_TIMEOUT_INF
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        display.drawString(0, 64 - 10, "E:startReceive");
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
}

// Formula to translate 33 bin to approximate RSSI value
int binToRSSI(int bin)
{
    // the first the strongest RSSI in bin value is 0
    return 11 + (bin * 4);
}

// returns tru if continue the code is false breake the loop
bool buttonPressHandler(float freq)
{
    // Detection level button short press
    if (button.pressedFor(100)
#ifdef JOYSTICK_ENABLED
        || joy_btn_click()
#endif
    )
    {
        button.update();
        button_pressed_counter = 0;
        // if long press stop
        while (button.pressedNow()
#ifdef JOYSTICK_ENABLED
               || joy_btn_click()
#endif
        )
        {
            delay(10);
            // Print Curent frequency
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(128 / 2, 0, String(freq));
            display.display();
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
            // Remove Curent Frequency Text
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.setColor(BLACK);
            display.drawString(128 / 2, 0, String(freq));
            display.setColor(WHITE);
            display.display();
            return false;
        }
        if (button_pressed_counter > 50 && button_pressed_counter < 150)
        {
            if (!joy_btn_clicked)
            {
                // Visually confirm it's off so user releases button
                display.displayOff();
                // Deep sleep (has wait for release so we don't wake up
                // immediately)
                heltec_deep_sleep();
            }
            return false;
        }
        button.update();
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        // erase old drone detection level value
        display.setColor(BLACK);
        display.fillRect(128 - 13, 0, 13, 13);
        display.setColor(WHITE);
        drone_detection_level++;
        // print new value
        display.drawString(128, 0, String(drone_detection_level));
        tone(BUZZER_PIN, 104, 150);
        if (drone_detection_level > 30)
        {
            drone_detection_level = 1;
        }
    }
    return true;
}

void drone_sound_alarm(int drone_detection_level, int detection_count)
{
    // If level is set to sensitive,
    // start beeping every 10th frequency and shorter
    // it improves performance less short beep delays...
    if (drone_detection_level <= 25)
    {
        if (detection_count == 1 && SOUND_ON)
        {
            tone(BUZZER_PIN, 205,
                 10); // same action ??? but first time
        }
        if (detection_count % 5 == 0 && SOUND_ON)
        {
            tone(BUZZER_PIN, 205,
                 10); // same action ??? but every 5th time
        }
    }
    else
    {
        if (detection_count % 20 == 0 && SOUND_ON)
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
        display.drawString(cursor_x_position, 0, String((int)freq));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10);
        display.display();
        delay(10);
    }
    else if (joy_x_pressed < 0)
    {
        cursor_x_position++;
        display.drawString(cursor_x_position, 0, String((int)freq));
        display.drawLine(cursor_x_position, 1, cursor_x_position, 10);
        display.display();
        delay(10);
    }
    if (cursor_x_position > DISPLAY_WIDTH || cursor_x_position < 0)
    {
        cursor_x_position = 0;
        display.drawString(cursor_x_position, 0, String((int)freq));
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

void check_ranges()
{
    if (RANGE_PER_PAGE == range)
    {
        single_page_scan = true;
    }
    else
    {
        single_page_scan = false;
    }

    for (int range : SCAN_RANGES)
    {
        ranges_count++;
    }

    if (ranges_count > 0)
    {
        iterations = ranges_count;
        single_page_scan = false;
    }
}
// MAX Frequency RSSI value of the samples
int max_rssi_x = 999;

void loop(void)
{
    UI_displayDecorate(0, 0, false); // some default values

    detection_count = 0;
    drone_detected_frequency_start = 0;
    ranges_count = 0;

    // reset scan time
    scan_time = 0;

    // general purpose loop counter
    loop_cnt++;

#ifdef PRINT_PROFILE_TIME
    loop_start = millis();
#endif

    if (!ANIMATED_RELOAD || !single_page_scan)
    {
        // clear the scan plot rectangle
        UI_clearPlotter();
    }

    // do the scan
    range = FREQ_END - FREQ_BEGIN;
    if (RANGE_PER_PAGE > range)
    {
        RANGE_PER_PAGE = range;
    }

    fr_begin = FREQ_BEGIN;
    fr_end = fr_begin;

    // 50 is a single-screen range
    // TODO: Make 50 a variable with the option to show the full range
    iterations = range / RANGE_PER_PAGE;

#if 0 // disabled code
    if (range % RANGE_PER_PAGE != 0)
    {
        // add more scan
        //++;
    }
#endif

    check_ranges();

    // Iterating by small ranges by 50 Mhz each pixel is 0.4 Mhz
    for (range_item = 0; range_item < iterations; range_item++)
    {
        range = RANGE_PER_PAGE;
        if (ranges_count == 0)
        {
            fr_begin = (range_item == 0) ? fr_begin : fr_begin += range;
            fr_end = fr_begin + RANGE_PER_PAGE;
        }
        else
        {
            fr_begin = SCAN_RANGES[range_item] / 1000;
            fr_end = SCAN_RANGES[range_item] % 1000;
            range = fr_end - fr_begin;
        }

#ifdef DISABLED_CODE
        if (!ANIMATED_RELOAD || !single_page_scan)
        {
            // clear the scan plot rectangle
            UI_clearPlotter();
        }
#endif

        if (single_page_scan == false)
        {
            UI_displayDecorate(fr_begin, fr_end, true);
        }

        drone_detected_frequency_start = 0;
        display.setTextAlignment(TEXT_ALIGN_RIGHT);

        for (int i = 0; i < MAX_POWER_LEVELS; i++)
        {
            max_bins_array_value[i] = 0;
        }

        // horizontal (x axis) Frequency loop
        osd_x = 1, osd_y = 2, col = 0, max_bin = 0;
        // x loop
        for (x = 0; x < STEPS * SCAN_RBW_FACTOR; x++)
        {
            new_pixel = is_new_x_pixel(x);
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
            waterfall[display_x] = false;
            float step = (range * ((float)x / (STEPS * SCAN_RBW_FACTOR)));

            freq = fr_begin + step;
#ifdef PRINT_DEBUG
            Serial.println("setFrequency:" + String(freq));
#endif

#ifdef LILYGO
            state = radio.setFrequency(freq, false); // false = no calibration need here
#else
            state = radio.setFrequency(freq, false); // false = no calibration need here
#endif
            int radio_error_count = 0;
            if (state != RADIOLIB_ERR_NONE)
            {
                display.drawString(0, 64 - 10, "E:setFrequency:" + String(freq));
                // display.drawString(0, 64 - 10, "E:setFrequency:" + String(freq));
                Serial.println("E:setFrequency:" + String(freq));
                display.display();
                delay(2);
                radio_error_count++;
                if (radio_error_count > 10)
                    continue;
            }

#ifdef PRINT_DEBUG
            Serial.printf("Step:%d Freq: %f\n", x, freq);
#endif
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
#ifdef PRINT_DEBUG
                Serial.println("METHOD RSSI");
#endif
                // memset
                // memset(result, 0, RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);
                // Some issues with memset function
                for (i = 0; i < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; i++)
                {
                    result[i] = 0;
                }
                result_index = 0;
                // N of samples
                for (int r = 0; r < SAMPLES_RSSI; r++)
                {
                    rssi = radio.getRSSI(false);
                    // ToDO: check if 4 is correct value for 33 power bins
                    // Now we have more space because we are ignoring low dB values
                    // we can  / 3 default 4
                    if (RSSI_OUTPUT_FORMULA == 1)
                    {
                        result_index =
                            /// still not clear formula but it works
                            uint8_t(abs(rssi) / 4);
                    }
                    else if (RSSI_OUTPUT_FORMULA == 2)
                    {
                        // I like this formula better
                        result_index = uint8_t(abs(rssi) / 2) - 22;
                    }

#ifdef PRINT_DEBUG
                    Serial.printf("RSSI: %d IDX: %d\n", rssi, result_index);
#endif
                    // avoid buffer overflow
                    if (result_index < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE)
                    {
                        // Saving max value only rss is negative so smaller is bigger
                        if (result[result_index] > rssi)
                        {
                            result[result_index] = rssi;
                        }
                    }
                    else
                    {
                        Serial.print("Out-of-Range: result_index %d\n");
                    }
                }
            }
#endif // SCAN_METHOD == METHOD_RSSI

            // if this code is not executed LORA radio doesn't work
            // basically SX1262 requires delay
            // osd.displayString(12, 1, String(FREQ_BEGIN));
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
            detected = false;
            detected_y[display_x] = false;
            max_rssi_x = 999;

            for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
            {

#ifdef PRINT_DEBUG
                Serial.print(String(y) + ":");
                Serial.print(String(result[y]) + ",");
#endif
#if !defined(FILTER_SPECTRUM_RESULTS) || FILTER_SPECTRUM_RESULTS == false
                if (result[y] && result[y] != 0)
                {
                    filtered_result[y] = 1;
                }
                else
                {
                    filtered_result[y] = 0;
                }
#endif

// if samples low ~1 filter removes all values
#if FILTER_SPECTRUM_RESULTS

                filtered_result[y] = 0;
                // Filter Elements without neighbors
                // if RSSI method actual value is -xxx dB
                if (result[y] > 0 && samples > 1)
                {
                    // do not process 'first' and 'last' row to avoid out of index
                    // access.
                    if ((y > 0) && (y != (RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE - 3)))
                    {
                        if (((result[y + 1] != 0) && (result[y + 2] != 0)) ||
                            (result[y - 1] != 0))
                        {
                            filtered_result[y] = 1;
                        }
                        else
                        {
#ifdef PRINT_DEBUG
                            Serial.print("Filtered:" + String(x) + ":" + String(y) + ",");
#endif
                        }
                    }
                } // not filtering if samples == 1
                else if (result[y] > 0 && samples == 1)
                {
                    filtered_result[y] = 1;
                }

#endif
                // check if we should alarm about a drone presence
                if ((filtered_result[y] == 1) // we have some data and
                    && (y <= drone_detection_level) &&
                    detected_y[display_x] == false) // detection threshold match
                {
                    // Set LED to ON (filtered in UI component)
                    UI_setLedFlag(true);
#if (WATERFALL_ENABLED == true)
                    if (single_page_scan)
                    {
                        // Drone detection true for waterfall
                        if (!waterfall[display_x])
                        {
                            waterfall[display_x] = true;
                            display.setColor(WHITE);
                            display.setPixel(display_x, w);
                        }
                    }
#endif
                    if (drone_detected_frequency_start == 0)
                    {
                        // mark freq start
                        drone_detected_frequency_start = freq;
                    }

                    // mark freq end ... will shift right to last detected range
                    drone_detected_frequency_end = freq;
                    if (SOUND_ON == true)
                    {
                        drone_sound_alarm(drone_detection_level, detection_count);
                    }

                    if (DRAW_DETECTION_TICKS == true)
                    {
                        // draw vertical line on top of display for "drone detected"
                        // frequencies
                        if (!detected_y[display_x])
                        {
                            display.drawLine(display_x, 1, display_x, 4);
                            detected_y[display_x] = true;
                        }
                    }
                }
#if (WATERFALL_ENABLED == true)
                if ((filtered_result[y] == 1) && (y <= drone_detection_level) &&
                    (single_page_scan) && (waterfall[display_x] != true) && new_pixel)
                {
                    // If drone not found set dark pixel on the waterfall
                    // TODO: make something like scrolling up if possible
                    waterfall[display_x] = false;
                    display.setColor(BLACK);
                    display.setPixel(display_x, w);
                    display.setColor(WHITE);
                }
#endif
                // next 2 If's ... adds !!!! 10ms of runtime ......tfk ???
                if (filtered_result[y] == 1)
                {
#ifdef PRINT_DEBUG
                    Serial.print("Pixel:" + String(display_x) + "(" + String(x) + ")" +
                                 ":" + String(y) + ",");
#endif
                    if (max_rssi_x > y)
                    {
                        max_rssi_x = y;
                    }
                    // Set signal level pixel
                    if (y < MAX_POWER_LEVELS - START_LOW)
                    {
                        display.setPixel(display_x, y + START_LOW);
                    }
                    if (!detected)
                    {
                        detected = true;
                    }
                }

                // -------------------------------------------------------------
                // Draw "Detection Level line" every 2 pixel
                // -------------------------------------------------------------
                if ((y == drone_detection_level) && (display_x % 2 == 0))
                {
                    display.setColor(WHITE);
                    if (filtered_result[y] == 1)
                    {
                        display.setColor(INVERSE);
                    }
                    display.setPixel(display_x, y + START_LOW);
                    // display.setPixel(display_x, y + START_LOW - 1); // 2 px wide

                    display.setColor(WHITE);
                }
            }

#ifdef JOYSTICK_ENABLED
            // Draw joystick cursor and Frequency RSSI value
            if (display_x == cursor_x_position)
            {
                display.drawString(display_x - 1, 0, String((int)freq));
                display.drawLine(display_x, 1, display_x, 12);
                // if method scan RSSI we can get exact RSSI value
                display.drawString(display_x + 17, 0, "-" + String((int)max_rssi_x * 4));
            }
#endif

#ifdef PRINT_PROFILE_TIME
            scan_time += (millis() - scan_start_time);
#endif
            // count detected
            if (detected)
            {
                detection_count++;
            }

#ifdef PRINT_DEBUG
            Serial.println("....\n");
#endif
            if (first_run || ANIMATED_RELOAD)
            {
                display.display();
            }

// LiLyGo doesn't have button ;(
// ToDO: Check if we use BOOT button
#ifndef LILYGO
            if (buttonPressHandler() == false)
                break;
#endif // END LILYGO

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
#if (WATERFALL_ENABLED == true)
        // Draw waterfall position cursor
        if (single_page_scan)
        {
            display.setColor(BLACK);
            display.drawHorizontalLine(0, w, STEPS);
            display.setColor(WHITE);
        }
#endif
        // Render display data here
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

    loop_time = millis() - loop_start;
    joy_btn_clicked = false;

#ifdef PRINT_PROFILE_TIME
    Serial.printf("LOOP: %lld ms; SCAN: %lld ms;\n  ", loop_time, scan_time);
#endif
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
#endif
}
