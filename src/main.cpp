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

#include <Arduino.h>
#include <heltec_unofficial.h>
// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"
#define OSD_ENABLED true

#ifdef OSD_ENABLED
#include "DFRobot_OSD.h"
#define MAX_POWER_LEVELS 33

static const uint16_t power_level[MAX_POWER_LEVELS] = {
    0x10E, // 0
    0x10E, // 1
    0x10D, // 2
    0x10C, // 3
    0x10B, // 4
    0x10A, // 5
    0x109, // 6
    0x108, // 7
    0x107, // 8
    0x106, // 9
    // new line
    0x10E, // 10
    0x10D, // 11
    0x10C, // 12
    0x10B, // 13
    0x10A, // 14
    0x109, // 15
    0x108, // 16
    0x107, // 17
    0x106, // 18
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
    0x105, // 28
    0x105, // 29
    0x105, // 30
    0x105, // 31
    0x105  // 32
};
#endif
// SPI pins
#define CS 47
#define OSD_MISO 33
#define OSD_MOSI 34
#define OSD_SCK 26

#define OSD_WIDTH 30
#define OSD_HEIGHT 16
#define OSD_CHART_WIDTH 15
#define OSD_CHART_HEIGHT 5
#define OSD_X_START 1
#define OSD_Y_START 16

// TODO: Calculate dinammicaly:
// osd_steps = osd_mhz_in_bin / (FM range / LORA radio x Steps)
int osd_mhz_in_bin = 5;
int osd_steps = 12;
int global_counter = 0;

#ifdef OSD_ENABLED
DFRobot_OSD osd(CS);
#endif

/*Define Custom characters Example*/
static const int buf0[36] = {0x02, 0x80, 0x02, 0x40, 0x7F, 0xE0, 0x42, 0x00,
                             0x42, 0x00, 0x7A, 0x40, 0x4A, 0x40, 0x4A, 0x80,
                             0x49, 0x20, 0x5A, 0xA0, 0x44, 0x60, 0x88, 0x20};

// project components
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
#define METHOD_SPECTRAL
// #define METHOD_RSSI // Uncomment this and comment METHOD_SPECTRAL fot RSSI

// Feature to scan diapazones. Other frequency settings will be ignored.
// int SCAN_RANGES[] = {850890, 920950};
int SCAN_RANGES[] = {};

// MHZ per page
// to put everething into one page set RANGE_PER_PAGE = FREQ_END - 800
uint64_t RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN

// multiplies STEPS * N to increase scan resolution.
#define SCAN_RBW_RFACTOR 2

int OSD_PIXELS_PER_CHAR = (STEPS * SCAN_RBW_RFACTOR) / OSD_CHART_WIDTH;

// To Enable Multi Screen scan
//  uint64_t RANGE_PER_PAGE = 50;
//  Default Range on Menu Button Switch

#define DEFAULT_RANGE_PER_PAGE 50

// TODO: Ignore power lines
#define UP_FILTER 5
#define LOW_FILTER 3
// Remove reading without neighbors
#define FILTER_SPECTRUM_RESULTS true
#define DRAW_DETECTION_TICKS true

// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
// if more than 100 it can freez
#define SAMPLES 100 //(scan time = 1294)
// number of samples for RSSI method
#define SAMPLES_RSSI 21 // 21 //

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / (STEPS * SCAN_RBW_RFACTOR))

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
uint16_t max_bins_array[OSD_WIDTH];
int max_bins_array_value[OSD_WIDTH];

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

uint64_t x, y, range_item, w, i = 0;
int osd_x = 1, osd_y = 2;
uint64_t ranges_count = 0;

float freq = 0;
int rssi = 0;
int state = 0;
uint8_t result_index = 0;

uint8_t button_pressed_counter = 0;

uint64_t loop_cnt = 0;

unsigned short selectFreqChar(int bin)
{
    if (bin >= 0 && bin < MAX_POWER_LEVELS)
        return power_level[bin];
    return 0x121;
}

void setup(void)
{
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

    pinMode(LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(REB_PIN, OUTPUT);
    heltec_setup();
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

    // initialize SX1262 FSK modem at the initial frequency
    both.println("Init radio");
    RADIOLIB_OR_HALT(radio.beginFSK(FREQ_BEGIN));

    // upload a patch to the SX1262 to enable spectral scan
    // NOTE: this patch is uploaded into volatile memory,
    // and must be re-uploaded on every power up
    both.println("Upload SX1262 patch");

    // Upload binary patch into the SX126x device RAM. Patch is needed to e.g.,
    // enable spectral scan and must be uploaded again on every power cycle.
    RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
    // configure scan bandwidth and disable the data shaping

    both.println("Setting up radio");
    RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));

    // and disable the data shaping
    RADIOLIB_OR_HALT(radio.setDataShaping(RADIOLIB_SHAPING_NONE));
    both.println("Starting scanning...");
    vbat = heltec_vbat();
    both.printf("V battery: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
    delay(400);
    display.clear();

    resolution = RANGE / (STEPS * SCAN_RBW_RFACTOR);

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
        both.println("Multi Screan Res: " + String(resolution) + "Mhz/tick");
        both.println(
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_RFACTOR)) +
            "Mhz/tick");
        for (int i = 0; i < 500; i++)
        {
            button.update();
            delay(10);
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
            "Resolution: " + String((float)RANGE_PER_PAGE / (STEPS * SCAN_RBW_RFACTOR)) +
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

    // calibrate only once ,,, at startup
    // TODO: check documentation (9.2.1) if we must calibrate in certain ranges
    radio.setFrequency(FREQ_BEGIN, true);

#ifdef METHOD_RSSI
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        Serial.println(state);
    }
#endif
    // waterfall start line y-axis
    w = WATERFALL_START;
}

void osdPrintSignalLevelChart(int col, int signal_value)
{
    // Third line
    if (signal_value <= 7)
    {
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(12, col + 2, selectFreqChar(signal_value));
    }
    // Second line
    else if (max_bins_array[col] < 17)
    {
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(14, col + 2, 0x100);
        osd.displayChar(13, col + 2, selectFreqChar(signal_value));
    }
    // First line
    else
    {
        // Clean Up symbol
        osd.displayChar(12, col + 2, 0x100);
        osd.displayChar(13, col + 2, 0x100);
        osd.displayChar(14, col + 2, selectFreqChar(signal_value));
    }
}

// Formula to translate 33 bin to aproximate RSSI value
int binToRSSI(int bin) { return bin * 4; }

void loop(void)
{
    UI_displayDecorate(0, 0, false); // some default values
    detection_count = 0;
    drone_detected_frequency_start = 0;
    ranges_count = 0;

    // reset scan time
    scan_time = 0;

    // general purpose loop conter
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

        if (!ANIMATED_RELOAD || !single_page_scan)
        {
            // clear the scan plot rectangle
            UI_clearPlotter();
        }

        if (single_page_scan == false)
        {
            UI_displayDecorate(fr_begin, fr_end, true);
        }

        drone_detected_frequency_start = 0;
        display.setTextAlignment(TEXT_ALIGN_RIGHT);

        // horizontal (x axis) Frequency loop
        int osd_x = 1, osd_y = 1, col = 0, max_bin = 0;
        // x loop
        for (x = 0; x < STEPS * SCAN_RBW_RFACTOR; x++)
        {

            if (x % SCAN_RBW_RFACTOR == 0)
                new_pixel = true;
            else
                new_pixel = false;
#if ANIMATED_RELOAD
            UI_drawCursor(x);
#endif

#ifdef PRINT_PROFILE_TIME
            scan_start_time = millis();
#endif
            // Real display pixel x - axis.
            // Because of the SCAN_RBW_RFACTOR x is not a display coordinate anymore
            // x > STEPS on SCAN_RBW_RFACTOR
            int dispaly_x = x / SCAN_RBW_RFACTOR;
            waterfall[dispaly_x] = false;
            float step = (range * ((float)x / (STEPS * SCAN_RBW_RFACTOR)));

            freq = fr_begin + step;

            radio.setFrequency(freq, false); // false = no calibration need here

#ifdef PRINT_DEBUG
            Serial.printf("Step:%d Freq: %f\n", x, freq);
#endif
            // SpectralScan Method
#ifdef METHOD_SPECTRAL
            {
                // start spectral scan third parameter is a sleep interval
                radio.spectralScanStart(SAMPLES, 1);
                // wait for spectral scan to finish
                while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE)
                {
                    Serial.print("radio.spectralScanGetStatus ERROR: ");
                    Serial.println(radio.spectralScanGetStatus());
                    heltec_delay(ONE_MILLISEC);
                }
                // read the results Array to which the results will be saved
                radio.spectralScanGetResult(result);
            }
#endif
#ifdef METHOD_RSSI
// Spectrum analyzer using getRSSI
#ifdef PRINT_DEBUG
            Serial.println("METHOD RSSI");
#endif
            {

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
                    // delay(ONE_MILLISEC);
                    // ToDO: check if 4 is correct value for 33 power bins
                    result_index = uint8_t(abs(rssi) / 4); /// still not clear formula

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

#ifdef OSD_ENABLED
            { // OSD enabled

                for (int i = 0; i < OSD_WIDTH; i++)
                {
                    max_bins_array[i] = 33;
                    max_bins_array_value[i] = 0;
                }
                // memset(max_bins_array, 33, 30);
                max_bin = 0;

                osd.displayString(12, 1, String(FREQ_BEGIN));
                osd.displayString(12, 30 - 8, String(FREQ_END));
                for (int i = 1; i < 32; i++)
                {
                    if (result[i] > 0 && (result[i + 1] > 0))
                    {
                        max_bin = i;
#ifdef PRINT_DEBUG
                        Serial.print("MAX in bin:" + String(max_bin));
                        Serial.println();
#endif
                        break;
                    }
                }
                if (max_bins_array[col] > max_bin)
                {
                    max_bins_array[col] = max_bin;
                    // Store RSSI value for RSSI Method
                    max_bins_array_value[col] = result[max_bin];
                }
                // Going to the next OSD step
                if (x % osd_steps == 0 && col < 30)
                {
                    // OSD SIDE BAR
                    if (true)
                    {
                        osd.displayString(col, 30 - 7,
                                          String(FREQ_BEGIN + (col * osd_mhz_in_bin)) +
                                              ":" + String(max_bins_array[col]));
                    }
                    // Test with Random Result...
                    // max_bins_array[s] = rand() % 32;
#ifdef METHOD_RSSI
                    // With THe RSSI method we can get real RSSI value not just a bin
#endif
                    // PRINT SIGNAL CHAR ROW, COL, VALUE
                    osdPrintSignalLevelChart(col, max_bins_array[col]);

#ifdef PRINT_DEBUG
                    Serial.println("MAX:" + String(max_bins_array[s]));
#endif
                    col++;
                }
            }
#endif // END OSD ENABLED
            detected = false;
            detected_y[dispaly_x] = false;

            for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
            {

#ifdef PRINT_DEBUG
                Serial.print(String(y) + ":");
                Serial.print(String(result[y]) + ",");
#endif

#if FILTER_SPECTRUM_RESULTS == false
                if (result[y] && result[y] != 0)
                {
                    filtered_result[y] = 1;
                }
                else
                {
                    filtered_result[y] = 0;
                }
#endif

#if FILTER_SPECTRUM_RESULTS

                filtered_result[y] = 0;
                // Filter Elements without neighbors
                // if RSSI method actual value is -xxx dB
                if (result[y])
                {
                    // do not process 'first' and 'last' row to avoid out of index
                    // access
                    if ((y != 0) && (y != (RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE - 1)))
                    {
                        if ((result[y + 1] != 0) || (result[y - 1] != 0))
                        {
                            filtered_result[y] = 1;
                        }
                    }
                }
#endif

                // if (result[y] || y == drone_detection_level)
                {
                    // check if we should alarm about a drone presence
                    if ((filtered_result[y] == 1) // we have some data and
                        && (y <= drone_detection_level) &&
                        detected_y[dispaly_x] == false) // detection threshold match
                    {

                        // Set LED to ON (filtered in UI component)
                        UI_setLedFlag(true);

#if (WATERFALL_ENABLED == true)
                        if (single_page_scan)
                        {
                            // Drone detection true for waterfall
                            if (!waterfall[dispaly_x])
                            {
                                waterfall[dispaly_x] = true;
                                display.setColor(WHITE);
                                display.setPixel(dispaly_x, w);
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
                                     10); // same action ??? but everey 5th time
                            }
                        }
                        else
                        {
                            if (detection_count % 20 == 0 && SOUND_ON)
                            {
                                tone(BUZZER_PIN, 205,
                                     10); // same action ??? but everey 20th detection
                            }
                        }

#if (DRAW_DETECTION_TICKS == true)
                        // draw vertical line on top of display for "drone detected"
                        // frequencies
                        if (!detected_y[dispaly_x])
                        {
                            display.drawLine(dispaly_x, 1, dispaly_x, 6);
                            detected_y[dispaly_x] = true;
                        }
#endif
                    }

#if (WATERFALL_ENABLED == true)
                    if ((filtered_result[y] == 1) && (y < drone_detection_level) &&
                        (single_page_scan) && (waterfall[dispaly_x] != true) && new_pixel)
                    {
                        // If drone not found set dark pixel on the waterfall
                        // TODO: make something like scrolling up if possible
                        waterfall[dispaly_x] = false;
                        display.setColor(BLACK);
                        display.setPixel(dispaly_x, w);
                        display.setColor(WHITE);
                    }
#endif

#if 0

#endif // If 0

                    // next 2 If's ... adds !!!! 10ms of runtime ......tfk ???
                    if (filtered_result[y] == 1)
                    {
                        // Set signal level pixel
                        display.setPixel(dispaly_x, y);
                        if (!detected)
                        {
                            detected = true;
                        }
                    }

                    // -------------------------------------------------------------
                    // Draw "Detection Level line" every 2 pixel
                    // -------------------------------------------------------------
                    if ((y == drone_detection_level) && (dispaly_x % 2 == 0))
                    {
                        display.setColor(WHITE);
                        if (filtered_result[y] == 1)
                        {
                            display.setColor(INVERSE);
                        }
                        display.setPixel(dispaly_x, y);
                        display.setPixel(dispaly_x, y - 1); // 2 px wide

                        display.setColor(WHITE);
                    }
                }
            }

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

            // Detection level button short press
            if (button.pressedFor(100))
            {
                button.update();
                button_pressed_counter = 0;
                // if long press stop
                while (button.pressedNow())
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
                    // Remove Curent Freqancy Text
                    display.setTextAlignment(TEXT_ALIGN_CENTER);
                    display.setColor(BLACK);
                    display.drawString(128 / 2, 0, String(freq));
                    display.setColor(WHITE);
                    display.display();
                    break;
                }
                if (button_pressed_counter > 50 && button_pressed_counter < 150)
                {
                    // Visually confirm it's off so user releases button
                    display.displayOff();
                    // Deep sleep (has wait for release so we don't wake up
                    // immediately)
                    heltec_deep_sleep();
                    break;
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
            // wait a little bit before the next scan,
            // otherwise the SX1262 hangs
            // Add more logic before insead of long delay...
            // heltec_delay(1);
            // Loop is needed if heltec_delay(1) not used
            heltec_loop();
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
        if (global_counter != 0 && global_counter % 50 == 0)
        {
            osd.clear();
            osd.displayChar(14, 1, 0x10f);
            global_counter = 0;
        }
        global_counter++;
#endif
    }

#ifdef PRINT_DEBUG
    // Serial.println("----");
#endif

    loop_time = millis() - loop_start;

#ifdef PRINT_PROFILE_TIME
    Serial.printf("LOOP: %lld ms; SCAN: %lld ms;\n  ", loop_time, scan_time);
#endif
}
