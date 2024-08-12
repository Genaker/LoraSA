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

#define SCAN_METHOD METHOD_SPECTRAL

// Feature to scan diapazones. Other frequency settings will be ignored.
// int SCAN_RANGES[] = {850890, 920950};
uint32_t SCAN_RANGES[] = {};

// MHZ per page
// to put everething into one page set RANGE_PER_PAGE = FREQ_END - 800
uint64_t RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN

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
#define SAMPLES_RSSI RADIOLIB_SX126X_SPECTRAL_SCAN_WINDOW_DEFAULT // 21 //

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / STEPS)

uint64_t range = (int)(FREQ_END - FREQ_BEGIN);
uint64_t fr_begin = FREQ_BEGIN;
uint64_t fr_end = FREQ_BEGIN;

uint64_t iterations = RANGE / RANGE_PER_PAGE;

// uint64_t range_frequency = FREQ_END - FREQ_BEGIN;
uint64_t median_frequency = FREQ_BEGIN + FREQ_END - FREQ_BEGIN / 2;

// #define OSD_ENABLED true           // unused
// #define DISABLE_PLOT_CHART false   // unused

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

// Waterfall array
bool waterfall[10][STEPS][10]; // 10 - ??? steps of the waterfall

// global variable

// Used as a Led Light and Buzzer/count trigger
bool first_run = false;
// drone detection flag
bool detected = false;
uint64_t drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL;
uint64_t drone_detected_frequency_start = 0;
uint64_t drone_detected_frequency_end = 0;
uint64_t detection_count = 0;
bool single_page_scan = false;
bool SOUND_ON = true;

#define PRINT_PROFILE_TIME

#ifdef PRINT_PROFILE_TIME
uint64_t loop_start = 0;
uint64_t loop_time = 0;
uint64_t scan_time = 0;
uint64_t scan_start_time = 0;
#endif

uint64_t x, y, range_item, w = 0;
uint64_t ranges_count = 0;

float freq = 0;
int rssi = 0;
int state = 0;
uint8_t result_index = 0;

uint8_t button_pressed_counter = 0;

uint64_t loop_cnt = 0;

void setup(void)
{
    float vbat;
    float resolution = (float)RANGE / (float)STEPS;
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
            SOUND_ON = false;
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
    delay(300);
    display.clear();

    single_page_scan = (RANGE_PER_PAGE == range);

#ifdef DISABLED_CODE
    // Adjust range if it is not even to RANGE_PER_PAGE
    if (!single_page_scan && range % RANGE_PER_PAGE != 0)
    {
        // range = range + range % RANGE_PER_PAGE;
    }
#endif

    if (single_page_scan)
    {
        both.println("Mode: SINGLE page scan");
        both.println("Scr Res: " + String(resolution) + "Mhz/tick");
        // both.println("-Cur.Res: " + String((float)RANGE_PER_PAGE / STEPS) +
        // "Mhz/tick");
        both.println("Multi Screen View Press P - button");

        for (uint16_t i = 500; i > 1; i--)
        {
            button.update();
            delay(10);
            if ((i % 23) == 0)
            {
                display.print("#");
            }

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
        both.println("Mode: MULTI page scan");
        both.println("Single screen View Press P - button");
        // both.println("Single screen Resolution: " + String(resolution) + "Mhz/tick");
        both.println("Curent Resolution: " + String((float)RANGE_PER_PAGE / STEPS) +
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

    // waterfall start line y-axis
    w = WATERFALL_START;

    // count nr of ranges
    ranges_count = sizeof(SCAN_RANGES) / sizeof(uint32_t);
}

void loop(void)
{
    UI_displayDecorate(0, 0, false); // some default values
    detection_count = 0;
    drone_detected_frequency_start = 0;

    // general purpose loop conter
    loop_cnt++;

#ifdef PRINT_PROFILE_TIME
    // reset scan time
    scan_time = 0;
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

    single_page_scan = (RANGE_PER_PAGE == range);

    if (ranges_count > 0)
    {
        iterations = ranges_count;
        single_page_scan = false;
    }

    // only one itteration at this momment
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
            // Iterating by small ranges, each pixel is (RANGE_PER_PAGE/STEPS) Mhz
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

        // horizontal x axis loop
        for (x = 0; x < STEPS; x++)
        {
#if ANIMATED_RELOAD
            UI_drawCursor(x);
#endif

#ifdef PRINT_PROFILE_TIME
            scan_start_time = millis();
#endif

            waterfall[range_item][x][w] = false;
            freq = fr_begin + (range * ((float)x / STEPS));

            radio.setFrequency(freq, false); // false = no calibration need here

#ifdef PRINT_DEBUG
            // Serial.printf("Step:%d Freq: %f\n",x,freq);
#endif
            // SpectralScan Method
#if SCAN_METHOD == METHOD_SPECTRAL
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
#if SCAN_METHOD == METHOD_RSSI
            // Spectrum analyzer using getRSSI
            {
                state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
                if (state != RADIOLIB_ERR_NONE)
                {
                    Serial.print(F("Failed to start receive mode, error code: "));
                    Serial.println(state);
                }

                // memset
                memset(result, 0, RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE);
                result_index = 0u;
                // N of samples
                for (int r = 0; r < SAMPLES_RSSI; r++)
                {
                    rssi = radio.getRSSI(false);
                    // delay(ONE_MILLISEC);
                    // ToDO: check if 4 is correct value for 33 power bins
                    result_index = uint8_t(abs(rssi) / 4); /// still not clear formula

#ifdef PRINT_DEBUG
                    // Serial.printf("RSSI: %d IDX: %d\n",rssi,result_index);
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
#ifdef PRINT_DEBUG
                    else
                    {
                        Serial.print("Out-of-Range: result_index %d\n");
                    }
#endif
                }
            }
#endif // SCAN_METHOD == METHOD_RSSI

            detected = false;

            for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
            {
#ifdef PRINT_DEBUG
                // Serial.printf("%04X,", result[y]);
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
                    // do not process 'first' and 'last' row to avoid out of index access
                    if ((y != 0) && (y != (RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE - 1)))
                    {
                        if ((result[y + 1] != 0) || (result[y - 1] != 0))
                        {
                            // Filling the empty pixel between signals int the level < 27
                            // (noise level)
                            /* if (y < 27 && result[y + 1] == 0 && result[y + 2] > 0)
                                {
                                result[y + 1] = 1;
                                filtered_result[y + 1] = 1;
                                }*/
                            filtered_result[y] = 1;
                        }
                    }
                }
#endif

                // if (result[y] || y == drone_detection_level)
                {
                    // check if we should alarm about a drone presence
                    if ((filtered_result[y] == 1)        // we have some data and
                        && (y <= drone_detection_level)) // detection threshold match
                    {

                        // Set LED to ON (filtered in UI component)
                        UI_setLedFlag(true);

#if (WATERFALL_ENABLED == true)
                        if (single_page_scan)
                        {
                            // Drone detection true for waterfall
                            waterfall[range_item][x][w] = true;
                            display.setColor(WHITE);
                            display.setPixel(x, w);
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
                        display.drawLine(x, 1, x, 6);
#endif
                    }

#if (WATERFALL_ENABLED == true)
                    if ((filtered_result[y] == 1) && (y > drone_detection_level) &&
                        (single_page_scan) && (waterfall[range_item][x][w] != true))
                    {
                        // If drone not found set dark pixel on the waterfall
                        // TODO: make something like scrolling up if possible
                        waterfall[range_item][x][w] = false;
                        display.setColor(BLACK);
                        display.setPixel(x, w);
                        display.setColor(WHITE);
                    }
#endif

#if 0

#endif // If 0

                    // next 2 If's ... adds !!!! 10ms of runtime ......tfk ???
                    if (filtered_result[y] == 1)
                    {
                        // Set signal level pixel
                        display.setPixel(x, y);
                        if (!detected)
                            detected = true;
                    }

                    // -------------------------------------------------------------
                    // Draw "Detection Level line" every 2 pixel
                    // -------------------------------------------------------------
                    if ((y == drone_detection_level) && (x % 2 == 0))
                    {
                        display.setColor(INVERSE);
                        display.setPixel(x, y);
                        display.setPixel(x, y + 1); // 2 px wide
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
            // Serial.println("....");
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
                    // Deep sleep (has wait for release so we don't wake up immediately)
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
    }

#ifdef PRINT_DEBUG
    // Serial.println("----");
#endif

    loop_time = millis() - loop_start;

#ifdef PRINT_PROFILE_TIME
#ifdef PRINT_DEBUG
    Serial.printf("LOOP: %lld ms; SCAN: %lld ms;\n  ", loop_time, scan_time);
#endif
#endif
}
