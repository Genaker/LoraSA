/* Heltec Automation Ink screen example
 * NOTE!!!: to upload we new code you need to press button BOOT and RESET or you will
 * have serial error. After upload you need reset device...
 *
 * Description:
 * 1.Inherited from ssd1306 for drawing points, lines, and functions
 *
 * All code e link examples you cand find here:
 * */
// Variables required to boot Heltec E290 defined at platformio.ini
// #define HELTEC_BOARD 37
// #define SLOW_CLK_TPYE 1
// #define ARDUINO_USB_CDC_ON_BOOT 1
// #define LoRaWAN_DEBUG_LEVEL 0
#include "HT_DEPG0290BxS800FxX_BW.h"
#include "global_config.h"
#include "images.h"
#include "ui.h"
#include <Arduino.h>

// Disabling default Heltec lib OLED display
#define HELTEC_NO_DISPLAY
#define DISPLAY_WIDTH 296
#define DISPLAY_HEIGHT 128
// Without this line Lora Radio doesn't work with heltec lib
#define BUTTON 21
#define ARDUINO_heltec_wifi_32_lora_V3
#include "heltec_unofficial.h"

// We are not using spectral scan here only RSSI method
// #include "modules/SX126x/patches/SX126x_patch_scan.h"
// #define PRINT_DEBUG

// TODO: move variables to common file
// <--- Spectrum display Variables START
#define SCAN_METHOD
#define METHOD_SPECTRAL
//  numbers of the spectrum screen lines = width of screen
#define STEPS 296 // 128
// Number of samples for each scan. Fewer samples = better temporal resolution.
#define MAX_POWER_LEVELS 33
// multiplies STEPS * N to increase scan resolution.
#define SCAN_RBW_FACTOR 1 // 2
// Print spectrum values pixels at once or by line
bool ANIMATED_RELOAD = false;
// Remove reading without neighbors
#define FILTER_SPECTRUM_RESULTS true
#define FILTER_SAMPLES_MIN
constexpr bool DRAW_DETECTION_TICKS = true;
// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
// if more than 100 it can freez
#define SAMPLES 35 //(scan time = 1294)
// number of samples for RSSI method
#define SAMPLES_RSSI 30 // 21 //

#define FREQ_BEGIN 650

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / (STEPS * SCAN_RBW_FACTOR))

uint64_t range = (int)(FREQ_END - FREQ_BEGIN);
uint64_t fr_begin = FREQ_BEGIN;
uint64_t fr_end = FREQ_BEGIN;

// Feature to scan diapasones. Other frequency settings will be ignored.
// int SCAN_RANGES[] = {850890, 920950};
int SCAN_RANGES[] = {};

// MHZ per page
// to put everything into one page set RANGE_PER_PAGE = FREQ_END - 800
// uint64_t RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN

// Override or e-ink
uint64_t RANGE_PER_PAGE = FREQ_BEGIN + DISPLAY_WIDTH;

uint64_t iterations = RANGE / RANGE_PER_PAGE;

// uint64_t range_frequency = FREQ_END - FREQ_BEGIN;
uint64_t median_frequency = FREQ_BEGIN + FREQ_END - FREQ_BEGIN / 2;

// #define DISABLE_PLOT_CHART false   // unused

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t result_display_set[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t result_detections[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

// Waterfall array
bool waterfall[STEPS], detected_y[STEPS]; // 20 - ??? steps of the waterfall

// global variable

// Used as a Led Light and Buzzer/count trigger
bool first_run, new_pixel, detected_x = false;
// drone detection flag
bool detected = false;
uint64_t drone_detection_level = 90;
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
// <--- Spectrum display Variables END

// Initialize the display
DEPG0290BxS800FxX_BW display(5, 4, 3, 6, 2, 1, -1,
                             6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
/* screen rotation
 * ANGLE_0_DEGREE
 * ANGLE_90_DEGREE
 * ANGLE_180_DEGREE
 * ANGLE_270_DEGREE
 */
#define DIRECTION ANGLE_0_DEGREE

// TODO: move to common file
void init_radio()
{
    // initialize SX1262 FSK modem at the initial frequency
    Serial.println("Init radio");
    RADIOLIB_OR_HALT(radio.beginFSK(FREQ_BEGIN));

    // upload a patch to the SX1262 to enable spectral scan
    // NOTE: this patch is uploaded into volatile memory,
    // and must be re-uploaded on every power up
    Serial.println("Upload SX1262 patch");

    // Upload binary patch into the SX126x device RAM. Patch is needed to e.g.,
    // enable spectral scan and must be uploaded again on every power cycle.
    // RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
    // configure scan bandwidth and disable the data shaping

    Serial.println("Setting up radio");
    RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));

    // and disable the data shaping
    RADIOLIB_OR_HALT(radio.setDataShaping(RADIOLIB_SHAPING_NONE));
    Serial.println("Starting scanning...");

    // calibrate only once ,,, at startup
    // TODO: check documentation (9.2.1) if we must calibrate in certain ranges
    radio.setFrequency(FREQ_BEGIN, true);
    delay(50);
}

#define HEIGHT 4

void drawSetupText()
{
    // create more fonts at http://oleddisplay.squix.ch/
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Spectrum Analyzer Lora SA");
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 10, "SX 1262");
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 26, "e-ink display");
    display.drawString(0, 56, "RF Spectrum X-Ray");
    display.setFont(ArialMT_Plain_24);
}

#define battery_w 13
#define battery_h 13
#define BATTERY_PIN 7

void battery()
{
    analogReadResolution(12);
    int battery_levl = analogRead(BATTERY_PIN) / 238.7; // battary/4096*3.3* coefficient
    float battery_one = 0.4125;
#ifdef PRINT_DEBUG
    Serial.printf("ADC analog value = %.2f\n", battery_levl);
#endif
    display.drawString(257, 0, String(heltec_battery_percent(battery_levl)) + "%");
    // TODO: battery voltage doesn't work
    if (battery_levl < battery_one)
    {
        display.drawXbm(275, 0, battery_w, battery_h, battery0);
    }
    else if (battery_levl < 2 * battery_one && battery_levl > battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery1);
    }
    else if (battery_levl < 3 * battery_one && battery_levl > 2 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery2);
    }
    else if (battery_levl < 4 * battery_one && battery_levl > 3 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery3);
    }
    else if (battery_levl < 5 * battery_one && battery_levl > 4 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery4);
    }
    else if (battery_levl < 6 * battery_one && battery_levl > 5 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery5);
    }
    else if (battery_levl < 7 * battery_one && battery_levl > 6 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, battery6);
    }
    else if (battery_levl < 7 * battery_one && battery_levl > 6 * battery_one)
    {
        display.drawXbm(285, 0, battery_w, battery_h, batteryfull);
    }
}

void VextON(void)
{
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
}
void VextOFF(void) // Vext default OFF
{
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);
}

constexpr int lower_level = 108;
constexpr int up_level = 40;
constexpr int start_pixel = 80;

int rssiToPix(int rssi)
{
    // Bigger is lower signal
    if (abs(rssi) >= lower_level)
    {
        return start_pixel - 1;
    }
    if (abs(rssi) <= up_level)
    {
        return start_pixel - up_level;
    }

    return start_pixel - (lower_level - abs(rssi));
}

long timeSinceLastModeSwitch = 0;

float fr = FREQ_BEGIN, fr_x[STEPS + 5], vbat = 0;
// MHz in one screen pix step
// END will be Begin + 289 * mhz_step
constexpr int mhz_step = 1;
// TODO: make end_freq
// Measure RSS every step
constexpr float rssi_mhz_step = 0.33;
int rssi2 = 0;
int x1 = 0, y2 = 0;
unsigned int screen_update_loop_counter = 0;
unsigned int x_screen_update = 0;
int rssi_printed = 0;
constexpr int rssi_window_size = 30;
int max_i_rssi = -999;
int window_max_rssi = -999;
int window_max_fr = -999;
int max_scan_rssi[STEPS + 2];
long display_scan_start = 0;
long display_scan_end = 0;
long display_scan_i_end = 0;
int scan_iterations = 0;
bool waterfall_values[DISPLAY_HEIGHT][DISPLAY_WIDTH] = {false};

constexpr unsigned int SCANS_PER_DISPLAY = 5;
constexpr unsigned int STATUS_BAR_HEIGHT = 5;

void button_logic(void)
{
    heltec_loop();
    button_pressed_counter = 0;
    if (button.pressed())
    {
        drone_detection_level++;
        if (drone_detection_level > 107)
            drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL - 20;
        while (button.pressedNow())
        {
            delay(100);
            display.display();
            button_pressed_counter++;
            // button.update();

            if (button_pressed_counter > 18)
            {
                // Some sign there
            }
        }
    }

    if (button_pressed_counter < 9 && button_pressed_counter > 5)
    {
        display.clear();
        display.display();
        heltec_deep_sleep();
    }
}

void loop()
{
    if (screen_update_loop_counter == 0)
    {
        // Zero arrays
        for (int i = 0; i < STEPS; i++)
        {
            fr_x[x1] = 0;
            max_scan_rssi[i] = -999;
        }
        display_scan_start = millis();
    }
    fr_x[x1] = fr;
    int u = 0;
    for (int i = 0; i < SAMPLES_RSSI; i++)
    {
        radio.setFrequency((float)fr + (float)(rssi_mhz_step * u),
                           false); // false = no calibration need here
        u++;
        if (rssi_mhz_step * u >= mhz_step)
        {
            u = 0;
        }

        rssi2 = radio.getRSSI(false);
        scan_iterations++;
        if (rssi2 > lower_level)
        {
            max_scan_rssi[x1] = rssi2;
            continue;
        }
#ifdef PRINT_DEBUG
        Serial.println(String(fr) + ":" + String(rssi2));
#endif
        button_logic();
        // display.drawString(x1, (int)y2, String(fr) + ":" + String(rssi2));
        display.setPixel(x1, rssiToPix(rssi2));

        if (max_scan_rssi[x1] < rssi2)
        {
            max_scan_rssi[x1] = rssi2;
        }
    }

    // Waterfall per scan not per screen
    if (abs(max_scan_rssi[x1]) <= drone_detection_level)
    {
        waterfall_values[w][x1] = true;
    }
    else
    {
        waterfall_values[w][x1] = false;
    }

    // drone detection level line
    if (x1 % 2 == 0)
    {
        display.setPixel(x1, rssiToPix(drone_detection_level));
    }
    fr += mhz_step;
    x1++;
    if (display_scan_i_end == 0)
    {
        display_scan_i_end = millis();
    }
    button_logic();
    // Main N x-axis full loop end logic
    if (x1 >= STEPS)
    {
        w++;
        if (w >= DISPLAY_HEIGHT - start_pixel - 13)
        {
            w = 0;
        }
        if (screen_update_loop_counter == SCANS_PER_DISPLAY)
        {
            // max Mhz and dB in window
            for (int i = 0; i < STEPS; i++)
            {
                // Max dB in window
                if (window_max_rssi < max_scan_rssi[i])
                {
                    // Max Mhz in window
                    window_max_fr = fr_x[i];
                    window_max_rssi = max_scan_rssi[i];
                }
                if (i % rssi_window_size == 0 || (i % (DISPLAY_WIDTH - 1)) == 0)
                {

                    if (abs(window_max_rssi) < drone_detection_level)
                    {
                        y2 = 10;

                        display.setFont(ArialMT_Plain_10);
                        display.drawStringMaxWidth(i - rssi_window_size, y2,
                                                   rssi_window_size,
                                                   String(window_max_rssi) + "dB");
                        display.drawString(i - rssi_window_size + 5, y2 + 10,
                                           String(window_max_fr));
                        // Vertical lines between windows
                        for (int l = y2; l < start_pixel; l += 4)
                        {
                            display.setPixel(i, l);
                        }
                    }
                    window_max_rssi = -999;
                }

                // Draw Waterfall
                for (int y = 0; y < DISPLAY_HEIGHT; y++)
                    if (waterfall_values[y][i] == true)
                    {
                        display.setPixel(i, start_pixel + 5 + y);
                    }
            }
            // Draw Waterfall cursor
            display.drawHorizontalLine(0, start_pixel + 5 + w, DISPLAY_WIDTH);

            display_scan_end = millis();

            display.setFont(ArialMT_Plain_10);
            display.drawString(0, 0,
                               "T:" + String(display_scan_end - display_scan_start) +
                                   "/" + String(display_scan_i_end - display_scan_start) +
                                   " L:-" + String(drone_detection_level) + "dB");

            // some issues with the performance.
            // TODO: fix this issue
            if (display_scan_end - display_scan_start > 20000)
            {
                esp_restart();
            }

            // ToDo: it doesn't work
            battery();
            // iteration full scan / samples pixel step / numbers of scan per display
            display.drawString(DISPLAY_WIDTH - ((DISPLAY_WIDTH / 6) * 2) - 5, 0,
                               "i:" + String(scan_iterations) + "/" + String(SAMPLES) +
                                   "/" + String(SCANS_PER_DISPLAY));
            // Scan resolution
            display.drawString(DISPLAY_WIDTH - ((DISPLAY_WIDTH / 6) * 2) - 35, 0,
                               "r:" + String(rssi_mhz_step));
            // Mhz in pixel
            display.drawString(DISPLAY_WIDTH - ((DISPLAY_WIDTH / 6) * 2) - 55, 0,
                               "s:" + String(mhz_step));

            // Draw a line horizontally
            display.drawHorizontalLine(0, 1 + start_pixel, DISPLAY_WIDTH);
            // Generate Ticks
            for (int x = 0; x < DISPLAY_WIDTH; x++)
            {
                if (x % (DISPLAY_WIDTH / 2) == 0 && x > 5)
                {
                    display.drawVerticalLine(x, 1 + start_pixel, 11);
                    // central tick width
                    // display.drawVerticalLine(x - 1, lower_level + 1, 8);
                    // display.drawVerticalLine(x + 1, lower_level + 1, 8);
                }
                if (x % 10 == 0 || x == 0)
                    display.drawVerticalLine(x, 1 + start_pixel, 6);
                if (x % 5 == 0)
                    display.drawVerticalLine(x, 1 + start_pixel, 3);
            }
            display.setFont(ArialMT_Plain_10);

            // Begin Mhz
            display.drawString(1, DISPLAY_HEIGHT - 10, String(FREQ_BEGIN));
            // Median -1/2 Mhz
            display.drawString((DISPLAY_WIDTH / 4) - 10, DISPLAY_HEIGHT - 10,
                               String(FREQ_BEGIN + (((int)fr - FREQ_BEGIN) / 4)));
            // Median Mhz
            display.drawString((DISPLAY_WIDTH / 2) - 10, DISPLAY_HEIGHT - 10,
                               String(FREQ_BEGIN + (((int)fr - FREQ_BEGIN) / 2)));
            // Median + 1/2 Mhz
            display.drawString((DISPLAY_WIDTH - (DISPLAY_WIDTH / 4)) - 10,
                               DISPLAY_HEIGHT - 10,
                               String(FREQ_BEGIN + (((int)fr - FREQ_BEGIN) -
                                                    ((int)fr - FREQ_BEGIN) / 4)));
            // End Mhz
            display.drawString(DISPLAY_WIDTH - 20, DISPLAY_HEIGHT - 10, String((int)fr));

            display.display();
            // display will be cleared next scan iteration. it is just buffer clear
            // memset(buffer, 0, displayBufferSize);
            display.clear();
            screen_update_loop_counter = 0;

            scan_iterations = 0;
            display_scan_i_end = 0;
        }
        fr = FREQ_BEGIN;
        x1 = 0;
        rssi_printed = 0;
        // Prevent screen_update_loop_counter++ when it is just nulled
        if (scan_iterations > 0)
        {
            screen_update_loop_counter++;
        }
    }
#ifdef PRINT_DEBUG
    Serial.println("Full Scan:" + String(screen_update_loop_counter));
#endif
}

void setup()
{
    // Initialising the UI will init the display too.
    display.init();
    // Of not this screen doesn't work
    VextON();
    display.screenRotate(DIRECTION);
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawXbm((DISPLAY_WIDTH / 3) - 10, DISPLAY_HEIGHT / 4, 128, 60,
                    epd_bitmap_ucog);
    display.display();
    delay(1000);
    display.clear();
    Serial.begin(115200);
    w = 0; // WATERFALL_START;
    init_radio();
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        Serial.println(state);
    }
    heltec_setup();

    drawSetupText();
    display.display();
    display.clear();
    delay(100);
}
