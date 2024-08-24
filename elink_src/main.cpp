/* Heltec Automation Ink screen example
 * NOTE!!!: to upload we neew code you need to press button BOOT and RESET or you will
 * have serial error. After upload you need reset device...
 *
 * Function:
 * 1. Ink screen full brush demonstration
 *
 * Description:
 * 1.Inherited from ssd1306 for drawing points, lines, and functions
 *
 * */

#include "HT_DEPG0290BxS800FxX_BW.h"
#include "global_config.h"
#include "images.h"
#include "ui.h"
#include <Arduino.h>

// Disabling default lib display
#define HELTEC_NO_DISPLAY
#define DISPLAY_WIDTH 296
#define DISPLAY_HEIGHT 128
// Without this line Lora Radio doesn't work
#define ARDUINO_heltec_wifi_32_lora_V3
#include "heltec_unofficial.h"
#include "modules/SX126x/patches/SX126x_patch_scan.h"

// <--- Spectrum display Varriables START
#define SCAN_METHOD
#define METHOD_SPECTRAL
//  numbers of the spectrum screan lines = width of screan
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

#define FREQ_BEGIN 750

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / (STEPS * SCAN_RBW_FACTOR))

uint64_t range = (int)(FREQ_END - FREQ_BEGIN);
uint64_t fr_begin = FREQ_BEGIN;
uint64_t fr_end = FREQ_BEGIN;

// Feature to scan diapazones. Other frequency settings will be ignored.
// int SCAN_RANGES[] = {850890, 920950};
int SCAN_RANGES[] = {};

// MHZ per page
// to put everething into one page set RANGE_PER_PAGE = FREQ_END - 800
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
// <--- Spectrum display Varriables END

// Initialize the display
DEPG0290BxS800FxX_BW display(5, 4, 3, 6, 2, 1, -1,
                             6000000); // rst,dc,cs,busy,sck,mosi,miso,frequency
typedef void (*Demo)(void);
/* screen rotation
 * ANGLE_0_DEGREE
 * ANGLE_90_DEGREE
 * ANGLE_180_DEGREE
 * ANGLE_270_DEGREE
 */
#define DIRECTION ANGLE_0_DEGREE
int demoMode = 0;

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
    RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
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
DEPG0290BxS800FxX_BW display_instance = display;
/**
 * @brief Draws ticks on the display at regular whole intervals.
 *
 * @param every The interval between ticks in MHz.
 * @param length The length of each tick in pixels.
 */
void drawTicks(float every, int length)
{
    int first_tick;
    bool correction;
    int pixels_per_step;
    int correction_number;
    int tick;
    int tick_minor;
    int median;

    first_tick = 0;
    //+ (every - (fr_begin - (int)(fr_begin / every) * every));
    /*if (first_tick < fr_begin)
    {
        first_tick += every;
    }*/
    correction = false;
    pixels_per_step = STEPS / (RANGE_PER_PAGE / every);
    if (STEPS / RANGE_PER_PAGE != 0)
    {
        correction = true;
    }
    correction_number = STEPS - (int)(pixels_per_step * (RANGE_PER_PAGE / every));
    tick = 0;
    tick_minor = 0;
    median = (RANGE_PER_PAGE / every) / 2;
    // TODO: (RANGE_PER_PAGE / every)
    //	* 2 has twice extra steps we need to figureout correct logic or minor
    // ticks is not showing to the end
    for (int t = 0; t <= (RANGE_PER_PAGE / every) * 2; t++)
    {
        // fix if pixels per step is not int and we have shift
        if (correction && t % 2 != 0 && correction_number > 1)
        {
            // pixels_per_step++;
            correction_number--;
        }
        tick += pixels_per_step;
        tick_minor = tick / 2;
        if (tick <= 128 - 3)
        {
            display_instance.drawLine(tick, HEIGHT + X_AXIS_WEIGHT, tick,
                                      HEIGHT + X_AXIS_WEIGHT + length);
            // Central tick
            if (tick > (128 / 2) - 3 && tick < (128 / 2) + 3)
            {
                display_instance.drawLine(tick + 1, HEIGHT + X_AXIS_WEIGHT, tick + 1,
                                          HEIGHT + X_AXIS_WEIGHT + length);
            }
        }
#ifdef MINOR_TICKS
        // Fix two ticks together
        if ((tick_minor + 1 != tick) && (tick_minor - 1 != tick) &&
            (tick_minor + 2 != tick) && (tick_minor - 2 != tick))
        {
            display_instance.drawLine(tick_minor, HEIGHT + X_AXIS_WEIGHT, tick_minor,
                                      HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
        }
        // Central tick
        if (tick_minor > (128 / 2) - 3 && tick_minor < (128 / 2) + 3)
        {
            display_instance.drawLine(tick_minor + 1, HEIGHT + X_AXIS_WEIGHT,
                                      tick_minor + 1,
                                      HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
        }
#endif
    }
}

void drawFontFaceDemo()
{
    // Font Demo1
    // create more fonts at http://oleddisplay.squix.ch/
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Spectrum Analizer Lora SA");
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 10, "SX 1262");
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 26, "e-ink display");
}
void drawTextFlowDemo()
{
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawStringMaxWidth(
        0, 0, DISPLAY_HEIGHT,
        "Lorem ipsum\n dolor sit amet, consetetur sadipscing elitr, sed diam nonumy "
        "eirmod tempor invidunt ut labore.");
}
void drawTextAlignmentDemo()
{
    // Text alignment demo
    char str[30];
    int x = 0;
    int y = 0;
    display.setFont(ArialMT_Plain_10);
    // The coordinates define the left starting point of the text
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(x, y, "Left aligned (0,0)");
    // The coordinates define the center of the text
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    x = display.width() / 2;
    y = display.height() / 2 - 5;
    sprintf(str, "Center aligned (%d,%d)", x, y);
    display.drawString(x, y, str);
    // The coordinates define the right end of the text
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    x = display.width();
    y = display.height() - 12;
    sprintf(str, "Right aligned (%d,%d)", x, y);
    display.drawString(x, y, str);
}
void drawRectDemo()
{
    // Draw a pixel at given position
    for (int i = 0; i < 10; i++)
    {
        display.setPixel(i, i);
        display.setPixel(10 - i, i);
    }
    display.drawRect(12, 12, 20, 20);
    // Fill the rectangle
    display.fillRect(14, 14, 17, 17);
    // Draw a line horizontally
    display.drawHorizontalLine(0, 40, 20);
    // Draw a line horizontally
    display.drawVerticalLine(40, 0, 20);
}
void drawCircleDemo()
{
    int x = display.width() / 4;
    int y = display.height() / 2;
    for (int i = 1; i < 8; i++)
    {
        display.setColor(WHITE);
        display.drawCircle(x, y, i * 3);
        if (i % 2 == 0)
        {
            display.setColor(BLACK);
        }
        int x = display.width() / 4 * 3;
        display.fillCircle(x, y, 32 - i * 3);
    }
}
void drawImageDemo()
{
    // see http://blog.squix.org/2015/05/esp8266-nodemcu-how-to-create-xbm.html
    // on how to create xbm files
    int x = display.width() / 2 - WiFi_Logo_width / 2;
    int y = display.height() / 2 - WiFi_Logo_height / 2;
    display.drawXbm(x, y, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
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

int lower_level = 108;
int up_level = 40;
int rssiToPix(int rssi)
{
    // Bigger is lower signal
    if (abs(rssi) >= lower_level)
    {
        return lower_level - 1;
    }
    if (abs(rssi) <= up_level)
    {
        return up_level;
    }
    return abs(rssi);
}

Demo demos[] = {drawFontFaceDemo, drawTextFlowDemo, drawTextAlignmentDemo,
                drawRectDemo,     drawCircleDemo,   drawImageDemo};
int demoLength = (sizeof(demos) / sizeof(Demo));
long timeSinceLastModeSwitch = 0;

float fr = FREQ_BEGIN;
int rssi2 = 0;
int x1 = 0, y2 = 0;
unsigned int loop_counter = 1;
void loop()
{
    radio.setFrequency(fr, false); // false = no calibration need here
    for (int i = 0; i < SAMPLES_RSSI; i++)
    {
        if (i % 2 == 0)
            radio.setFrequency((float)fr + 0.33, false);
        else if (i % 3 == 0)
            radio.setFrequency((float)fr + 0.33, false);
        rssi2 = radio.getRSSI(false);
        if (rssi2 > lower_level)
            continue;
        // Serial.println(String(fr) + ":" + String(rssi2));
        //  display.drawString(x1, (int)y2, String(fr) + ":" + String(rssi2));
        display.setPixel(x1, rssiToPix(rssi2));
    }

    fr++;
    x1++;
    if (x1 >= STEPS)
    {
        if (loop_counter > STEPS * 5)
        {
            loop_counter = 0;
            // Draw a line horizontally
            display.drawHorizontalLine(0, lower_level + 1, DISPLAY_WIDTH);
            for (int x = 0; x < DISPLAY_WIDTH; x++)
            {
                if (x % (DISPLAY_WIDTH / 2) == 0 && x > 5)
                {
                    display.drawVerticalLine(x, lower_level + 1, 8);
                    display.drawVerticalLine(x - 1, lower_level + 1, 8);
                    display.drawVerticalLine(x + 1, lower_level + 1, 8);
                }
                if (x % 10 == 0 || x == 0)
                    display.drawVerticalLine(x, lower_level + 1, 6);
                if (x % 5 == 0)
                    display.drawVerticalLine(x, lower_level + 1, 3);
            }
            display.setFont(ArialMT_Plain_10);

            display.drawString(1, DISPLAY_HEIGHT - 10, String(FREQ_BEGIN));
            display.drawString(DISPLAY_WIDTH - 24, DISPLAY_HEIGHT - 10, String(fr));
            display.drawString((DISPLAY_WIDTH / 2) - 10, DISPLAY_HEIGHT - 10,
                               String(FREQ_BEGIN + ((fr - FREQ_BEGIN) / 2)));

            display.display();
            // delay(2000);
            if (loop_counter == 0)
            {
                display.clear();
            }
        }
        fr = FREQ_BEGIN;
        x1 = 0;
    }
    loop_counter++;
}

void setup()
{
    // Initialising the UI will init the display too.
    display.init();
    display.screenRotate(DIRECTION);
    display.setFont(ArialMT_Plain_10);
    display.clear();
    display.drawXbm((DISPLAY_WIDTH / 3) - 10, DISPLAY_HEIGHT / 4, 128, 60,
                    epd_bitmap_ucog);
    display.display();
    delay(2000);
    display.clear();
    Serial.begin(115200);
    w = WATERFALL_START;
    init_radio();
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        Serial.println(state);
    }
    heltec_setup();
    Serial.println();
    Serial.println();
    drawFontFaceDemo();
    display.display();
    delay(1000);
    VextON();
    display.clear();
}
