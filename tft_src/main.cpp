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
#include "HT_ST7789spi.h"
// #include "global_config.h"
#include "images.h"
// #include "ui.h"
#include <Adafruit_GFX.h>
#include <Arduino.h>

#define st7789_CS_Pin 39
#define st7789_REST_Pin 40
#define st7789_DC_Pin 47
#define st7789_SCLK_Pin 38
#define st7789_MOSI_Pin 48
#define st7789_LED_K_Pin 17
#define st7789_VTFT_CTRL_Pin 7

// lcd object pointer, it's a 240x135 lcd display, Adafruit dependcy
static HT_ST7789 *st7789 = NULL;
static SPIClass *gspi_lcd = NULL;

char buffer[256];

// Disabling default Heltec lib OLED display
#define HELTEC_NO_DISPLAY
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 170
// Without this line Lora Radio doesn't work with heltec lib
#define ARDUINO_heltec_wifi_32_lora_V3
// T190 button pin
#define BUTTON GPIO_NUM_21
#define HELTEC_POWER_BUTTON
#include "heltec_unofficial.h"

// We are not using spectral scan here only RSSI method
// #include "modules/SX126x/patches/SX126x_patch_scan.h"
// #define PRINT_DEBUG

// TODO: move variables to common file
// <--- Spectrum display Variables START
#define SCAN_METHOD
#define METHOD_SPECTRAL
//  numbers of the spectrum screen lines = width of screen
#define STEPS DISPLAY_WIDTH // 128
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
#define SAMPLES_RSSI 5 // 21 //

#define FREQ_BEGIN 650
#define FREQ_END 960
#define BANDWIDTH 467.0
#define DEFAULT_DRONE_DETECTION_LEVEL 90

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

// To remove waterfall adjust this and this
#define LOWER_LEVEL 108
#define WATERFALL_START 115
#define WATERFALL_END DISPLAY_HEIGHT - 10 - 2
#define DISABLE_WATERFALL 0 // to disable set to 1

uint64_t x, y, range_item, w = WATERFALL_START, i = 0;
int osd_x = 1, osd_y = 2, col = 0, max_bin = 32;
uint64_t ranges_count = 0;

float freq = 0;
int rssi = 0;
int state = 0;

#define MAX_MHZ_INTERVAL 2000
// 2KB ToDo: make dynamic array or sam structure
uint16_t detailed_scan_candidate[MAX_MHZ_INTERVAL];

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

void drawText(uint16_t x, uint16_t y, String text, uint16_t color = ST7789_WHITE)
{
    st7789->setCursor(x, y);
    st7789->setTextColor(color);
    st7789->setTextWrap(true);
    st7789->print(text.c_str());
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
    // TODO: battery voltage doesn't work
    if (battery_levl < battery_one)
    {
        // display.drawXbm(275, 0, battery_w, battery_h, battery0);
    }
    else if (battery_levl < 2 * battery_one && battery_levl > battery_one)
    {
        // display.drawXbm(285, 0, battery_w, battery_h, battery1);
    }
    else if (battery_levl < 3 * battery_one && battery_levl > 2 * battery_one)
    {
        // display.drawXbm(285, 0, battery_w, battery_h, battery2);
    }
    else if (battery_levl < 4 * battery_one && battery_levl > 3 * battery_one)
    {
        //  display.drawXbm(285, 0, battery_w, battery_h, battery3);
    }
    else if (battery_levl < 5 * battery_one && battery_levl > 4 * battery_one)
    {
        //  display.drawXbm(285, 0, battery_w, battery_h, battery4);
    }
    else if (battery_levl < 6 * battery_one && battery_levl > 5 * battery_one)
    {
        //  display.drawXbm(285, 0, battery_w, battery_h, battery5);
    }
    else if (battery_levl < 7 * battery_one && battery_levl > 6 * battery_one)
    {
        //  display.drawXbm(285, 0, battery_w, battery_h, battery6);
    }
    else if (battery_levl < 7 * battery_one && battery_levl > 6 * battery_one)
    {
        //  display.drawXbm(285, 0, battery_w, battery_h, batteryfull);
    }
}

constexpr int lower_level = LOWER_LEVEL;
constexpr int up_level = 40;
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

//
int rssiToColor(int rssi, bool waterfall = false)
{
    if (rssi < 80)
        return ST7789_RED; // Red
    if (rssi < 85)
        return ST7789_GREEN; // Green
    if (rssi < 90)
        return ST7789_YELLOW; // Yellow
    if (rssi < 95)
        return ST7789_BLUE; // Blue
    if (rssi < 100)
        return ST7789_MAGENTA; // Magenta
    if (waterfall)
        return ST7789_BLACK; // Black on waterfall
    return ST7789_WHITE;     // White on chart
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
constexpr int rssi_window_size = 45;
int max_i_rssi = -999;
int window_max_rssi = -999;
int window_max_fr = -999;
int max_scan_rssi[STEPS + 2];
int max_history_rssi[STEPS + 2];
long display_scan_start = 0;
long display_scan_end = 0;
long display_scan_i_end = 0;
long rssi_single_start = 0;
long rssi_single_end = 0;
int scan_iterations = 0;
// will be changed to false after first run
bool clear_rssi_history = true;

constexpr unsigned int SCANS_PER_DISPLAY = 1;
constexpr unsigned int STATUS_BAR_HEIGHT = 5;

void loop()
{
    Serial.println("Loop");
    if (screen_update_loop_counter == 0)
    {
        fr_x[x1] = 0;
        // Zero arrays
        for (int i = 0; i < STEPS; i++)
        {
            max_scan_rssi[i] = -999;
            if (clear_rssi_history == true)
                max_history_rssi[i] = -999;
        }
        clear_rssi_history = false;
        display_scan_start = millis();
    }
    fr_x[x1] = fr;

    int u = 0;
    int additional_samples = 0;

    // Clear old data with the cursor ...
    st7789->drawFastVLine(x1, lower_level, -lower_level + 11, ST7789_BLACK);
    // Draw max history line
    st7789->drawLine(x1, rssiToPix(max_history_rssi[x1]), x1, lower_level,
                     12710 /*gray*/);
    // Fetch samples
    for (int i = 0; i < SAMPLES_RSSI; i++)
    {
        // Checking more times curtain freq
        if (additional_samples > 0 &&
            (detailed_scan_candidate[(int)fr] + detailed_scan_candidate[(int)fr + 1] +
                 detailed_scan_candidate[(int)fr + 2] >
             0))
        {
            i--;
            additional_samples--;
        }

        bool calibrate = true;
        float freq = (float)fr + (float)(rssi_mhz_step * u);
        if ((int)freq % 10 == 0)
        {
            calibrate = true;
        }
        radio.setFrequency(freq,
                           /*false*/ calibrate); // false = no calibration need here
        // Serial.println((float)fr + (float)(rssi_mhz_step * u));
        u++;
        if (rssi_mhz_step * u >= mhz_step)
        {
            u = 0;
        }
        if (rssi_single_start == 0)
        {
            rssi_single_start = millis();
        }
        rssi2 = radio.getRSSI(false);
        // Serial.print(" RSSI : " + String(rssi2));
        scan_iterations++;
        if (rssi_single_end == 0)
        {
            rssi_single_end = millis();
        }
        if (abs(rssi2) > lower_level)
        {
#ifdef PRINT_DEBUG
            Serial.print("SKIP -> " + String(fr) + ":" + String(rssi2));
#endif
            // if lower than detection level set any
            if (max_scan_rssi[x1] == -999)
            {
                max_scan_rssi[x1] = rssi2;
            }
            continue;
        }
#ifdef PRINT_DEBUG
        Serial.println(String(fr) + ":" + String(rssi2));
#endif
        int lineHeight = 0;

        st7789->drawPixel(x1, rssiToPix(rssi2), rssiToColor(abs(rssi2)));
        st7789->drawPixel(x1, rssiToPix(rssi2) - 1, rssiToColor(abs(rssi2)));
        st7789->drawPixel(x1, rssiToPix(rssi2) - 2, rssiToColor(abs(rssi2)));

        if (true /*draw full line*/)
        {
            st7789->drawFastVLine(x1, rssiToPix(rssi2), lower_level - rssiToPix(rssi2),
                                  rssiToColor(abs(rssi2)));
        }
        // Draw Update Cursor
        st7789->drawFastVLine(x1 + 1, lower_level, -lower_level + 11, ST7789_BLACK);
        st7789->drawFastVLine(x1 + 2, lower_level, -lower_level + 11, ST7789_BLACK);
        // st7789->drawFastVLine(x1 + 3, lower_level, -lower_level + 11, ST7789_BLACK);

        if (max_scan_rssi[x1] == -999)
        {
            max_scan_rssi[x1] = rssi2;
        }
        /// -999 < -100
        if (max_scan_rssi[x1] < rssi2)
        {
#ifdef PRINT_DEBUG
            Serial.println("MAx Scan x-" + String(x1) + ": " + String(max_scan_rssi[x1]) +
                           "< " + String(rssi2));
#endif
            max_scan_rssi[x1] = rssi2;
            if (max_history_rssi[x1] < max_scan_rssi[x1])
            {
                max_history_rssi[x1] = rssi2;
            }
        }
        // Max dB in window
        if (window_max_rssi < max_scan_rssi[x1])
        {
            // Max Mhz in window
            window_max_fr = fr_x[x1];
            window_max_rssi = max_scan_rssi[x1];
        }
    }
    // Writing pixel only if it is bigger than drone detection level
    if (abs(max_scan_rssi[x1]) < drone_detection_level)
    {
        if (DISABLE_WATERFALL == 0)
        {
            // Waterfall Pixel
            st7789->drawPixel(x1, w, rssiToColor(abs(max_scan_rssi[x1]), true));
        }

        detailed_scan_candidate[(int)fr] = (int)fr;
    }
    else
    {
        detailed_scan_candidate[(int)fr] = (int)0;
    }
    // Draw legend for windows
    if (x1 % rssi_window_size == 0 || x1 == DISPLAY_WIDTH)
    {
        if (abs(window_max_rssi) < drone_detection_level && window_max_rssi != 0 &&
            window_max_rssi != -999)
        {
            y2 = 15;

            drawText(x1 - rssi_window_size + 3, y2, String(window_max_rssi) + "dB",
                     rssiToColor(abs(window_max_rssi)));
            drawText(x1 - rssi_window_size + 3, y2 + 10,
                     String((int)window_max_fr) + "MHz",
                     rssiToColor(abs(window_max_rssi)));
            // Vertical lines between windows
            for (int l = y2; l < 100; l += 4)
            {
                st7789->drawPixel(x1, l, ST7789_YELLOW);
            }
        }
        window_max_rssi = -999;
    }

    if (DISABLE_WATERFALL == 0)
    {
        // Waterfall cursor
        st7789->drawFastHLine(0, w + 1, DISPLAY_WIDTH, ST7789_BLACK);
        if (w < WATERFALL_END)
        {
            st7789->drawFastHLine(0, w + 2, DISPLAY_WIDTH, ST7789_ORANGE);
        }
    }

    // drone detection level line
    if (x1 % 2 == 0)
    {
        st7789->drawPixel(x1, rssiToPix(drone_detection_level), ST7789_GREEN);
    }
    fr += mhz_step;

    if (display_scan_i_end == 0)
    {
        display_scan_i_end = millis();
    }
    // Button Logic
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
            button_pressed_counter++;
            // button.update();

            if (button_pressed_counter > 18)
            {
                drawText(320 - 5, 5, "*", ST7789_WHITE);
            }
        }
    }

    if (button_pressed_counter < 9 && button_pressed_counter > 5)
    {
        heltec_deep_sleep();
    }

    // Main N x-axis full loop end logic
    if (x1 >= STEPS)
    {
        w++;
        if (w > WATERFALL_END)
        {
            w = WATERFALL_START;
        }
#ifdef PRINT_DEBUG
        Serial.println("Screen End for Output: " + String(screen_update_loop_counter));
#endif
        // Doing output only after full scan
        if (screen_update_loop_counter + 1 == SCANS_PER_DISPLAY)
        {
            // Scan results to max Mhz and dB in window
            display_scan_end = millis();

            st7789->fillRect(0, 0, DISPLAY_WIDTH, 11, ST7789_BLACK);
            drawText(0, 0,
                     "T:" + String(display_scan_end - display_scan_start) + "/" +
                         String(rssi_single_end - rssi_single_start) + " L:-" +
                         String(drone_detection_level) + "dB",
                     ST7789_BLUE);

            /// battery();
            // iteration full scan / samples pixel step / numbers of scan per display
            drawText(DISPLAY_WIDTH - ((DISPLAY_WIDTH / 6) * 2) + 20, 0,
                     "i:" + String(scan_iterations) + "/" + String(SAMPLES_RSSI) + "/" +
                         String(SCANS_PER_DISPLAY),
                     ST7789_GREEN);
            // Scan resolution - r
            // Mhz in pixel - s
            drawText(DISPLAY_WIDTH - ((DISPLAY_WIDTH / 6) * 2) - 55, 0,
                     "r:" + String(rssi_mhz_step) + " s:" + String(mhz_step), ST7789_RED);

            // Draw a line horizontally
            st7789->drawFastHLine(0, lower_level + 1, DISPLAY_WIDTH, ST7789_WHITE);
            // Generate Ticks
            for (int x = 0; x < DISPLAY_WIDTH; x++)
            {
                if (x % (DISPLAY_WIDTH / 2) == 0 && x > 5)
                {
                    st7789->drawFastVLine(x, lower_level + 1, 11, ST7789_WHITE);
                    //  central tick width
                    st7789->drawFastVLine(x - 1, lower_level + 1, 8, ST7789_WHITE);
                    st7789->drawFastVLine(x + 1, lower_level + 1, 8, ST7789_WHITE);
                }
                if (x % 10 == 0 || x == 0)
                    st7789->drawFastVLine(x, lower_level + 1, 6, ST7789_WHITE);
                if (x % 5 == 0)
                    st7789->drawFastVLine(x, lower_level + 1, 3, ST7789_WHITE);
            }
            // st7789.setFont(ArialMT_Plain_10);

            // Begin Mhz
            drawText(1, DISPLAY_HEIGHT - 10, String(FREQ_BEGIN));
            // Median -1/2 Mhz
            drawText((DISPLAY_WIDTH / 4) - 10, DISPLAY_HEIGHT - 10,
                     String(FREQ_BEGIN + (((int)fr - FREQ_BEGIN) / 4)));
            // Median Mhz
            drawText((DISPLAY_WIDTH / 2) - 10, DISPLAY_HEIGHT - 10,
                     String(FREQ_BEGIN + (((int)fr - FREQ_BEGIN) / 2)));
            // Median + 1/2 Mhz
            drawText((DISPLAY_WIDTH - (DISPLAY_WIDTH / 4)) - 10, DISPLAY_HEIGHT - 10,
                     String(FREQ_BEGIN +
                            (((int)fr - FREQ_BEGIN) - ((int)fr - FREQ_BEGIN) / 4)));
            // End Mhz
            drawText(DISPLAY_WIDTH - 24, DISPLAY_HEIGHT - 10, String((int)fr));

            screen_update_loop_counter = 0;
            scan_iterations = 0;
            display_scan_i_end = 0;
        }
        fr = FREQ_BEGIN;
        rssi_single_start = 0;
        rssi_single_end = 0;
        x1 = 0;
        rssi_printed = 0;
        // Prevent screen_update_loop_counter++ when it is just nulled
        if (scan_iterations > 0)
        {
            screen_update_loop_counter++;
        }
    }
    // not increase at the end of scan when nulled
    else
    {
        x1++;
    }

#ifdef PRINT_DEBUG
    Serial.println("Full Scan Counter:" + String(screen_update_loop_counter));
#endif
}

void setup()
{
    for (int i = 0; i < MAX_MHZ_INTERVAL; i++)
    {
        detailed_scan_candidate[i] = 0;
    }
    Serial.begin(115200);
    pinMode(7, OUTPUT);
    digitalWrite(7, LOW);
    delay(20);
    gspi_lcd = new SPIClass(HSPI);
    st7789 =
        new HT_ST7789(240, 320, gspi_lcd, st7789_CS_Pin, st7789_DC_Pin, st7789_REST_Pin);
    gspi_lcd->begin(st7789_SCLK_Pin, -1, st7789_MOSI_Pin, st7789_CS_Pin);
    // set up slave select pins as outputs as the Arduino API
    pinMode(gspi_lcd->pinSS(), OUTPUT);
    st7789->init(170, 320);
    st7789->setSPISpeed(40000000);
    /// st7789->setSPISpeed(3000000); /// default ~ 1000000

    Serial.printf("Ready!\r\n");
    st7789->setRotation(1);
    st7789->fillScreen(ST7789_BLACK);
    drawText(0, 0, "init >>> ", ST7789_WHITE);

    pinMode(st7789_LED_K_Pin, OUTPUT);
    digitalWrite(st7789_LED_K_Pin, HIGH);
    // pinMode(5, OUTPUT);
    // digitalWrite(5, HIGH);

    st7789->fillScreen(ST7789_BLACK);

    st7789->drawXBitmap(100, 50, epd_bitmap_ucog, 128, 64, ST7789_WHITE);
    init_radio();
    state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print(F("Failed to start receive mode, error code: "));
        Serial.println(state);
    }
    heltec_setup();
    delay(2500);
    st7789->fillScreen(ST7789_BLACK);
}
