#include "ui.h"
#include "images.h"
#include "global_config.h"
#include "RadioLib.h"

// -------------------------------------------------
// LOCAL DEFINES
// Height of the plotter area
// -------------------------------------------------

#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE
//
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH)

static unsigned int start_scan_text = (128 / 2) - 3;
// initialized flag
static bool ui_initialized = false;
static bool led_flag = false;
static unsigned short int scan_progress_count = 0;

//(0x3c, SDA_OLED, SCL_OLED, DISPLAY_GEOMETRY);
static SSD1306Wire *display_instance;

// temporary dirty import ... to be solved durring upcoming refactoring
extern unsigned int drone_detection_level;
extern unsigned int RANGE_PER_PAGE;
extern unsigned int median_frequency;
extern unsigned int detection_count;
extern bool SOUND_ON;
extern bool drone_detected;
extern unsigned int drone_detected_frequency_start;
extern unsigned int drone_detected_frequency_end;
extern unsigned int ranges_count;
extern int SCAN_RANGES[];
extern unsigned int ranges_count;
extern unsigned int iterations;
extern unsigned int scan_iteration;

void UI_Init(SSD1306Wire *display_ptr)
{
    // init pointer to display instance.
    display_instance = display_ptr;
    // check for null ???
    display_instance->clear();
    // draw the UCOG welcome logo
    display_instance->drawXbm(0, 2, 128, 64, epd_bitmap_ucog);
    display_instance->display();
}

void UI_setLedFlag(bool new_status)
{
    led_flag = new_status;
}

void clearStatus(void)
{
    // clear status line
    display_instance->setColor(BLACK);
    display_instance->fillRect(0, STATUS_TEXT_TOP + 2, 128, 13);
    display_instance->setColor(WHITE);
}

void UI_clearPlotter(void)
{
    // clear the scan plot rectangle
    display_instance->setColor(BLACK);
    display_instance->fillRect(0, 0, STEPS, HEIGHT);
    display_instance->setColor(WHITE);
}

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
    //	* 2 has twice extra steps we need to figureout correct logic or minor ticks is not showing to the end
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
            display_instance->drawLine(tick, HEIGHT + X_AXIS_WEIGHT, tick,
                                       HEIGHT + X_AXIS_WEIGHT + length);
            // Central tick
            if (tick > (128 / 2) - 3 && tick < (128 / 2) + 3)
            {
                display_instance->drawLine(tick + 1, HEIGHT + X_AXIS_WEIGHT,
                                           tick + 1, HEIGHT + X_AXIS_WEIGHT + length);
            }
        }
#ifdef MINOR_TICKS
        // Fix two ticks together
        if ((tick_minor + 1 != tick) && (tick_minor - 1 != tick) && (tick_minor + 2 != tick) && (tick_minor - 2 != tick))
        {
            display_instance->drawLine(tick_minor, HEIGHT + X_AXIS_WEIGHT,
                                       tick_minor, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
        }
        // Central tick
        if (tick_minor > (128 / 2) - 3 && tick_minor < (128 / 2) + 3)
        {
            display_instance->drawLine(tick_minor + 1, HEIGHT + X_AXIS_WEIGHT,
                                       tick_minor + 1, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
        }
#endif
    }
}

void UI_drawCursor(int16_t possition)
{
    // Draw animated cursor on reload process
    display_instance->setColor(BLACK);
    display_instance->drawVerticalLine(possition, 0, HEIGHT);
    display_instance->drawVerticalLine(possition + 1, 0, HEIGHT);
    display_instance->setColor(WHITE);
}

/**
 * @brief Decorates the display: everything but the plot itself.
 */
void UI_displayDecorate(int begin = 0, int end = 0, bool redraw = false)
{
    if (!ui_initialized)
    {
        // Start and end ticks
        display_instance->fillRect(0, HEIGHT + X_AXIS_WEIGHT, 2,
                                   MAJOR_TICK_LENGTH + 1);
        display_instance->fillRect(126, HEIGHT + X_AXIS_WEIGHT, 2,
                                   MAJOR_TICK_LENGTH + 1);
        // Drone detection level
        display_instance->setTextAlignment(TEXT_ALIGN_RIGHT);
        display_instance->drawString(128, 0, String(drone_detection_level));
    }
    if (!ui_initialized || redraw)
    {
        // Clear something
        display_instance->setColor(BLACK);
        display_instance->fillRect(0, SCALE_TEXT_TOP + 1, 128, 12);
        display_instance->setColor(WHITE);
        // Drone detection level
        display_instance->setTextAlignment(TEXT_ALIGN_RIGHT);
        display_instance->drawString(128, 0, String(drone_detection_level));
        // Frequency start
        display_instance->setTextAlignment(TEXT_ALIGN_LEFT);
        display_instance->drawString(0, SCALE_TEXT_TOP,
                                     (begin == 0) ? String(FREQ_BEGIN) : String(begin));
        display_instance->setTextAlignment(TEXT_ALIGN_CENTER);
        display_instance->drawString(128 / 2, SCALE_TEXT_TOP,
                                     (begin == 0) ? String(median_frequency) : String(begin + ((end - begin) / 2)));
        // Frequency end
        display_instance->setTextAlignment(TEXT_ALIGN_RIGHT);
        display_instance->drawString(128, SCALE_TEXT_TOP,
                                     (end == 0) ? String(FREQ_END) : String(end));
    }
    if (led_flag == true && detection_count >= 5)
    {
        digitalWrite(LED, HIGH);
        if (SOUND_ON)
        {
            tone(BUZZER_PIN, 104, 100);
        }
        digitalWrite(REB_PIN, HIGH);
        led_flag = false;
    }
    else if (!redraw)
    {
        digitalWrite(LED, LOW);
    }
    // Status text block
    if (!drone_detected)
    {
        // "Scanning"
        display_instance->setTextAlignment(TEXT_ALIGN_CENTER);
        // clear status line
        clearStatus();
        if (scan_progress_count == 0)
        {
            display_instance->drawString(start_scan_text, STATUS_TEXT_TOP,
                                         "Scan.  ");
        }
        else if (scan_progress_count == 1)
        {
            display_instance->drawString(start_scan_text, STATUS_TEXT_TOP,
                                         "Scan.. ");
        }
        else if (scan_progress_count == 2)
        {
            display_instance->drawString(start_scan_text, STATUS_TEXT_TOP,
                                         "Scan...");
        }
        scan_progress_count++;
        if (scan_progress_count == 3)
        {
            scan_progress_count = 0;
        }
    }
    if (drone_detected)
    {
        display_instance->setTextAlignment(TEXT_ALIGN_CENTER);
        // clear status line
        clearStatus();
        display_instance->drawString(start_scan_text, STATUS_TEXT_TOP,
                                     String(drone_detected_frequency_start) + ">RF<" + String(drone_detected_frequency_end));
    }
    if (ranges_count == 0)
    {
        display_instance->setTextAlignment(TEXT_ALIGN_LEFT);
        display_instance->drawString(0, STATUS_TEXT_TOP, String(FREQ_BEGIN));
        display_instance->setTextAlignment(TEXT_ALIGN_RIGHT);
        display_instance->drawString(128, STATUS_TEXT_TOP, String(FREQ_END));
    }
    else if (ranges_count > 0)
    {
        display_instance->setTextAlignment(TEXT_ALIGN_LEFT);
        display_instance->drawString(0, STATUS_TEXT_TOP,
                                     String(SCAN_RANGES[scan_iteration] / 1000) + "-" + String(SCAN_RANGES[scan_iteration] % 1000));
        if (scan_iteration + 1 < iterations)
        {
            display_instance->setTextAlignment(TEXT_ALIGN_RIGHT);
            display_instance->drawString(128, STATUS_TEXT_TOP,
                                         String(SCAN_RANGES[scan_iteration + 1] / 1000) + "-" + String(SCAN_RANGES[scan_iteration + 1] % 1000));
        }
    }
    if (!ui_initialized)
    {
        // X-axis
        display_instance->fillRect(0, HEIGHT, STEPS, X_AXIS_WEIGHT);
// ticks
#ifdef MAJOR_TICKS
        drawTicks(MAJOR_TICKS, MAJOR_TICK_LENGTH);
#endif
    }
    ui_initialized = true;
}
