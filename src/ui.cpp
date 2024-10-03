#include "ui.h"
#include "RadioLib.h"
#include "global_config.h"
#include "images.h"
#include <charts.h>
#include <scan.h>

// -------------------------------------------------
// LOCAL DEFINES
// Height of the plotter area
// -------------------------------------------------

#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE
//
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH)

// temporary dirty import ... to be solved durring upcoming refactoring
extern unsigned int RANGE_PER_PAGE;
extern unsigned int median_frequency;
extern unsigned int drone_detected_frequency_start;
extern unsigned int drone_detected_frequency_end;
extern unsigned int ranges_count;
extern int SCAN_RANGES[];
extern unsigned int ranges_count;
extern unsigned int iterations;
extern unsigned int range_item;

extern uint64_t loop_time;

void UI_Init(Display_t *display_ptr)
{
    // check for null ???
    display_ptr->clear();
    // draw the UCOG welcome logo
    display_ptr->drawXbm(0, 2, 128, 64, epd_bitmap_ucog);
    display_ptr->display();
}

void StatusBar::clearStatus(void)
{
    // clear status line
    display.setColor(BLACK);
    display.fillRect(pos_x, pos_y, width, height);
}

void UI_clearPlotter(void)
{
    // clear the scan plot rectangle (top part)
    //    display_instance->setColor(BLACK);
    //    display_instance->fillRect(0, 10, STEPS, HEIGHT - 10);
    //    display_instance->setColor(WHITE);
}

void UI_clearTopStatus(void)
{
    // clear the scan plot rectangle (top part)
    //    display_instance->setColor(BLACK);
    //    display_instance->fillRect(0, 0, STEPS, 10);
    //    display_instance->setColor(WHITE);
}

void UI_drawCursor(int16_t possition)
{
    // Draw animated vertical cursor on reload process
    // display_instance->setColor(BLACK);
    // display_instance->drawVerticalLine(possition, 0, HEIGHT);
    // display_instance->drawVerticalLine(possition + 1, 0, HEIGHT);
    // display_instance->drawVerticalLine(possition + 2, 0, HEIGHT);
    // display_instance->setColor(WHITE);
}

/**
 * @brief Decorates the display: everything but the plot itself.
 */
void StatusBar::draw()
{
    uint16_t text_y = pos_y + height - 10;

    if (!ui_initialized)
    {
        // Drone detection level
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(width, 0, String(r.drone_detection_level));
    }
    if (!ui_initialized)
    {
        // Clear something
        /*        display_instance->setColor(BLACK);
                display_instance->fillRect(0, SCALE_TEXT_TOP + 1, 128, 12);
                display_instance->setColor(WHITE);
        */
        // Drone detection level
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(pos_x + width, 0, String(r.drone_detection_level));
        // Frequency start
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(pos_x, text_y,
                           (r.fr_begin == 0) ? String(FREQ_BEGIN) : String(r.fr_begin));

        // Frequency detected
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(pos_x + width / 2, text_y,
                           (r.fr_begin == 0)
                               ? String(median_frequency)
                               : String(r.fr_begin + ((r.fr_end - r.fr_begin) / 2)));
        // Frequency end
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(pos_x + width, text_y,
                           (r.fr_end == 0) ? String(FREQ_END) : String(r.fr_end));
    }

    // Status text block
    if (r.led_flag) // 'drone' detected
    {
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        // clear status line
        clearStatus();
        display.setColor(WHITE);
        display.drawString(pos_x + width / 2, text_y,
                           String(drone_detected_frequency_start) + ">RF<" +
                               String(drone_detected_frequency_end));
    }
    else
    {
        // "Scanning"
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        // clear status line
        clearStatus();
        String s = "Scan  \\";
        if (scan_progress_count == 1)
        {
            s = "Scan  |";
        }
        else if (scan_progress_count == 2)
        {
            s = "Scan  /";
        }
        else if (scan_progress_count == 3)
        {
            s = "Scan  -";
        }
        scan_progress_count++;
        if (scan_progress_count >= 4)
        {
            scan_progress_count = 0;
        }
        display.setColor(WHITE);
        display.drawString(pos_x + width / 2 - 3, text_y, s);
    }

    if (r.led_flag && r.detection_count >= 5)
    {
        digitalWrite(LED, HIGH);
        if (r.sound_on)
        {
            tone(BUZZER_PIN, 104, 100);
        }
        digitalWrite(REB_PIN, HIGH);
        r.led_flag = false;
    }
    else if (!r.led_flag)
    {
        digitalWrite(LED, LOW);
    }

    if (ranges_count == 0)
    {
#ifdef DEBUG
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(pos_x, text_y, String(loop_time));
#else
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(pos_x, text_y, String(FREQ_BEGIN));

#endif
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(pos_x + width, text_y, String(FREQ_END));
    }
    else if (ranges_count > 0)
    {
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(pos_x, text_y,
                           String(SCAN_RANGES[range_item] / 1000) + "-" +
                               String(SCAN_RANGES[range_item] % 1000));
        if (range_item + 1 < iterations)
        {
            display.setTextAlignment(TEXT_ALIGN_RIGHT);
            display.drawString(pos_x + width, text_y,
                               String(SCAN_RANGES[range_item + 1] / 1000) + "-" +
                                   String(SCAN_RANGES[range_item + 1] % 1000));
        }
    }
    ui_initialized = true;
}
