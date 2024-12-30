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
extern uint64_t CONF_FREQ_BEGIN;
extern uint64_t CONF_FREQ_END;
extern unsigned int median_frequency;
extern unsigned int drone_detected_frequency_start;
extern unsigned int drone_detected_frequency_end;
extern size_t scan_pages_sz;
extern ScanPage *scan_pages;
extern size_t scan_page;

extern uint64_t loop_time;

void UI_Init(Display_t *display_ptr)
{
    // check for null ???
    display_ptr->clearDisplay();
    // draw the UCOG welcome logo
    display_ptr->drawBitmap(0, 0, epd_bitmap_ucog, 128, 64, SSD1306_WHITE);
    display_ptr->display();
}

void StatusBar::clearStatus(void)
{
    // clear status line
    // display.setColor(BLACK);
    display.fillRect(pos_x, pos_y - 1, width, height, BLACK);
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
    uint16_t text_y = pos_y + height - 8;

    if (!ui_initialized)
    {
        // Drone detection level
        // display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.setCursor(120, 0);
        display.print(String(r.drone_detection_level));
    }
    if (!ui_initialized)
    {
        // Clear something
        /*        display_instance->setColor(BLACK);
                display_instance->fillRect(0, SCALE_TEXT_TOP + 1, 128, 12);
                display_instance->setColor(WHITE);
        */
        // Drone detection level
        // display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.setTextSize(1);
        display.setCursor(pos_x + width, 0);
        display.print(String(r.drone_detection_level));
        // Frequency start
        // display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setCursor(pos_x, text_y);
        display.print((r.fr_begin == 0) ? String(CONF_FREQ_BEGIN) : String(r.fr_begin));

        // Frequency median
        // display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setCursor(pos_x + width / 2, text_y);
        display.print((r.fr_begin == 0)
                          ? String(median_frequency)
                          : String(r.fr_begin + ((r.fr_end - r.fr_begin) / 2)));
        // Frequency end
        // display.setTextAlignment(TEXT_ALIGN_RIGHT);
        int16_t x0, y0; // Variables to store the top-left corner
        uint16_t w, h0; // Variables to store width and height
        String s = String(CONF_FREQ_END);
        display.getTextBounds(s.c_str(), 0, 0, &x0, &y0, &w, &h0);

        display.setCursor(display.width() - w, text_y);
        display.print((r.fr_end == 0) ? String(CONF_FREQ_END) : String(r.fr_end));
    }

    // Status text block
    if (r.led_flag) // 'drone' detected
    {
        // display.setTextAlignment(TEXT_ALIGN_CENTER);
        //  clear status line
        clearStatus();
        display.setTextColor(WHITE);
        String s = String(drone_detected_frequency_start) + ">RF<" +
                   String(drone_detected_frequency_end);
        int16_t x0, y0; // Variables to store the top-left corner
        uint16_t w, h0; // Variables to store width and height
        display.getTextBounds(s.c_str(), 0, 0, &x0, &y0, &w, &h0);

        display.setCursor((display.width() - w) / 2, text_y);
        display.print(s);
    }
    else
    {
        // "Scanning"
        /// display.setTextAlignment(TEXT_ALIGN_CENTER);
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
        display.setTextColor(WHITE);
        int16_t x1, y1;
        uint16_t w, h;

        // Measure the text dimensions
        display.getTextBounds(s.c_str(), 0, text_y, &x1, &y1, &w, &h);
        int x = (128 - w) / 2;

        display.setCursor(x, text_y);
        display.print(s);
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

    if (scan_pages_sz == 1)
    {
#ifdef DEBUG
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(pos_x, text_y, String(loop_time));
#else
        // display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setCursor(pos_x, text_y);
        display.print(String(CONF_FREQ_BEGIN));

#endif
        // display.setTextAlignment(TEXT_ALIGN_RIGHT);

        int16_t x1, y1;
        uint16_t w, h;
        String end = String(CONF_FREQ_END);
        // Measure the text dimensions
        display.getTextBounds(end.c_str(), 0, text_y, &x1, &y1, &w, &h);

        display.setCursor(display.width() - w, text_y);
        display.print(String(CONF_FREQ_END));
    }
    else if (scan_pages_sz > 1)
    {
        // display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setCursor(pos_x, text_y);
        display.print(String(scan_pages[scan_page].start_mhz) + "-" +
                      String(scan_pages[scan_page].end_mhz));
        if (scan_page + 1 < scan_pages_sz)
        {
            // display.setTextAlignment(TEXT_ALIGN_RIGHT);
            display.setCursor(pos_x + width, text_y);
            display.print(String(scan_pages[scan_page + 1].start_mhz) + "-" +
                          String(scan_pages[scan_page + 1].end_mhz));
        }
    }
    ui_initialized = true;
}
