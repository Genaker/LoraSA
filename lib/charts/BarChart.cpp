#include "charts.h"

void BarChart::reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if (w != width)
    {
        delete[] ys;
        delete[] changed;

        ys = new float[w];
        changed = new bool[w];
    }

    memset(ys, 0, w * sizeof(float));
    memset(changed, false, w * sizeof(bool));
    redraw_all = true;

    Chart::reset(x, y, w, h);
}

int BarChart::updatePoint(float x, float y)
{
    if (x < min_x || x >= max_x)
    {
        return -1;
    }

    size_t idx = width * (x - min_x) / (max_x - min_x);
    if (idx >= width)
    {
        idx = width - 1;
    }

    if (!changed[idx] || ys[idx] < y)
    {
        ys[idx] = y;
        changed[idx] = true;
    }

    return idx;
}

void BarChart::draw()
{
    for (int x = 0; x < width; x++)
    {
        if (!changed[x] && !redraw_all)
            continue;

        drawOne(x);
    }

    redraw_all = false;
}

void BarChart::drawOne(int x)
{
    if (x < 0)
        return;

    int y = y2pos(ys[x]);

    if (y < height)
    {
        // display.setColor(BLACK);
        display.drawFastVLine(pos_x + x, pos_y, y, BLACK);
        // display.setColor(WHITE);
        display.drawFastVLine(pos_x + x, pos_y + y, height - y, WHITE);
    }
    else
    {
        // display.setColor(BLACK);
        display.drawFastVLine(pos_x + x, pos_y, height, BLACK);
    }

    if (x % 2 == 0)
    {
        // display.setColor(INVERSE);
        display.drawPixel(pos_x + x, pos_y + y2pos(level_y), INVERSE);
    }

    changed[x] = false;
}

int BarChart::x2pos(float x)
{
    if (x < min_x)
        x = min_x;
    if (x > max_x)
        x = max_x;

    return width * (x - min_x) / (max_x - min_x);
}

int BarChart::y2pos(float y)
{
    if (y < min_y)
        y = min_y;
    if (y > max_y)
        y = max_y;

    return height - height * (y - min_y) / (max_y - min_y);
}

void BarChart::onEvent(Event &e)
{
    if (e.type != DETECTED)
    {
        return;
    }

    level_y = e.emitter.trigger_level;

    int u = updatePoint(e.emitter.current_frequency, e.detected.rssi);
    if (e.emitter.animated)
    {
        drawOne(u);
    }
}

void DecoratedBarChart::reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    Chart::reset(x, y, w, h);
    bar.reset(x, y + LABEL_HEIGHT, w, h - LABEL_HEIGHT - AXIS_HEIGHT);
}

void DecoratedBarChart::draw()
{
    bool draw_axis = bar.redraw_all;

    bar.draw();

    // display.setColor(BLACK);
    display.fillRect(pos_x, pos_y, width, bar.pos_y - pos_y, BLACK);

    // display.setColor(WHITE);
    // display.setTextAlignment(TEXT_ALIGN_LEFT);

    uint16_t first_untouched = 0;

    for (uint16_t x = 0; x < width; x++)
    {
        float y = bar.ys[x];
        if (y >= bar.level_y)
        {
            String s = String(bar.ys[x], 0);
            int16_t x0, y0; // Variables to store the top-left corner
            uint16_t w, h0; // Variables to store width and height
            display.getTextBounds(s.c_str(), 0, 0, &x0, &y0, &w, &h0);
            uint16_t x1 = x;
            for (; x < x1 + w && x < width; x++)
            {
                if (bar.ys[x] > y)
                {
                    y = bar.ys[x];
                    s = String(y, 0);
                    int16_t x0, y0;  // Variables to store the top-left corner
                    uint16_t w1, h0; // Variables to store width and height
                    display.getTextBounds(s.c_str(), 0, 0, &x0, &y0, &w1, &h0);
                    w = max(w, w1);
                }
            }

            if (x > width && first_untouched <= width - w)
            {
                x1 = width - w;
            }

            first_untouched = x;

            if (x1 + w <= width)
            {
                // Detected dB level text
                display.setCursor(pos_x + x1 - 10, pos_y);
                display.print(s);
            }
        }
    }

    if (draw_axis)
    {

        uint16_t y = pos_y + height - AXIS_HEIGHT + 2;
        display.fillRect(pos_x, y - 1, width, X_AXIS_WEIGHT, WHITE);

        // Start and end ticks
        display.fillRect(pos_x, y - 1, 2, AXIS_HEIGHT, WHITE);
        display.fillRect(pos_x + width - 2, y - 1, 2, AXIS_HEIGHT, WHITE);

        for (float step = 0; bar.min_x + step * MAJOR_TICKS < bar.max_x; step += 1)
        {
            int tick_pos = bar.x2pos(bar.min_x + step * MAJOR_TICKS);
            display.drawFastVLine(pos_x + tick_pos, y, MAJOR_TICK_LENGTH, WHITE);
        }

        for (float step = 0; bar.min_x + step * MINOR_TICKS < bar.max_x; step += 1)
        {
            int tick_pos = bar.x2pos(bar.min_x + step * MINOR_TICKS);
            display.drawFastVLine(pos_x + tick_pos, y, MINOR_TICK_LENGTH, WHITE);
        }
    }
}
