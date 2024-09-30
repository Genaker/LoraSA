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
        display.setColor(BLACK);
        display.drawVerticalLine(pos_x + x, pos_y, y);
        display.setColor(WHITE);
        display.drawVerticalLine(pos_x + x, pos_y + y, height - y);
    }
    else
    {
        display.setColor(BLACK);
        display.drawVerticalLine(pos_x + x, pos_y, height);
    }

    if (x % 2 == 0)
    {
        display.setColor(INVERSE);
        display.setPixel(pos_x + x, pos_y + y2pos(level_y));
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

void DecoratedBarChart::reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    Chart::reset(x, y, w, h);
    bar.reset(x, y + LABEL_HEIGHT, w, h - LABEL_HEIGHT - AXIS_HEIGHT);
}

void DecoratedBarChart::draw()
{
    bool draw_axis = bar.redraw_all;

    bar.draw();

    display.setColor(BLACK);
    display.fillRect(pos_x, pos_y, width, bar.pos_y - pos_y);

    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

    for (uint16_t x = 0; x < width; x++)
    {
        float y = bar.ys[x];
        if (y >= bar.level_y)
        {
            String s = String(bar.ys[x], 0);
            uint16_t w = display.getStringWidth(s);
            uint16_t x1 = x;
            for (; x < x1 + w; x++)
            {
                if (bar.ys[x] > y)
                {
                    y = bar.ys[x];
                    s = String(y, 0);
                    w = max(w, display.getStringWidth(s));
                }
            }

            display.drawString(x1, pos_y, s);
        }
    }

    if (draw_axis)
    {
        display.setColor(WHITE);

        uint16_t y = pos_y + height - AXIS_HEIGHT + 2;
        display.fillRect(pos_x, y - 1, width, X_AXIS_WEIGHT);

        // Start and end ticks
        display.fillRect(pos_x, y - 1, 2, AXIS_HEIGHT);
        display.fillRect(pos_x + width - 2, y - 1, 2, AXIS_HEIGHT);

        for (float step = 0; bar.min_x + step * MAJOR_TICKS < bar.max_x; step += 1)
        {
            int tick_pos = bar.x2pos(bar.min_x + step * MAJOR_TICKS);
            display.drawVerticalLine(pos_x + tick_pos, y, MAJOR_TICK_LENGTH);
        }

        for (float step = 0; bar.min_x + step * MINOR_TICKS < bar.max_x; step += 1)
        {
            int tick_pos = bar.x2pos(bar.min_x + step * MINOR_TICKS);
            display.drawVerticalLine(pos_x + tick_pos, y, MINOR_TICK_LENGTH);
        }
    }
}
