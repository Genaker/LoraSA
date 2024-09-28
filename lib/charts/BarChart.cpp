#include "charts.h"

void BarChart::reset()
{
    memset(ys, 0, width * sizeof(float));
    memset(changed, false, width * sizeof(bool));
    redraw_all = true;
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
