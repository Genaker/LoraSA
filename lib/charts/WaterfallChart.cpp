#include "charts.h"
#include <cstdint>

void WaterfallChart::reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    Chart::reset(x, y, w, h);

    model->reset(model->times[0], w);

    update_to = model->buckets;
}

void WaterfallChart::updatePoint(uint64_t t, float x, float y)
{
    if (x < min_x || x >= max_x)
    {
        return;
    }

    update_to = max(update_to, model->updateModel(t, x2pos(x), y >= level_y));
}

void WaterfallChart::draw()
{
    size_t h = min(update_to, (size_t)height);

    for (int y = 0; y < h; y++)
    {
        for (int x = 0; x < width; x++)
        {
            bool b = model->counts[y][x] > 0 &&
                     (model->events[y][x] >= model->counts[y][x] * threshold);
            if (b)
            {
                display.setColor(WHITE);
            }
            else
            {
                display.setColor(BLACK);
            }

            display.setPixel(pos_x + x, pos_y + y);
        }
    }

    update_to = 0;
}

int WaterfallChart::x2pos(float x)
{
    if (x < min_x)
        x = min_x;
    if (x > max_x)
        x = max_x;

    return width * (x - min_x) / (max_x - min_x);
}
