#include "charts.h"

uint16_t trim_w(uint16_t pos, uint16_t width, uint16_t w)
{
    return min(width, (uint16_t)(max(w, pos) - pos));
}

size_t StackedChart::addChart(Chart *c)
{
    Chart **cc = new Chart *[charts_sz + 1];
    memcpy(cc, charts, charts_sz * sizeof(Chart *));
    cc[charts_sz] = c;
    
    // Use delete[] instead of free since we allocated with new[]
    delete[] charts;

    c->reset(pos_x + c->pos_x, pos_y + c->pos_y, trim_w(c->pos_x, c->width, width),
             c->height);
    charts = cc;
    return charts_sz++;
}

uint16_t StackedChart::setHeight(size_t c, uint16_t h)
{
    if (h < height)
    {
        charts[c]->reset(charts[c]->pos_x, charts[c]->pos_y, charts[c]->width, h);

        uint16_t used_space = 0;
        for (int i = 0; i < charts_sz; i++)
        {
            used_space += charts[i]->height;
        }

        return used_space;
    }
    // this chart gets special treatment - pack all other charts,
    // and make this one as big as possible
    uint16_t used_space = 0;
    for (int i = 0; i < c; i++)
    {
        charts[i]->reset(charts[i]->pos_x, pos_y + used_space, charts[i]->width,
                         charts[i]->height);
        used_space += charts[i]->height;
    }

    uint16_t more_used_space = used_space;
    for (int i = c + 1; i < charts_sz; i++)
    {
        more_used_space += charts[i]->height;
    }

    if (more_used_space < height)
    {
        charts[c]->reset(charts[c]->pos_x, pos_y + used_space, charts[c]->width,
                         height - more_used_space);
        used_space += charts[c]->height;
    }

    for (int i = c + 1; i < charts_sz; i++)
    {
        charts[i]->reset(charts[i]->pos_x, pos_y + used_space, charts[i]->width,
                         charts[i]->height);
        used_space += charts[i]->height;
    }

    return used_space;
}

void StackedChart::reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    for (int i = 0; i < charts_sz; i++)
    {
        uint16_t rel_x = charts[i]->pos_x - pos_x;
        uint16_t rel_y = charts[i]->pos_y - pos_y;
        charts[i]->reset(x + rel_x, y + rel_y, trim_w(rel_x, charts[i]->width, w),
                         charts[i]->height);
    }

    Chart::reset(x, y, w, h);
}

void StackedChart::draw()
{
    for (int i = 0; i < charts_sz; i++)
        charts[i]->draw();
}

void StackedChart::onEvent(Event &e)
{
    if (e.type != SCAN_TASK_COMPLETE)
    {
        return;
    }

    draw();
}
