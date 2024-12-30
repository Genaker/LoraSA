#ifndef CHARTS_H
#define CHARTS_H

#ifdef Vision_Master_E290
#include "HT_DEPG0290BxS800FxX_BW.h"
typedef DEPG0290BxS800FxX_BW Display_t;
#else
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
typedef Adafruit_SSD1306 Display_t;
#endif

#include <cstdint>
#include <events.h>
#include <models.h>
#include <stdlib.h>

struct Chart
{
    uint16_t pos_x, pos_y;
    uint16_t width, height;
    Display_t &display;

    Chart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
        : display(d), pos_x(x), pos_y(y), width(w), height(h) {};

    /*
     * This method resets the state and sets the reference time.
     */
    virtual void reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
    {
        pos_x = x;
        pos_y = y;
        width = w;
        height = h;
    }

    /*
     * Redraw everything that needs redrawing.
     */
    virtual void draw() {};
};

/*
 * ProgressChart supports updates with progressive redraw of just the affected area.
 */
struct ProgressChart : Chart
{
    ProgressChart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
        : Chart(d, x, y, w, h) {};

    /*
     * Update one data point, and return what column needs redrawing.
     */
    virtual int updatePoint(float x, float y) = 0;

    /*
     * If you fancy animated progress, then pass the output of updatePoint to here.
     */
    virtual void drawOne(int x) = 0;
};

struct BarChart : ProgressChart, Listener
{
    float min_x, max_x, min_y, max_y;
    float level_y;

    float *ys;
    bool *changed;
    bool redraw_all;

    BarChart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float min_x,
             float max_x, float min_y, float max_y, float level_y)
        : ProgressChart(d, x, y, w, h), min_x(min_x), max_x(max_x), min_y(min_y),
          max_y(max_y), level_y(level_y), redraw_all(true)
    {
        ys = new float[w];
        changed = new bool[w];

        memset(ys, 0, w * sizeof(float));
        memset(changed, 0, w * sizeof(bool));
    };

    void reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;
    int updatePoint(float x, float y) override;
    void drawOne(int x) override;
    void draw() override;
    void onEvent(Event &) override;

    int x2pos(float x);
    int y2pos(float y);
};

#define LABEL_HEIGHT 7
#define X_AXIS_WEIGHT 1
#define MAJOR_TICK_LENGTH 2
#define MAJOR_TICKS 10
#define MINOR_TICK_LENGTH 1
#define MINOR_TICKS 5
#define AXIS_HEIGHT (X_AXIS_WEIGHT + MAJOR_TICK_LENGTH + 2)
struct DecoratedBarChart : Chart
{
    BarChart bar;

    DecoratedBarChart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      float min_x, float max_x, float min_y, float max_y, float level_y)
        : Chart(d, x, y, w, h),
          bar(d, x, y + LABEL_HEIGHT, w, h - LABEL_HEIGHT - AXIS_HEIGHT, min_x, max_x,
              min_y, max_y, level_y) {};

    void reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;
    void draw() override;
};

struct StackedChart : Chart, Listener
{
    Chart **charts;
    size_t charts_sz;

    StackedChart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
        : Chart(d, x, y, w, h), charts(NULL), charts_sz(0) {};

    /*
     * addChart adds c to the StackedChart, treats pos_x and pos_y of the chart
     * as relative to this chart's origin, and trims width to fit. Adjust the
     * height and pack charts using setHeight.
     */
    size_t addChart(Chart *c);

    /*
     * Adjust the height of the chart and return the resulting required height.
     * If h is >= height, the chart gets a special treatment: packs all other
     * charts, and uses up the rest of space.
     */
    uint16_t setHeight(size_t c, uint16_t h);

    void reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;

    void draw() override;

    void onEvent(Event &e) override;
};

struct WaterfallChart : Chart, Listener
{
    float min_x, max_x;
    float level_y, threshold;

    size_t update_to;

    WaterfallModel *model;

    WaterfallChart(Display_t &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                   float min_x, float max_x, float level_y, float threshold,
                   WaterfallModel *m)
        : Chart(d, x, y, w, h), model(m), min_x(min_x), max_x(max_x), level_y(level_y),
          threshold(threshold), update_to(m->buckets) {};

    void updatePoint(uint64_t t, float x, float y);

    void reset(uint16_t x, uint16_t y, uint16_t w, uint16_t h) override;

    void draw() override;
    void onEvent(Event &e) override;

    int x2pos(float x);
};

struct UptimeClock : Chart
{
    uint64_t t0;
    uint64_t t1;
    UptimeClock(Display_t &d, uint64_t t0) : Chart(d, 0, 0, 0, 0), t0(t0), t1(t0) {};

    void draw(uint64_t t);
    virtual void draw() override;
};

#endif
