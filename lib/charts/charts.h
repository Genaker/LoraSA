#ifndef CHARTS_H
#define CHARTS_H

#include <OLEDDisplay.h>
#include <cstdint>
#include <stdlib.h>

struct Chart
{
    uint16_t pos_x, pos_y;
    uint16_t width, height;
    OLEDDisplay &display;

    Chart(OLEDDisplay &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
        : display(d), pos_x(x), pos_y(y), width(w), height(h) {};

    /*
     * This method resets the state and sets the reference time.
     */
    virtual void reset() = 0;

    /*
     * Update one data point, and return what column needs redrawing.
     */
    virtual int updatePoint(float x, float y) = 0;

    /*
     * If you fancy animated progress, then pass the output of updatePoint to here.
     */
    virtual void drawOne(int x) = 0;

    /*
     * Redraw everything that needs redrawing.
     */
    virtual void draw() = 0;
};

struct BarChart : Chart
{
    float min_x, max_x, min_y, max_y;
    float level_y;

    float *ys;
    bool *changed;
    bool redraw_all;

    BarChart(OLEDDisplay &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h, float min_x,
             float max_x, float min_y, float max_y, float level_y)
        : Chart(d, x, y, w, h), min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y),
          level_y(level_y), redraw_all(true)
    {
        ys = new float[w];
        changed = new bool[w];

        memset(ys, 0, w * sizeof(float));
        memset(changed, 0, w * sizeof(bool));
    };

    void reset() override;
    int updatePoint(float x, float y) override;
    void drawOne(int x) override;
    void draw() override;

    int x2pos(float x);
    int y2pos(float y);
};

#define LABEL_HEIGHT 6
#define X_AXIS_WEIGHT 1
#define MAJOR_TICK_LENGTH 2
#define MAJOR_TICKS 10
#define MINOR_TICK_LENGTH 1
#define MINOR_TICKS 5
#define AXIS_HEIGHT (X_AXIS_WEIGHT + MAJOR_TICK_LENGTH + 1)
struct DecoratedBarChart : BarChart
{
    int text_y;

    DecoratedBarChart(OLEDDisplay &d, uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      float min_x, float max_x, float min_y, float max_y, float level_y)
        : BarChart(d, x, y + LABEL_HEIGHT, w, h - LABEL_HEIGHT - AXIS_HEIGHT, min_x,
                   max_x, min_y, max_y, level_y),
          text_y(y) {};

    void draw() override;
};

#endif
