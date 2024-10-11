#include "charts.h"

void UptimeClock::draw(uint64_t t)
{
    t1 = t;
    draw();
}

void UptimeClock::draw()
{
    uint64_t uptime = t1 - t0;
    int mils = uptime % 1000;
    int seconds = (uptime / 1000) % 60;
    int minutes = (uptime / 60000) % 60;
    int hours = uptime / 3600000;
    String s = String(hours) + (minutes < 10 ? ":0" : ":") + String(minutes) +
               (seconds < 10 ? ":0" : ":") + String(seconds) +
               (mils < 10    ? ".00"
                : mils < 100 ? ".0"
                             : ".") +
               String(mils);
    int w = display.getStringWidth(s);
    display.setColor(BLACK);
    display.fillRect((display.width() - w) / 2, display.height() / 2 - 3, w, 7);
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(display.width() / 2, display.height() / 2, s);
}
