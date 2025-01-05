#include "charts.h"

extern const uint8_t ArialMT_Plain_10[]; // borrowed from OLEDDisplayFonts.h

uint8_t *ArialMT_Plain_10_Vert = NULL;

uint64_t _horz_line(const uint8_t *font, size_t chr_off, size_t height, size_t bytes,
                    size_t ln)
{
    size_t raster_height = (height + 7) / 8;
    uint8_t b = 1 << (ln & 7);

    uint64_t ret = 0;
    uint64_t mask = 1;
    for (size_t y = ln / 8; y < bytes; y += raster_height, mask <<= 1)
    {
        if (font[chr_off + y] & b)
        {
            ret |= mask;
        }
    }

    return ret;
}

uint64_t _flip(uint64_t v, size_t width)
{
    for (uint64_t b = 1, e = 1 << (width - 1); b < e; b <<= 1, e >>= 1)
    {
        if (!!(v & b) != !!(v & e))
        {
            v = v ^ (b | e);
        }
    }

    return v;
}

uint8_t *_rot(const uint8_t *font)
{
    size_t sz = 10000;
    size_t p = 0;
    uint8_t *vert = (uint8_t *)malloc(sz);

    size_t height = font[1];
    vert[0] = (uint8_t)height; // height -> width
    vert[1] = font[0];         // width -> height
    vert[2] = font[2];         // first char
    size_t chars = font[3];    // char count
    vert[3] = (uint8_t)chars;

    size_t map_off = chars * 4 + 4;

    for (int i = 0; i < chars; i++)
    {
        uint32_t v = ((uint32_t *)font)[i + 1];
        size_t chr_off = ((v & 0xff) << 8) | ((v >> 8) & 0xff);
        if (chr_off != 0xffff)
        {
            size_t bs = (v >> 16) & 0xff;
            size_t width = (v >> 24) & 0xff;
            size_t raster_height =
                (width + 7) / 8; // how many bytes needed to store one vertical line

            size_t bytes = height * raster_height;
            v = (v & 0xff000000L) | (bytes << 16) | ((p >> 8) & 0xff) | ((p & 0xff) << 8);

            chr_off += map_off;

            if (map_off + p + bytes > sz)
            {
                uint8_t *vv = vert;
                sz += 1000;
                vert = (uint8_t *)malloc(sz);
                memcpy(vert, vv, sz - 1000);
                free(vv);
            }

            for (size_t j = 0; j < height; j++)
            {
                uint64_t ln = _flip(_horz_line(font, chr_off, height, bs, j), width);
                for (size_t k = 0; k < raster_height; k++, p++, ln >>= 8)
                {
                    vert[map_off + p] = (uint8_t)ln;
                }
            }
        }

        ((uint32_t *)vert)[i + 1] = v;
    }

    if (map_off + p < sz)
    {
        uint8_t *vv = vert;
        sz = map_off + p;
        vert = (uint8_t *)malloc(sz);
        memcpy(vert, vv, sz);
        free(vv);
    }
    return vert;
}

void init_fonts()
{
    if (ArialMT_Plain_10_Vert != NULL)
    {
        free(ArialMT_Plain_10_Vert);
        ArialMT_Plain_10_Vert = NULL;
    }

    ArialMT_Plain_10_Vert = _rot(ArialMT_Plain_10);
}

void drawVerticalString(Display_t &display, const uint8_t *v_font, int x, int y, String s)
{
    if (v_font == NULL)
        return;

    size_t w = v_font[0];
    char init_ch = v_font[2];
    size_t chars = v_font[3];

    const uint8_t *v_chars = v_font + (chars + 1) * 4;

    for (int i = s.length(); i-- > 0;)
    {
        char c = s.charAt(i);
        if (c < init_ch)
            continue;
        c -= init_ch;
        if (c >= chars)
            continue;

        size_t map_off = (c + 1) * 4;
        size_t chr_off = (((uint16_t)v_font[map_off]) << 8) | (v_font[map_off + 1]);
        size_t bs = v_font[map_off + 2];
        size_t h = v_font[map_off + 3];
        if (chr_off == 0xffff)
        {
            y += h;
            continue;
        }

        display.drawFastImage(x, y, w, h, v_chars + chr_off);
        y += h;
    }
}
