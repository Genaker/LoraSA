#ifndef _LORA_CONFIG_H
#define _LORA_CONFIG_H

#include <SD.h>

struct ScanRange
{
    uint64_t start_khz;
    uint64_t end_khz;
    uint64_t step_khz;
};

#define CREATE_MISSING_CONFIG true
struct Config
{
    bool create_missing_config;
    bool print_profile_time;
    String detection_strategy;
    int samples;
    int scan_ranges_sz;
    ScanRange *scan_ranges;
    int log_data_json_interval;
    String listen_on_serial0;
    String listen_on_serial1;
    String listen_on_usb;

    Config()
        : create_missing_config(CREATE_MISSING_CONFIG), print_profile_time(false),
          detection_strategy("RSSI"), samples(0), scan_ranges_sz(0), scan_ranges(NULL),
          log_data_json_interval(1000), listen_on_serial0(String("none")),
          listen_on_serial1(String("readline")), listen_on_usb("readline") {};
    bool write_config(const char *path);

    static Config init();

    void configureDetectionStrategy(String cfg);
};

struct ParseResult
{
    String key;
    String value;
    String error;

    ParseResult(String e) : key(String()), value(String()), error(e) {};
    ParseResult(String k, String v) : key(k), value(v), error(String()) {};
};

ParseResult parse_config_line(String ln);
#endif
