#ifndef _LORA_CONFIG_H
#define _LORA_CONFIG_H

#include <SD.h>

#define CREATE_MISSING_CONFIG true
struct Config
{
    bool create_missing_config;
    bool print_profile_time;
    int log_data_json_interval;
    String listen_on_serial0;
    String listen_on_serial1;
    String listen_on_usb;

    Config()
        : create_missing_config(CREATE_MISSING_CONFIG), print_profile_time(false),
          log_data_json_interval(1000), listen_on_serial0(String("none")),
          listen_on_serial1(String("readline")), listen_on_usb("readline") {};
    bool write_config(const char *path);

    static Config init();
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
