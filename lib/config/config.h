#ifndef _LORA_CONFIG_H
#define _LORA_CONFIG_H

#include <SD.h>

#define CREATE_MISSING_CONFIG true
struct Config
{
    bool create_missing_config;
    bool print_profile_time;
    int log_data_json_interval;

    Config()
        : create_missing_config(CREATE_MISSING_CONFIG), print_profile_time(false),
          log_data_json_interval(1000) {};
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
