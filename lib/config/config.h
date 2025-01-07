#ifndef _LORA_CONFIG_H
#define _LORA_CONFIG_H

#include <SD.h>

template <typename L, typename R> struct Result
{
    bool is_ok;
    union
    {
        L not_ok;
        R ok;
    };
};

struct ScanRange
{
    uint64_t start_khz;
    uint64_t end_khz;
    uint64_t step_khz;
};

struct ScanPage
{
    uint64_t start_mhz;
    uint64_t end_mhz;
    size_t page_sz;
    ScanRange *scan_ranges;

    ~ScanPage()
    {
        if (page_sz > 0)
        {
            delete[] scan_ranges;
        }
    }
};

struct LoRaConfig
{
    float freq;
    uint16_t bw;
    uint8_t sf;
    uint8_t cr;
    uint8_t tx_power;
    uint16_t preamble_len;
    uint8_t sync_word;
    bool crc;
    uint8_t implicit_header;
};

LoRaConfig *configureLora(String cfg);

#ifndef FREQ_RX
#define FREQ_RX 866
#endif

#ifndef FREQ_TX
#define FREQ_TX (FREQ_RX + 2)
#endif

#define DEFAULT_RX String("freq:") + String(FREQ_RX)
#define DEFAULT_TX String("freq:") + String(FREQ_TX)

#ifndef DEFAULT_TX_SCAN_ON_BOOT
#define DEFAULT_TX_SCAN_ON_BOOT false
#endif

#ifndef DEFAULT_IS_LORA_HOST
#define DEFAULT_IS_LORA_HOST false
#endif

#ifndef DEFAULT_LORA_ENABLED
#define DEFAULT_LORA_ENABLED false
#endif

#ifndef DEFAULT_LORA_TX_POWER
#define DEFAULT_LORA_TX_POWER 1
#endif

#ifndef DEFAULT_LORA_SF
#define DEFAULT_LORA_SF 7
#endif

#define CREATE_MISSING_CONFIG true
struct Config
{
    bool create_missing_config;
    bool print_profile_time;
    String detection_strategy;
    int samples;
    size_t scan_ranges_sz;
    ScanRange *scan_ranges;
    int log_data_json_interval;
    String listen_on_serial0;
    String listen_on_serial1;
    String listen_on_usb;
    LoRaConfig *rx_lora;
    LoRaConfig *tx_lora;

    bool is_host;
    bool lora_enabled;

    Config()
        : create_missing_config(CREATE_MISSING_CONFIG), print_profile_time(false),
          detection_strategy(String("RSSI")), samples(0), scan_ranges_sz(0),
          scan_ranges(NULL), log_data_json_interval(1000),
          listen_on_serial0(String("none")), listen_on_serial1(String("readline")),
          listen_on_usb(String("readline")), rx_lora(configureLora(DEFAULT_RX)),
          tx_lora(configureLora(DEFAULT_TX)), is_host(DEFAULT_IS_LORA_HOST),
          lora_enabled(DEFAULT_LORA_ENABLED) {};

    bool write_config(const char *path);

    static Config init();
    bool updateConfig(String key, String value);
    String getConfig(String key);

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
