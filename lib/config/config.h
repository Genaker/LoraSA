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
    uint16_t crc_seed;
    uint16_t crc_poly;
    uint8_t implicit_header;
};

LoRaConfig *configureLora(String cfg);

struct BusConfig
{
    enum
    {
        NONE, // No bus
        UART, // Serial
        SPI,
        WIRE // I2C
    } bus_type;

    int8_t bus_num;

    bool enabled;

    uint32_t clock_freq;

    union
    {
        int8_t scl;
        int8_t clk;
    };

    union
    {
        int8_t sda;
        int8_t mosi;
        int8_t tx;
    };

    union
    {
        int8_t miso;
        int8_t rx;
    };

    BusConfig()
        : bus_type(NONE), bus_num(-1), enabled(false), clock_freq(115200), clk(-1),
          rx(-1), tx(-1) {};

    static BusConfig configure(String cfg);
    String toStr();
};

#ifndef RADIO_MODULE_CS_PIN
#define RADIO_MODULE_CS_PIN 38
#endif

#ifndef RADIO_MODULE_DIO1_PIN
#define RADIO_MODULE_DIO1_PIN 40
#endif

#ifndef RADIO_MODULE_RST_PIN
#define RADIO_MODULE_RST_PIN 41
#endif

#ifndef RADIO_MODULE_BUSY_PIN
#define RADIO_MODULE_BUSY_PIN 39
#endif

#ifndef RADIO_MODULE_CLOCK_FREQ
#define RADIO_MODULE_CLOCK_FREQ 16000000
#endif

#ifndef RADIO_MODULE_MSB_FIRST
#define RADIO_MODULE_MSB_FIRST 1
#endif

#ifndef RADIO_MODULE_SPI_MODE
#define RADIO_MODULE_SPI_MODE 0
#endif

struct RadioModuleSPIConfig
{
    bool enabled;
    String module;
    int8_t bus_num;
    int8_t cs;
    int8_t rst;
    int8_t dio1;
    int8_t busy;

    uint32_t clock_freq;
    bool msb_first;
    int8_t spi_mode;

    static RadioModuleSPIConfig configure(String cfg);
    String toStr();
};

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

#ifndef DEFAULT_UART0
#define DEFAULT_UART0 "none"
#endif

#ifndef DEFAULT_UART1
#define DEFAULT_UART1 "u1:115200"
#endif

#ifndef DEFAULT_SPI1
#define DEFAULT_SPI1 "s1:16000000,clk:42,mosi:46,miso:45"
#endif

#ifndef DEFAULT_WIRE1
#define DEFAULT_WIRE1 "none"
#endif

#ifndef DEFAULT_RADIO2
#define DEFAULT_RADIO2 "none"
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

    BusConfig uart0;
    BusConfig uart1;
    BusConfig spi1;
    BusConfig wire1;
    RadioModuleSPIConfig radio2;

    bool is_host;
    bool lora_enabled;

    Config()
        : create_missing_config(CREATE_MISSING_CONFIG), print_profile_time(false),
          detection_strategy(String("RSSI")), samples(0), scan_ranges_sz(0),
          scan_ranges(NULL), log_data_json_interval(1000),
          listen_on_serial0(String("none")), listen_on_serial1(String("readline")),
          listen_on_usb(String("readline")), rx_lora(configureLora(DEFAULT_RX)),
          tx_lora(configureLora(DEFAULT_TX)), is_host(DEFAULT_IS_LORA_HOST),
          uart0(BusConfig::configure(DEFAULT_UART0)),
          uart1(BusConfig::configure(DEFAULT_UART1)),
          spi1(BusConfig::configure(DEFAULT_SPI1)),
          wire1(BusConfig::configure(DEFAULT_WIRE1)), lora_enabled(DEFAULT_LORA_ENABLED),
          radio2(RadioModuleSPIConfig::configure(DEFAULT_RADIO2)) {};

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
