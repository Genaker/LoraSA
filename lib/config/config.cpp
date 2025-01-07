#include "config.h"

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.path(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

#define LORA_CONFIG "/lora_config.txt"
Config Config::init()
{
    if (SD.cardType() == sdcard_type_t::CARD_NONE)
    {
        Serial.println("No SD card found, will assume defaults");
        return Config();
    }

    File f = SD.open(LORA_CONFIG, FILE_READ);
    if (!f)
    {
        Serial.println("Listing root directory:");
        listDir(SD, "/", 0);
        Serial.println("Config file " LORA_CONFIG " not found, will assume defaults");
        Config cfg;

        if (cfg.create_missing_config)
        {
            cfg.write_config(LORA_CONFIG);
        }
        return cfg;
    }

    Config c = Config();

    while (f.available() > 0)
    {
        String ln = f.readStringUntil('\n');
        ParseResult r = parse_config_line(ln);

        if (r.error.length() > 0)
        {
            Serial.printf("%s in '%s', will assume defaults\n", r.error, ln);
            return Config();
        }

        if (r.key.length() == 0)
        {
            // blank line or comment - skip
            continue;
        }

        // do something with known keys and values
        if (c.updateConfig(r.key, r.value))
        {
            continue;
        }

        if (r.key.equalsIgnoreCase("rx_lora"))
        {
            c.rx_lora = configureLora(r.value);
            continue;
        }

        if (r.key.equalsIgnoreCase("tx_lora"))
        {
            c.tx_lora = configureLora(r.value);
            continue;
        }

        Serial.printf("Unknown key '%s' will be ignored\n", r.key);
    }

    f.close();
    return c;
}

#define UPDATE_BOOL(val, key, value)                                                     \
    if (key.equalsIgnoreCase(#val))                                                      \
    {                                                                                    \
        String v = value;                                                                \
        bool p = v.equalsIgnoreCase("true");                                             \
        if (!p && !v.equalsIgnoreCase("false"))                                          \
        {                                                                                \
            Serial.printf("Expected bool for '%s', found '%s' - ignoring\n",             \
                          key.c_str(), value.c_str());                                   \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            val = p;                                                                     \
        }                                                                                \
        return true;                                                                     \
    }

bool Config::updateConfig(String key, String value)
{
    UPDATE_BOOL(print_profile_time, key, value);

    if (key.equalsIgnoreCase("log_data_json_interval"))
    {
        log_data_json_interval = value.toInt();
        return true;
    }

    if (key.equalsIgnoreCase("listen_on_serial0"))
    {
        listen_on_serial0 = value;
        return true;
    }

    if (key.equalsIgnoreCase("listen_on_serial1"))
    {
        listen_on_serial1 = value;
        return true;
    }

    if (key.equalsIgnoreCase("listen_on_usb"))
    {
        listen_on_serial0 = value;
        return true;
    }

    if (key.equalsIgnoreCase("detection_strategy"))
    {
        configureDetectionStrategy(value);
        return true;
    }

    if (key.equalsIgnoreCase("rx_lora"))
    {
        rx_lora = configureLora(value);
        return true;
    }

    if (key.equalsIgnoreCase("tx_lora"))
    {
        tx_lora = configureLora(value);
        return true;
    }

    UPDATE_BOOL(is_host, key, value);

    UPDATE_BOOL(lora_enabled, key, value);

    return false;
}

String loraConfigToStr(LoRaConfig *cfg)
{
    if (cfg == NULL)
    {
        return String("none");
    }

    return String("freq:") + String(cfg->freq) + String(",bw:") + String(cfg->bw) +
           String(",sf:") + String(cfg->sf) + String(",cr:") + String(cfg->cr) +
           String(",tx_power:") + String(cfg->tx_power) + String(",preamble_len:") +
           String(cfg->preamble_len) + String(",sync_word:") +
           String(cfg->sync_word, 16) + String(",crc:") + String(cfg->crc ? "1" : "0") +
           String(",implicit_header:") + String(cfg->implicit_header);
}

String detectionStrategyToStr(Config &c)
{
    String res = c.detection_strategy;
    if (c.scan_ranges_sz > 0)
    {
        res += ":";
        for (int i = 0; i < c.scan_ranges_sz; i++)
        {
            if (i > 0)
            {
                res += ",";
            }

            res += String(c.scan_ranges[i].start_khz);
            int s = c.scan_ranges[i].end_khz - c.scan_ranges[i].start_khz;
            if (s > 0)
            {
                res += ".." + String(c.scan_ranges[i].end_khz);
                if (c.scan_ranges[i].step_khz < s)
                {
                    res += "+" + String(c.scan_ranges[i].step_khz);
                }
            }
        }
    }
    return res;
}

// findSepa looks for a sepa in s from position begin, and does two things:
// updates end with the index of the end of the string between begin and
// separator or end of string, and returns index of the next search position
// or -1 to stop the search.
//
// So you can do something like this:
// for (int i = 0, j = 0, k = 0; (i = findSepa(s, sepa, j, k)) >= 0; j = i)
// ...s.substring(j, k)
int findSepa(String s, String sepa, int begin, int &end)
{
    int i = s.indexOf(sepa, begin);
    if (i < 0)
    {
        end = s.length();
        return begin == end ? -1 : end;
    }

    end = i;
    return i + sepa.length();
}

uint64_t fromHex(String s)
{
    uint64_t r = 0;
    for (char c : s)
    {
        uint64_t v;
        if (c >= 'A' && c <= 'F')
        {
            v = c - 'A' + 10;
        }
        else if (c >= 'a' && c <= 'f')
        {
            v = c - 'a' + 10;
        }
        else
        {
            v = c - '0';
        }

        r = (r << 4) + v;
    }

    return r;
}

uint64_t toUint64(String s)
{
    String v = s.substring(0, s.length() % 15);
    s = s.substring(v.length());
    uint64_t r = v.toInt();

    while (s.length() > 0)
    {
        r = r * ((uint64_t)1000000000000000ll) + s.substring(0, 15).toInt();
        s = s.substring(15);
    }
    return r;
}

ScanRange parseScanRange(String &cfg, int &begin)
{
    ScanRange res;
    int end;
    int i = findSepa(cfg, ",", begin, end);

    String r = cfg.substring(begin, end);
    begin = i;

    if (i < 0)
    {
        res.start_khz = -1;
        res.end_khz = -1;
        res.step_khz = -1;
        return res;
    }

    i = r.indexOf("..");
    if (i < 0)
    {
        i = r.length();
    }

    res.start_khz = toUint64(r.substring(0, i));
    if (i == r.length())
    {
        res.end_khz = res.start_khz;
        res.step_khz = 0;
        return res;
    }

    r = r.substring(i + 2);
    i = r.indexOf("+");
    if (i < 0)
    {
        i = r.indexOf("/");
        if (i < 0)
            i = r.length();
    }

    res.end_khz = toUint64(r.substring(0, i));
    if (i == r.length())
    {
        res.step_khz = res.end_khz - res.start_khz;
    }
    else
    {
        res.step_khz = toUint64(r.substring(i + 1));
        if (r.charAt(i) == '/')
        {
            // then it is not a literal increment, it is the number of steps
            if (res.step_khz == 0)
                res.step_khz = 1;
            res.step_khz = round(((double)(res.end_khz - res.start_khz)) / res.step_khz);
        }
    }

    return res;
}

LoRaConfig *configureLora(String cfg)
{
    if (cfg.equalsIgnoreCase("none"))
    {
        return NULL;
    }

    LoRaConfig *lora = new LoRaConfig({
        freq : 0,
        bw : 500,
        sf : DEFAULT_LORA_SF,
        cr : 5,
        tx_power : DEFAULT_LORA_TX_POWER,
        preamble_len : 8,
        sync_word : 0x1e,
        crc : false,
        implicit_header : 0
    });

    int begin = 0;
    int end, i;

    while ((i = findSepa(cfg, ",", begin, end)) >= 0)
    {
        String param = cfg.substring(begin, end);
        begin = i;
        int j = param.indexOf(":");
        if (j < 0)
        {
            Serial.printf("Expected ':' to be present in '%s' - ignoring config\n",
                          param);
            continue;
        }

        String k = param.substring(0, j);
        param = param.substring(j + 1);

        if (k.equalsIgnoreCase("sync_word"))
        {
            lora->sync_word = (uint8_t)fromHex(param);
            continue;
        }

        if (k.equalsIgnoreCase("freq"))
        {
            lora->freq = param.toFloat();
            continue;
        }

        int v = param.toInt();

        if (k.equalsIgnoreCase("bw"))
        {
            lora->bw = (uint16_t)v;
            continue;
        }

        if (k.equalsIgnoreCase("sf"))
        {
            lora->sf = (uint8_t)v;
            continue;
        }

        if (k.equalsIgnoreCase("cr"))
        {
            lora->cr = (uint8_t)v;
            continue;
        }

        if (k.equalsIgnoreCase("tx_power"))
        {
            lora->tx_power = (uint8_t)v;
            continue;
        }

        if (k.equalsIgnoreCase("preamble_len"))
        {
            lora->preamble_len = (uint8_t)v;
            continue;
        }

        if (k.equalsIgnoreCase("crc"))
        {
            lora->crc = v != 0;
            continue;
        }

        if (k.equalsIgnoreCase("implicit_header"))
        {
            lora->implicit_header = (uint8_t)v;
            continue;
        }

        Serial.printf("Unknown key '%s' will be ignored\n", k);
    }

    return lora;
}

void Config::configureDetectionStrategy(String cfg)
{
    if (scan_ranges_sz > 0)
        delete[] scan_ranges;
    scan_ranges = NULL;

    String method = cfg;
    int i = cfg.indexOf(":");
    if (i >= 0)
    {
        method = cfg.substring(0, i);
        cfg = cfg.substring(i + 1);
    }
    else
    {
        cfg = "";
    }

    samples = 0;
    i = method.indexOf(",");
    if (i >= 0)
    {
        samples = method.substring(i + 1).toInt();
        method = method.substring(0, i);
    }
    detection_strategy = method;

    scan_ranges_sz = 0;
    for (int i = 0, k = 0; (i = findSepa(cfg, ",", i, k)) >= 0; scan_ranges_sz++)
        ;

    if (scan_ranges_sz == 0)
    {
        return;
    }

    scan_ranges = new ScanRange[scan_ranges_sz];

    for (int i = 0, j = 0; i < scan_ranges_sz; i++)
    {
        scan_ranges[i] = parseScanRange(cfg, j);
    }
}

bool Config::write_config(const char *path)
{
    File f = SD.open(path, FILE_WRITE, /*create = */ true);
    if (!f)
    {
        return false;
    }

    f.println("print_profile_time = " + getConfig("print_profile_time"));
    f.println("log_data_json_interval = " + getConfig("log_data_json_interval"));
    f.println("listen_on_serial0 = " + getConfig("listen_on_serial0"));
    f.println("listen_on_serial1 = " + getConfig("listen_on_serial1"));
    f.println("listen_on_usb = " + getConfig("listen_on_usb"));

    f.println("detection_strategy = " + getConfig("detection_strategy"));

    f.println("rx_lora = " + getConfig("rx_lora"));
    f.println("tx_lora = " + getConfig("tx_lora"));
    f.println("is_host = " + getConfig("is_host"));

    f.println("lora_enabled = " + getConfig("lora_enabled"));

    f.close();
    return true;
}

String Config::getConfig(String key)
{
    if (key.equalsIgnoreCase("print_profile_time"))
    {
        return String(print_profile_time ? "true" : "false");
    }

    if (key.equalsIgnoreCase("log_data_json_interval"))
    {
        return String(log_data_json_interval);
    }

    if (key.equalsIgnoreCase("listen_on_serial0"))
    {
        return listen_on_serial0;
    }

    if (key.equalsIgnoreCase("listen_on_serial1"))
    {
        return listen_on_serial1;
    }

    if (key.equalsIgnoreCase("listen_on_usb"))
    {
        return listen_on_usb;
    }

    if (key.equalsIgnoreCase("detection_strategy"))
    {
        return detectionStrategyToStr(*this);
    }

    if (key.equalsIgnoreCase("rx_lora"))
    {
        return loraConfigToStr(rx_lora);
    }

    if (key.equalsIgnoreCase("tx_lora"))
    {
        return loraConfigToStr(tx_lora);
    }

    if (key.equalsIgnoreCase("is_host"))
    {
        return String(is_host ? "true" : "false");
    }

    return "";
}

ParseResult parse_config_line(String ln)
{
    ln.trim();
    if (ln.length() == 0 || ln.charAt(0) == '#')
    {
        // blank line or comment - skip
        return ParseResult(String(), String());
    }

    int i = ln.indexOf("="); // ok, this must exist
    if (i < 0)
    {
        return ParseResult(String("Broken config: expected '='"));
    }

    String k = ln.substring(0, i);
    k.trim();

    String v = ln.substring(i + 1);
    v.trim();

    if (v.length() == 0 || v.charAt(0) == '#')
    {
        return ParseResult(String("Broken config: expected non-empty value"));
    }

    if (v.charAt(0) == '"')
    {
        // quoted strings get special treatment
        int i = v.indexOf('"', 1);
        while (i > 0)
        {
            if (v.length() == i + 1 || v.charAt(i + 1) != '"')
                break;

            v = v.substring(0, i + 1) + v.substring(i + 2);
            i = v.indexOf('"', i + 1);
        }

        if (i < 0)
        {
            return ParseResult(String("Broken config: expected closing quotes"));
        }

        String c = v.substring(i + 1);
        c.trim();
        if (c.length() > 0 && c.charAt(0) != '#')
        {
            return ParseResult(
                String("Broken config: expected nothing but whitespace and "
                       "comments after value"));
        }

        v = v.substring(1, i);
    }
    else
    {
        int i = v.indexOf('#');
        if (i > 0)
        {
            v = v.substring(0, i);
            v.trim();
        }
    }

    return ParseResult(k, v);
}
