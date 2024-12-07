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

        if (r.key.equalsIgnoreCase("print_profile_time"))
        {
            String v = r.value;
            bool p = v.equalsIgnoreCase("true");
            if (!p && !v.equalsIgnoreCase("false"))
            {
                Serial.printf("Expected bool for '%s', found '%s' - ignoring\n",
                              r.key.c_str(), r.value.c_str());
            }
            else
            {
                c.print_profile_time = p;
            }
            continue;
        }

        if (r.key.equalsIgnoreCase("log_data_json_interval"))
        {
            c.log_data_json_interval = r.value.toInt();
            continue;
        }

        if (r.key.equalsIgnoreCase("listen_on_serial0"))
        {
            c.listen_on_serial0 = r.value;
            continue;
        }

        if (r.key.equalsIgnoreCase("listen_on_serial1"))
        {
            c.listen_on_serial1 = r.value;
            continue;
        }

        if (r.key.equalsIgnoreCase("listen_on_usb"))
        {
            c.listen_on_serial0 = r.value;
            continue;
        }

        Serial.printf("Unknown key '%s' will be ignored\n", r.key);
    }

    f.close();
    return c;
}

bool Config::write_config(const char *path)
{
    File f = SD.open(path, FILE_WRITE, /*create = */ true);
    if (!f)
    {
        return false;
    }

    f.println("print_profile_time = " + String(print_profile_time ? "true" : "false"));
    f.println("log_data_json_interval = " + String(log_data_json_interval));
    f.println("listen_on_serial0 = " + listen_on_serial0);
    f.println("listen_on_serial1 = " + listen_on_serial1);
    f.println("listen_on_usb = " + listen_on_usb);

    f.close();
    return true;
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
