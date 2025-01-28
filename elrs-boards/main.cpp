#include <Arduino.h>
#include <ELRS_GENERIC_LR1221_DIVERSITY.h>
#include <HardwareSerial.h>
#include <vector>

// Direct access to the low-level SPI communication between RadioLib and the radio module.
#define RADIOLIB_LOW_LEVEL (1)
//  In this mode, all methods and member variables of all RadioLib classes will be made
//  public and so will be exposed to the user. This allows direct manipulation of the
//  library internals.
#define RADIOLIB_GODMODE (1)
#define RADIOLIB_CHECK_PARAMS (0)

#include <RadioLib.h>
#include <SPI.h>
// #include <SPIEx.h>
#include <Wire.h>

SPIClass hspi = SPIClass(HSPI);

#ifdef USING_LR1121
// Default SPI on pins from pins_arduino.h
#define RADIO_TYPE LR1121
#define RADIO_MODULE_INIT()                                                              \
    new Module(RADIO_NSS, RADIO_DIO1, RADIO_RST, RADIO_BUSY, hspi);

#define RADIO_MODULE2_INIT()                                                             \
    new Module(RADIO_NSS_2, RADIO_DIO1_2, RADIO_RST_2, RADIO_BUSY_2, hspi);
#endif // end USING_LR1121

RADIO_TYPE radio = RADIO_MODULE_INIT();
RADIO_TYPE radio2 = RADIO_MODULE2_INIT();

int radioIsHigh = false;
int radio2IsHigh = false;

// SPIClass hspi = new SPIClass(2);

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
SPIExClass SPIEx(FSPI);
#elif defined(PLATFORM_ESP32)
SPIExClass SPIEx(VSPI);
#else
// SPIExClass SPIEx;
#endif //

// Define a struct to represent a single RSSI scan
struct RSSI
{
    float freq;  // Frequency in MHz
    float rssi1; // RSSI value 1
    float rssi2; // RSSI value 2
    // float rssi3; // RSSI value 3 (optional, can add more if needed)
};

bool isSetCommand(const String &command);
bool isGetCommand(const String &command);
void processCommands(const String &command);
bool parseSetCommand(const String &command, int &startFreq, int &endFreq);
bool parseGetCommand(const String &command, int &freq);

String uartGetResponse();
std::vector<RSSI> getRssiRange(int startFreq, int endFreq, float step = 1,
                               float samples = 1, int group = 1);

void getRssi(float freq, float &rssi, float &rssi2);
String printRssiRange(std::vector<RSSI> rssis);
void SetDioAsRfSwitch();
void initRadio(float freq);
String wrapResult(std::vector<RSSI> rssis, bool single = false);

// Function to check if the command starts with "SET"
bool isSetCommand(const String &command) { return command.startsWith("SET "); }

// Function to check if the command starts with "GET"
bool isGetCommand(const String &command) { return command.startsWith("GET "); }

enum class Direction
{
    Left,
    Right
};

String stringToMinLength(String input, int minLength);

void setup()
// https://github.com/ExpressLRS/ExpressLRS/blob/2e61e33723b3247300f95ed46ceee9648d536137/src/lib/LR1121Driver/LR1121_hal.cpp#L6
{
    // pinMode(0, INPUT_PULLUP); // Trying to enable auto boot mode. Notes it makes
    // Boot mode always.
    // TODO: make on button press without reset. or with reset :0
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO1, INPUT);
    pinMode(RADIO_NSS, OUTPUT);
    digitalWrite(RADIO_NSS, HIGH);

    pinMode(RADIO_BUSY_2, INPUT);
    pinMode(RADIO_DIO1_2, INPUT);
    pinMode(RADIO_NSS_2, OUTPUT);
    digitalWrite(RADIO_NSS_2, HIGH);

    hspi.begin(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS);
    gpio_pullup_en((gpio_num_t)RADIO_MISO);
    SPI.setFrequency(17500000);
    SPI.setHwCs(true);
    radio = RADIO_MODULE_INIT();
    radio2 = RADIO_MODULE2_INIT();

    // Initialize Serial Monitor
    Serial.begin(115200);
    delay(1000);
    Serial.println("ELRS TRUE DIVERSITY LR1121 ESP32-PICO-D4!");
    float freq = 900;
    initRadio(freq);
    delay(1000);
}

float freq = 1200;
// Scan Freq in KH
int startFreq = 0, endFreq = 0;
int getFreq = 0;
void loop()
{
    std::vector<RSSI> rssis;
    // getFreq = 0;
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n'); // Read until newline
        command.trim(); // Remove any leading/trailing whitespace

        // Process the command
        processCommands(command);
    }
    if (getFreq != 0)
    {
        freq = (float)getFreq; /// 1000.0f;
        float rssi = 0, rssi2 = 0;
        rssis = getRssiRange(freq, freq, 0.5, 1);
        // getRssi(freq, rssi, rssi2);
        rssi = rssis[0].rssi1;
        rssi2 = rssis[0].rssi2;
        Serial.println("[" + String(freq / 1000) + "Mhz]RSSI: " + String((float)rssi) +
                       " RSSI2: " + String((float)rssi2));
        Serial.println("RSSI1: " + String((float)rssi) +
                       " RSSI2: " + String((float)rssi2));
        int delta = 0;
        if ((int)rssi > (int)rssi2)
        {
            String arrow = "";
            delta = abs(abs(rssi) - abs(rssi2));
            for (int i = 0; i < 4; i++)
            {
                arrow += ">";
            }
            // arrow = stringToMinLength(arrow, 5);
            Serial.println("    " + String((int)rssi) + "[" + String((int)freq / 1000) +
                           "]" + String((int)rssi2) + arrow);
        }
        else if ((int)rssi < (int)rssi2)
        {
            delta = abs(abs(rssi) - abs(rssi2));
            String arrow = "";
            for (int i = 0; i < 4; i++)
            {
                arrow += "<";
            }
            // arrow = stringToMinLength(arrow, -5);
            Serial.println(arrow + String((int)rssi) + "[" + String((int)freq / 1000) +
                           "]" + String((int)rssi2) + "    ");
        }
        else if (abs((int)rssi2) - abs((int)rssi2) == 0)
        {
            Serial.println("    " + String((int)rssi) + "[" + String((int)freq / 1000) +
                           "]" + String((int)rssi2) + "    ");
        }
        // String rssiString = wrapResult(rssis); // printRssiRange(rssis);
        // const char *RSSIutf8Data = rssiString.c_str();
        // Serial.println(RSSIutf8Data);
        delay(1000);
    }
    if (startFreq != 0 && endFreq != 0)
    {
        rssis = getRssiRange(startFreq, endFreq, 1, 1);
        String rssiString = wrapResult(rssis); // printRssiRange(rssis);
        const char *RSSIutf8Data = rssiString.c_str();
        Serial.println(RSSIutf8Data);
    }
}

std::vector<RSSI> getRssiRange(int startFreq, int endFreq, float step, float samples,
                               int group)
{
    long int startTime = millis();
    std::vector<RSSI> rssis;
    if (startFreq > endFreq)
    {
        float temp = startFreq;
        startFreq = endFreq;
        endFreq = temp;
    }
    step = step * 1000; // Step size in MHz
    RSSI scan;
    for (float freq = startFreq; freq <= endFreq; freq += step)
    {
        float rssi = 0, rssi2 = 0;
        float maxRssi = -999, maxRssi2 = -999;
        for (int i = 0; i < samples; i++)
        {
            getRssi((float)freq / 1000.0f, rssi, rssi2); // Call the method to get RSSI
            if (maxRssi < rssi)
            {
                maxRssi = rssi;
            }
            if (maxRssi2 < rssi2)
            {
                maxRssi2 = rssi2;
            }
        }
        rssi = maxRssi;
        rssi2 = maxRssi2;
        /**
        Serial.print("Frequency: ");
        Serial.print(freq, 1); // Print frequency with 1 decimal place
        Serial.print(" KHz, RSSI: ");
        Serial.print(rssi);
        Serial.print(" RSSI2: ");
        Serial.println(rssi2);
        **/
        scan = {freq, rssi, rssi2};
        rssis.push_back(scan);
    }
    long int endTime = millis();
    // Serial.println("Get RSSI TIME: " + String(endTime - startTime));
    return rssis;
}

void getRssi(float freq, float &rssi, float &rssi2)
{
    if (freq > 1000 && !radioIsHigh)
    {
        initRadio(freq);
        radioIsHigh = true;
    }
    if (freq < 1000 && radioIsHigh)
    {
        initRadio(freq);
        radioIsHigh = false;
    }

    int state, state2;
    // Print message periodically
    // Serial.println("Freq: " + String(freq));
    state = radio.setFrequency((float)freq);
    state2 = radio2.setFrequency((float)freq);
    usleep(100);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:setFrequency:" + String(state));
    }
    if (state2 != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error2:setFrequency:" + String(state2));
    }

#if defined(USING_LR1121)
    // Try getRssiInst
    radio.getRssiInst(&rssi);
    radio2.getRssiInst(&rssi2);
    usleep(10);
    // Serial.println("RSSI: " + String(rssi));
    // pass the replies
#endif
}

// Function to process commands
void processCommands(const String &command)
{
    if (isSetCommand(command))
    {
        // Global freq range
        startFreq = 0, endFreq = 0;
        if (parseSetCommand(command, startFreq, endFreq))
        {
            Serial.print("Parsed successfully: Start = ");
            Serial.print(startFreq);
            Serial.print(", End = ");
            Serial.println(endFreq);
        }
        else
        {
            Serial.println("Invalid SET command format!");
        }
    }
    else if (isGetCommand(command))
    {
        int freq;
        if (parseGetCommand(command, freq))
        {
            Serial.print("Parsed successfully: Freq = ");
            Serial.println(freq);
            getFreq = freq;
        }
        else
        {
            Serial.println("Invalid GET command format!");
        }
    }
    else
    {
        Serial.println("Unknown command received!");
    }
}

// Function to parse the "SET" command
// Example: SET 915000-920000
// Example: SET 915000-916000
// Example: SET 900000-1000000
// Example: SET 900000-960000
// Example: SET 2000000-2500000
// Example: SET 2410000-2460000
// Example: SET 2448000-2452000
// Example: SET 0-0 //off
bool parseSetCommand(const String &command, int &startFreq, int &endFreq)
{
    int dashIndex = command.indexOf('-');
    int spaceIndex = command.indexOf(' ');

    if (spaceIndex == -1 || dashIndex == -1 || dashIndex < spaceIndex)
        return false;

    // Extract the two parts
    String startStr = command.substring(spaceIndex + 1, dashIndex);
    String endStr = command.substring(dashIndex + 1);

    // Convert to integers
    startFreq = startStr.toInt();
    endFreq = endStr.toInt();

    return true;
}

// Function to parse the "GET" command with a large frequency value
// GET 915000
// GET 2450000
bool parseGetCommand(const String &command, int &freq)
{
    int spaceIndex = command.indexOf(' ');

    if (spaceIndex == -1) // Ensure there's a space in the command
        return false;

    // Extract the frequency part
    String freqStr = command.substring(spaceIndex + 1);

    // Convert to a long integer
    freq = freqStr.toInt();

    // Validate the conversion
    if (freq <= 0) // Invalid conversion or value
        return false;

    return true;
}

String uartGetResponse()
{
    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 1000)
    { // Wait for up to 1 second
        while (Serial1.available())
        {                            // Check if data is available
            char c = Serial1.read(); // Read one character
            if (c == '\n')
            { // Check for end of response (newline)
                return response;
            }
            response += c; // Append character to response
        }
    }
    return ""; // Return empty string if no response
}

String printRssiRange(std::vector<RSSI> rssis)
{
    String result = "SCAN_RESULT ";
    for (const auto &rssi : rssis)
    {
        result += String((int)rssi.freq) + "," + String(rssi.rssi1) + "," +
                  String(rssi.rssi2) + ";";
    }
    if (result == "SCAN_RESULT ")
    {
        result = "";
    }
    return result;
}

#define LR1121_RFSW_CTRL_COUNT 8;

void SetDioAsRfSwitch()
{
    // 4.2.1 SetDioAsRfSwitch
    uint8_t switchbuf[8];
    /*if (LR1121_RFSW_CTRL_COUNT == 8)
    {
        switchbuf[0] = LR1121_RFSW_CTRL[0]; // RfswEnable
        switchbuf[1] = LR1121_RFSW_CTRL[1]; // RfSwStbyCfg
        switchbuf[2] = LR1121_RFSW_CTRL[2]; // RfSwRxCfg
        switchbuf[3] = LR1121_RFSW_CTRL[3]; // RfSwTxCfg
        switchbuf[4] = LR1121_RFSW_CTRL[4]; // RfSwTxHPCfg
        switchbuf[5] = LR1121_RFSW_CTRL[5]; // RfSwTxHfCfg
        switchbuf[6] = LR1121_RFSW_CTRL[6]; // Unused
        switchbuf[7] =
            LR1121_RFSW_CTRL[7]; // RfSwWifiCfg - Each bit indicates the state of the
                                 // relevant RFSW DIO when in Wi-Fi scanning mode or high
                                 // frequency RX mode (LR1110_H1_UM_V1-7-1.pdf)
    }
    else*/
    {
        switchbuf[0] = 0b00001111; // RfswEnable
        switchbuf[1] = 0b00000000; // RfSwStbyCfg
        switchbuf[2] = 0b00000100; // RfSwRxCfg
        switchbuf[3] = 0b00001000; // RfSwTxCfg
        switchbuf[4] = 0b00001000; // RfSwTxHPCfg
        switchbuf[5] = 0b00000010; // RfSwTxHfCfg
        switchbuf[6] = 0;          // Unused
        switchbuf[7] = 0b00000001; // RfSwWifiCfg - Each bit indicates the state of the
                                   // relevant RFSW DIO when in Wi-Fi scanning mode or
                                   // high frequency RX mode (LR1110_H1_UM_V1-7-1.pdf)
    }
    radio.SPIcommand(LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC, true, switchbuf,
                     sizeof(switchbuf));
    radio2.SPIcommand(LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC, true, switchbuf,
                      sizeof(switchbuf));

    // 4.1.1 SetDioIrqParams

    uint8_t buf[8] = {0};
    buf[3] = LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE;
    radio.SPIcommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, true, buf, sizeof(buf));
    radio2.SPIcommand(LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC, true, buf, sizeof(buf));

    if (RADIO_DCDC)
    {
        // 5.1.1 SetRegMode
        uint8_t RegMode[1] = {1}; // Enable DCDC converter instead of LDO
        radio.SPIcommand(LR11XX_SYSTEM_SET_REGMODE_OC, true, RegMode, sizeof(RegMode));
        radio2.SPIcommand(LR11XX_SYSTEM_SET_REGMODE_OC, true, RegMode, sizeof(RegMode));
    }
}

void initRadio(float freq)
{
    int state, state2;
    radio.begin();
    state = radio.beginGFSK(freq, 4.8F, 5.0F, 156.2F, 10, 16U, 1.6F);
    state2 = radio2.beginGFSK(freq, 4.8F, 5.0F, 156.2F, 10, 16U, 1.6F);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:beginGFSK" + String(state));
    }
    if (state2 != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error2:beginGFSK" + String(state2));
    }
    // RF Switch info Provided by LilyGo support:
    // https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/f2d3d995cba03c65a7031c73e212f106b03c95a2/examples/RadioLibExamples/Receive_Interrupt/Receive_Interrupt.ino#L279

    // LR1121
    // set RF switch configuration for Wio WM1110
    // Wio WM1110 uses DIO5 and DIO6 for RF switching
    static const uint32_t rfswitch_dio_pins[] = {RADIOLIB_LR11X0_DIO5,
                                                 RADIOLIB_LR11X0_DIO6, RADIOLIB_NC,
                                                 RADIOLIB_NC, RADIOLIB_NC};

    static const Module::RfSwitchMode_t rfswitch_table[] = {
        // mode              DIO5  DIO6
        {LR11x0::MODE_STBY, {LOW, LOW}},  {LR11x0::MODE_RX, {HIGH, LOW}},
        {LR11x0::MODE_TX, {LOW, HIGH}},   {LR11x0::MODE_TX_HP, {LOW, HIGH}},
        {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
        {LR11x0::MODE_WIFI, {LOW, LOW}},  END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
    radio2.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // SetDioAsRfSwitch();
    //    LR1121 TCXO Voltage 2.85~3.15V
    radio.setTCXO(3.0);
    radio2.setTCXO(3.0);
    state = radio.setDataShaping(RADIOLIB_SHAPING_NONE);
    state = radio.startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_NONE);
    state2 = radio2.setDataShaping(RADIOLIB_SHAPING_NONE);
    state2 = radio2.startReceive(RADIOLIB_LR11X0_RX_TIMEOUT_NONE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error:startReceive:" + String(state));
    }
    if (state2 != RADIOLIB_ERR_NONE)
    {
        Serial.println("Error2:startReceive:" + String(state2));
    }
}

String wrapResult(std::vector<RSSI> rssis, bool single)
{
    String wrapped = "SCAN_RESULT " + String(rssis.size()) + " [ ";

    for (const auto &rssi : rssis)
    {
        //(<частота кГц>, <сила сигналу, dBm>), (<частота кГц>, <сила сигналу, dBm>)
        wrapped += " (" + String((int)rssi.freq) + ", " + String(rssi.rssi1);
        if (!single)
        {
            wrapped += ", " + String(rssi.rssi2);
        }

        wrapped += "), ";
    }
    // Remove the last comma, if it exists
    if (wrapped.length() > 0)
    {
        wrapped.remove(wrapped.length() - 1);
        wrapped.remove(wrapped.length() - 1);
    }
    wrapped += "]";
    return wrapped;
}

String stringToMinLength(String input, int length)
{
    // Determine absolute length for truncation
    int absLength = abs(length);

    if (input.length() == absLength)
    {
        return input; // Exact length, no change needed
    }
    else if (input.length() > absLength)
    {
        return input.substring(0, absLength); // Truncate the string to the desired length
    }
    else
    {
        int paddingLength = absLength - input.length();
        String padding(paddingLength, ' '); // Create padding of spaces

        if (length > 0)
        {
            return padding + input; // Left padding
        }
        else
        {
            return input + padding; // Right padding
        }
    }
}
