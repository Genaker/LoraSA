#include <Arduino.h>
#include <ELRS_GENERIC_LR1221_DIVERSITY.h>
#include <HardwareSerial.h>

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

// SPIClass hspi = new SPIClass(2);

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
SPIExClass SPIEx(FSPI);
#elif defined(PLATFORM_ESP32)
SPIExClass SPIEx(VSPI);
#else
// SPIExClass SPIEx;
#endif //

bool isSetCommand(const String &command);
void processCommands(const String &command);
bool parseSetCommand(const String &command, int &startFreq, int &endFreq);

void setup()
// https://github.com/ExpressLRS/ExpressLRS/blob/2e61e33723b3247300f95ed46ceee9648d536137/src/lib/LR1121Driver/LR1121_hal.cpp#L6
{
    // pinMode(0, INPUT_PULLUP); // Trying to enable auto boot mode. Notes it makes Boot
    // mode always.
    // TODO: make on button press without reset. or with reset :0
    pinMode(RADIO_BUSY, INPUT);
    pinMode(RADIO_DIO1, INPUT);
    pinMode(RADIO_NSS, OUTPUT);
    digitalWrite(RADIO_NSS, HIGH);

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
    int state, state2;
    float freq = 900;
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
        // mode                  DIO5  DIO6
        {LR11x0::MODE_STBY, {LOW, LOW}},  {LR11x0::MODE_RX, {HIGH, LOW}},
        {LR11x0::MODE_TX, {LOW, HIGH}},   {LR11x0::MODE_TX_HP, {LOW, HIGH}},
        {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
        {LR11x0::MODE_WIFI, {LOW, LOW}},  END_OF_MODE_TABLE,
    };
    radio.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);
    radio2.setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

    // LR1121 TCXO Voltage 2.85~3.15V
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

    delay(1000);
}

void loop()
{
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n'); // Read until newline
        command.trim(); // Remove any leading/trailing whitespace

        // Process the command
        processCommands(command);
    }
    int state, state2;
    // Print message periodically
    Serial.println("Still running...");
    state = radio.setFrequency((float)900);
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
    float rssi, rssi2;
    radio.getRssiInst(&rssi);
    radio2.getRssiInst(&rssi2);
    // Serial.println("RSSI: " + String(rssi));
    // pass the replies
#endif
    Serial.println("[900Mhz]RSSI: " + String((float)rssi) +
                   " RSSI2: " + String((float)rssi2));
    delay(1000);
}

// Function to process commands
void processCommands(const String &command)
{
    if (isSetCommand(command))
    {
        int startFreq, endFreq;
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
    else
    {
        Serial.println("Unknown command received!");
    }
}

// Function to check if the command starts with "SET"
bool isSetCommand(const String &command) { return command.startsWith("SET "); }

// Function to parse the "SET" command
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

    // Validate the conversion
    if (startFreq == 0 || endFreq == 0)
        return false;

    return true;
}
