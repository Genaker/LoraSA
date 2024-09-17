/**
 * Send and receive LoRa-modulation packets with a sequence number, showing RSSI
 * and SNR for received packets on the little display.
 *
 * Note that while this send and received using LoRa modulation, it does not do
 * LoRaWAN. For that, see the LoRaWAN_TTN example.
 *
 * This works on the stick, but the output on the screen gets cut off.
 */

#include <Arduino.h>
// Turns the 'PRG' button into the power button, long press is off
#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include <heltec_unofficial.h>

// Pause between transmited packets in mseconds.
// Set to zero to only transmit a packet when pressing the user button
// Will not exceed 1% duty cycle, even if you set a lower value.
#define PAUSE 10

// Frequency in MHz. Keep the decimal point to designate float.
// Check your own rules and regulations to see what is legal where you are.
#define FREQUENCY 915 // for Europe
// #define FREQUENCY           905.2       // for US

// LoRa bandwidth. Keep the decimal point to designate float.
// Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0 and 500.0
// kHz.
#define BANDWIDTH 250.0

// Number from 5 to 12. Higher means slower but higher "processor gain",
// meaning (in nutshell) longer range and more robust against interference.
#define SPREADING_FACTOR 7

// Transmit power in dBm. 0 dBm = 1 mW, enough for tabletop-testing. This value can be
// set anywhere between -9 dBm (0.125 mW) to 22 dBm (158 mW). Note that the maximum ERP
// (which is what your antenna maximally radiates) on the EU ISM band is 25 mW, and that
// transmissting without an antenna can damage your hardware.
#define TRANSMIT_POWER 22

String rxdata;
volatile bool rxFlag = false;
long counter = 0;
uint64_t last_tx = 0;
uint64_t tx_time;
uint64_t minimum_pause;

// Can't do Serial or display things here, takes too much time for the interrupt
void rx() { rxFlag = true; }

void setup()
{
    heltec_setup();
    both.println("Radio init");
    RADIOLIB_OR_HALT(radio.begin());
    // Set the callback function for received packets
    radio.setDio1Action(rx);
    // Set radio parameters
    both.printf("Frequency: %.2f MHz\n", FREQUENCY);
    RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
    both.printf("Bandwidth: %.1f kHz\n", BANDWIDTH);
    RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
    both.printf("Spreading Factor: %i\n", SPREADING_FACTOR);
    RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
    both.printf("TX power: %i dBm\n", TRANSMIT_POWER);
    RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));
    // Start receiving
    RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
}

int FHSS = 0.25;
float FHSS_counter = -10;
void loop()
{
    heltec_loop();

    bool tx_legal = true; // millis() > last_tx + minimum_pause;
    // Emulate  frequency hopping spread spectrum (FHSS) is a method of transmitting radio
    // signals by rapidly switching the carrier between different frequency channels.
    float fr = (float)(FREQUENCY + (float)(FHSS_counter));
    RADIOLIB_OR_HALT(radio.setFrequency(fr, false));
    // Transmit a packet every PAUSE seconds or when the button is pressed
    if ((PAUSE && tx_legal && millis() - last_tx > (PAUSE)) || button.isSingleClick())
    {
        if (button.isSingleClick())
        {
            fr = 1000;
            RADIOLIB_OR_HALT(radio.setFrequency(fr, false));
        }
        // In case of button click, tell user to wait
        display.printf("TX[%s]", String(counter).c_str());
        radio.clearDio1Action();
        heltec_led(50); // 50% brightness is plenty for this LED
        tx_time = millis();
        RADIOLIB(radio.transmit(String("Putin Huylo!!! LA-LA-LA-LA").c_str()));
        tx_time = millis() - tx_time;
        heltec_led(0);
        if (_radiolib_status == RADIOLIB_ERR_NONE)
        {
            display.printf("OK(%ims)/%.2fMhz\n", (int)tx_time, fr);
        }
        else
        {
            display.printf("fail (%i)\n", _radiolib_status);
            heltec_delay(100);
        }
        // Maximum 1% duty cycle
        minimum_pause = tx_time * 100;
        last_tx = millis();
        radio.setDio1Action(rx);
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    }

    // If a packet was received, display it and the RSSI and SNR
    if (rxFlag)
    {
        rxFlag = false;
        radio.readData(rxdata);
        if (_radiolib_status == RADIOLIB_ERR_NONE)
        {
            display.printf("RX [%s]\n", rxdata.c_str());
            display.printf("  RSSI: %.2f dBm\n", radio.getRSSI());
            display.printf("  SNR: %.2f dB\n", radio.getSNR());
        }
        RADIOLIB_OR_HALT(radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF));
    }
    FHSS_counter += 0.250;
    counter++;
    if (FHSS_counter > 20)
    {
        FHSS_counter = -10;
    }
}
