#include <Arduino.h>

// Include MAVLink headers.
// Adjust the include path to match your local mavlink library structure.
#include "mavlink.h"

// Which UART on ESP32? For example, Serial2 can be mapped to pins:
constexpr int RX_PIN = 16;           // Flight Controller TX -> ESP32 RX
constexpr int TX_PIN = 17;           // Flight Controller RX <- ESP32 TX
constexpr long MAVLINK_BAUD = 57600; // Common default for MAVLink

// Identify our local system/component, and the target FC system
// (ArduPilot often uses sysid=1, compid=1, but it can vary).
constexpr uint8_t MAV_COMP_ID_ONBOARD = 1;  // or 190 for companion computer
constexpr uint8_t MAV_SYS_ID_ONBOARD = 255; // e.g. 255 for companion
constexpr uint8_t TARGET_SYS_ID = 1;        // autopilot system ID
constexpr uint8_t TARGET_COMP_ID = 1;       // autopilot component ID

// We'll send 8 channels. MAVLink 2 can handle up to 18 channels in
// set_rc_channels_override, but typical autopilots read at least 8 or 14 channels.
static uint16_t rcChannels[8] = {
    1500, // Channel 1
    1500, // Channel 2
    1500, // Channel 3
    1500, // Channel 4
    1000, // Channel 5
    1000, // Channel 6
    1000, // Channel 7
    1000  // Channel 8
};

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("MAVLink RC Override Example - ESP32");

    // Initialize Serial2 on given pins
    Serial2.begin(MAVLINK_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

    // (Optional) Wait a bit for the autopilot to boot, if needed
    delay(2000);
}

void loop()
{
    // For demonstration, let's do a small "animation" on channel 1
    // to show that the flight controller is indeed receiving changes.
    // We'll move channel 1 from 1000 to 2000 in steps.
    static int ch1Value = 1000;
    static int increment = 10;

    ch1Value += increment;
    if (ch1Value > 2000)
    {
        ch1Value = 2000;
        increment = -10;
    }
    else if (ch1Value < 1000)
    {
        ch1Value = 1000;
        increment = 10;
    }

    // Update channel 1 in our array
    rcChannels[0] = ch1Value;

    // Send the MAVLink Set RC Channels Override message
    sendRCChannelsOverride();

    // Send ~20 times per second
    delay(50);
}

void sendRCChannelsOverride()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Fill out the message:
    // mavlink_msg_set_rc_channels_override_pack(
    //   uint8_t system_id,
    //   uint8_t component_id,
    //   mavlink_message_t* msg,
    //   uint8_t target_system,
    //   uint8_t target_component,
    //   uint16_t chan1_raw,
    //   uint16_t chan2_raw,
    //   ...
    //   uint16_t chan8_raw,
    //   uint16_t chan9_raw,
    //   ...
    //   up to chan18_raw
    // )
    // For 8 channels, pass 0 for unused channels > 8.

    mavlink_msg_set_rc_channels_override_pack(
        MAV_SYS_ID_ONBOARD, MAV_COMP_ID_ONBOARD, &msg, TARGET_SYS_ID, TARGET_COMP_ID,
        rcChannels[0],         // chan1
        rcChannels[1],         // chan2
        rcChannels[2],         // chan3
        rcChannels[3],         // chan4
        rcChannels[4],         // chan5
        rcChannels[5],         // chan6
        rcChannels[6],         // chan7
        rcChannels[7],         // chan8
        0, 0, 0, 0, 0, 0, 0, 0 // for chan9..chan16 or up to chan18
    );

    // Encode the message into the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Write it out the serial port to the FC
    Serial2.write(buf, len);

    // For debugging
    Serial.print("Sent RC override: [");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(rcChannels[i]);
        if (i < 7)
            Serial.print(", ");
    }
    Serial.println("]");
}
