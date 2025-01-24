#define SERIAL_RX 3
#define SERIAL_TX 1

#define RADIO_MISO 33
#define RADIO_MOSI 32
#define RADIO_SCK 25

#define RADIO_BUSY 36
#define RADIO_DIO1 37
#define RADIO_NSS 27
#define RADIO_RST 26

#define RADIO_BUSY_2 39
#define RADIO_DIO1_2 34
#define RADIO_NSS_2 13
#define RADIO_RST_2 21

#define BUTTON 0
#define LED RGB 22
#define LR11XX_SYSTEM_SET_DIO_AS_RF_SWITCH_OC 0x0112
#define LR11XX_SYSTEM_SET_DIOIRQPARAMS_OC 0x0113
#define LR11XX_SYSTEM_SET_REGMODE_OC 0x0110
#define SX12XX_Radio_NONE 0b00000000 // Bit mask for no radio
#define SX12XX_Radio_1 0b00000001    // Bit mask for radio 1
#define SX12XX_Radio_2 0b00000010    // Bit mask for radio 2
#define SX12XX_Radio_All 0b00000011  // bit mask for both radios

#define LR1121_IRQ_TX_DONE 0x00000004
#define LR1121_IRQ_RX_DONE 0x00000008

#define RADIO_DCDC true

    const int LR1121_RFSW_CTRL[] = {31, 0, 4, 8, 8, 18, 0, 17};
const int LR1121_RFSW_CTRL_COUNT = sizeof(LR1121_RFSW_CTRL) / sizeof(LR1121_RFSW_CTRL);
// Source:
// https://github.com/ExpressLRS/targets/blob/4f3b2cba00c706da6a75f545a0650d0b0b0386ee/RX/Generic%20LR1121%20True%20Diversity.json

/**
 * "radio_miso": 33,
    "radio_mosi": 32,
    "radio_sck": 25,

    "radio_busy": 36,
    "radio_dio1": 37,
    "radio_nss": 27,
    "radio_rst": 26,

    "radio_busy_2": 39,
    "radio_dio1_2": 34,
    "radio_nss_2": 13,
    "radio_rst_2": 21,

    "radio_dcdc": true,
 */
