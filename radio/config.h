#pragma once

#define RUN_TESTS 0

#define TXD1 39
#define RXD2 40
// SBUS packet structure
#define SBUS_PACKET_SIZE 25

#ifndef LORA_SF
// Sets LoRa spreading factor. Allowed values range from 5 to 12.
#define LORA_SF 8 // For retranmission 6.
#endif

#ifndef LORA_CR
#define LORA_CR 8
#endif

#ifndef DEBUG_RX
#define DEBUG_RX 0
#endif

#ifndef LORA_HEADER
#define LORA_HEADER 0
#endif

#ifndef LORA_FHSS
#define LORA_FHSS 0
#endif

#ifndef LORA_RX
#define LORA_RX 0
#endif

#ifndef LORA_TX
#define LORA_TX 1
#endif

#ifndef LORA_BASE_FREQ
// Sets LoRa coding rate denominator. Allowed values range from 5 to 8.
#define LORA_BASE_FREQ 915
#endif

#ifndef LORA_BW
// Sets LoRa bandwidth. Allowed values are 62.5, 125.0, 250.0 and 500.0 kHz. (default,
// high = false)
#define LORA_BW 62.5 // 31.25 // 125.0 // 62.5 // 31.25 // 500
#endif

#ifndef LORA_DATA_BYTE
#define LORA_DATA_BYTE 2
#endif

#define SBUS 1
#define IBUS 2
#define CRSF 3 // Doesn't work

#ifndef PROTOCOL
#define PROTOCOL CRSF // CRSF // IBUS // SBUS
#endif

#ifndef SERIAL_INPUT
#define SERIAL_INPUT 0
#endif

#ifndef LORA_PREAMBLE
// 8 is default
#if LORA_SF == 6 || LORA_SF == 5
#define LORA_PREAMBLE 8
#elif LORA_SF == 7
#define LORA_PREAMBLE 8
#else
#define LORA_PREAMBLE 10
#endif
#endif
