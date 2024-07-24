 /**
   RadioLib SX126x Spectrum Scan 

   This code perform a spectrum power scan using SX126x.
   The output is in the form of scan lines, each line has 33 power bins.
   First power bin corresponds to -11 dBm, the second to -15 dBm and so on.
   Higher number of samples in a bin corresponds to more power received
   at that level.

   To show the results in a plot, run the Python script
   RadioLib/extras/SX126x_Spectrum_Scan/SpectrumScan.py

   WARNING: This functionality is experimental and requires a binary patch
   to be uploaded to the SX126x device. There may be some undocumented
   side effects!

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx126x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// frequency range in MHz to scan
#define FREQ_BEGIN 750
#define FREQ_END 900

unsigned int range_freqancy = FREQ_END - FREQ_BEGIN;
unsigned int median_freqancy = FREQ_BEGIN + range_freqancy / 2;

// Measurement bandwidth. Allowed bandwidth values (in kHz) are:
// 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
// 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0
#define BANDWIDTH 93.8//467.0

// (optional) major and minor tickmarks at x MHz
#define MAJOR_TICKS 10
//#define MINOR_TICKS 4

// Turns the 'PRG' button into the power button, long press is off 
#define HELTEC_POWER_BUTTON   // must be before "#include <heltec_unofficial.h>"
#include <Arduino.h>
#include <heltec_unofficial.h>
#include <images.h>

// This file contains binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

// Prints the scan measurement bins from the SX1262 in hex
#define PRINT_SCAN_VALUES
#define PRINT_PROFILE_TIME
//Change spectrum plot values at once or by line 
#define ANIMATED_RELOAD true

// numbers of the spectrum screan lines = width of screan 
#define STEPS 128
// Number of samples for each freqancy scan. Fewer samples = better temporal resolution.
#define SAMPLES 50 //(scan time = 1294)
#define MAJOR_TICK_LENGTH 3
#define MINOR_TICK_LENGTH 1
#define X_AXIS_WEIGHT 2
#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE
//
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH)
#define STATUS_TEXT_TOP (64 - 14)
#define RANGE (float)(FREQ_END - FREQ_BEGIN)
#define SINGLE_STEP (float)(RANGE / STEPS)

#define DRONE_DETECTION_LEVEL 20

#define BUZZZER_PIN 41

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
// global variable 
unsigned short int scan_var = 0;
// initialized flag
bool initialized = false;
bool led_flag = false;
bool first_run = false;
// drone tetection flag
bool drone_detected = false;
unsigned int drone_detected_freqancy_start = 0;
unsigned int drone_detected_freqancy_end = 0;

unsigned int start_scan_text = (128 / 2) - 3;

unsigned int scan_time = 0;

uint64_t start = 0;

unsigned int x,y = 0;

float freq = 0;

/**
 * @brief Draws ticks on the display at regular whole intervals.
 * 
 * @param every The interval between ticks in MHz. 
 * @param length The length of each tick in pixels.
 */
void drawTicks(float every, int length) {
  float first_tick  = FREQ_BEGIN + (every - (FREQ_BEGIN - (int)(FREQ_BEGIN / every) * every));
  if (first_tick < FREQ_BEGIN){
    first_tick += every;
  }
  for (float tick_freq = first_tick; tick_freq <= FREQ_END; tick_freq += every) {
    int tick = round((tick_freq - FREQ_BEGIN) / SINGLE_STEP);
    display.drawLine(tick, HEIGHT + X_AXIS_WEIGHT, tick, HEIGHT + X_AXIS_WEIGHT + length);
  }
}

/**
 * @brief Decorates the display: everything but the plot itself.
 */
void displayDecorate() {
  if (!initialized) {
    // begining and end ticks 
    display.fillRect(0, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH);
    display.fillRect(126, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH);
    // frequencies
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, SCALE_TEXT_TOP, String(FREQ_BEGIN));
    //drone detection level
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(DRONE_DETECTION_LEVEL));

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(128/2, SCALE_TEXT_TOP, String(median_freqancy));
    //Draw Central line
    display.drawLine(128/2, HEIGHT + X_AXIS_WEIGHT, 128 / 2 , HEIGHT + X_AXIS_WEIGHT + 4);

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, SCALE_TEXT_TOP, String(FREQ_END));
    }
  
  if(led_flag == true) {
    digitalWrite(LED, HIGH);
    tone(BUZZZER_PIN, 104, 500);
    led_flag = false;
  } else {
    digitalWrite(LED, LOW);
  }
  // Status text block
  if (drone_detected == 0) {
      // "Scanning"
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      //clear status line 
      display.setColor(BLACK);
      display.fillRect(0, STATUS_TEXT_TOP, 128, 16);
      display.setColor(WHITE);
      if (scan_var == 0) {
        display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scanning.  ");
      }
      else if (scan_var == 1) {
        display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scanning.. ");
      }
      else if (scan_var == 2) {
        display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scanning...");
      }
      scan_var++;
      if (scan_var == 3) scan_var = 0;
    } else {
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      //clear status line 
      display.setColor(BLACK);
      display.fillRect(0, STATUS_TEXT_TOP, 128, 16);
      display.setColor(WHITE);
      
      display.drawString(start_scan_text, STATUS_TEXT_TOP, String(drone_detected_freqancy_start) + "->!UAV!<- " + String(drone_detected_freqancy_end));
      drone_detected_freqancy_start = 0;
      drone_detected = 0;
    }

  if (!initialized) {
  // X-axis
  display.fillRect(0, HEIGHT, STEPS, X_AXIS_WEIGHT);
  // ticks
  #ifdef MAJOR_TICKS
    drawTicks(MAJOR_TICKS, MAJOR_TICK_LENGTH);
  #endif
  #ifdef MINOR_TICKS
    drawTicks(MINOR_TICKS, MINOR_TICK_LENGTH);
  #endif
  }
  initialized = true;
}


void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  heltec_setup();
  display.clear();
  // draw the logo
  display.drawXbm(0, 0, 128, 64, epd_bitmap_ucog);
  display.display();
  heltec_delay(4000);
  // initialize SX1262 FSK modem at the initial frequency
  both.println("Init radio");
  RADIOLIB_OR_HALT(radio.beginFSK(FREQ_BEGIN));
  // upload a patch to the SX1262 to enable spectral scan
  // NOTE: this patch is uploaded into volatile memory,
  //       and must be re-uploaded on every power up
  both.println("Upload SX1262 patch");
  // Upload binary patch into the SX126x device RAM. Patch is needed to e.g., enable spectral scan and must be uploaded again on every power cycle.
  RADIOLIB_OR_HALT(radio.uploadPatch(sx126x_patch_scan, sizeof(sx126x_patch_scan)));
  // configure scan bandwidth and disable the data shaping
  both.println("Setting up radio");
  RADIOLIB_OR_HALT(radio.setRxBandwidth(BANDWIDTH));
  // and disable the data shaping
  RADIOLIB_OR_HALT(radio.setDataShaping(RADIOLIB_SHAPING_NONE));
  both.println("Starting scaning...");
  float vbat = heltec_vbat();
  both.printf("V battrry: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
  heltec_delay(100);
 
  display.clear();
  displayDecorate();
}

void loop() {
  displayDecorate();
  #ifdef PRINT_PROFILE_TIME
    start = millis();
  #endif

  if (!ANIMATED_RELOAD) {
  // clear the scan plot rectangle
    display.setColor(BLACK);
    display.fillRect(0, 0, STEPS, HEIGHT);
    display.setColor(WHITE);
  }

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    //drone detection level 
    display.drawString(128, 0, String(DRONE_DETECTION_LEVEL));

  // do the scan
  for (x = 0; x < STEPS; x++) {
    if (ANIMATED_RELOAD) {
        display.setColor(BLACK);
        display.drawVerticalLine(x, 0, HEIGHT);
        display.setColor(WHITE);
    }
    freq = FREQ_BEGIN + (RANGE * ((float) x / STEPS));
    radio.setFrequency(freq);
    #ifdef PRINT_SCAN_VALUES
        Serial.println();
        Serial.print("step-");
        Serial.print(x);
        Serial.print(" Frequancy:");
        Serial.print(freq);
        Serial.println();
    #endif
    // start spectral scan
    radio.spectralScanStart(SAMPLES, 1);
    // wait for spectral scan to finish
    while(radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE) {
      heltec_delay(1);
    }
    // read the results Array to which the results will be saved
    radio.spectralScanGetResult(result);
    //Filter Elements without neabors 
    for (y = 1; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++) {
      if(result[y] && (result[y + 1] > 0 || result[y - 1] > 0 )){
        filtered_result[y] = 1;
      } else {
        filtered_result[y] = 0;
      }
    }

    for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++) {
    #ifdef PRINT_SCAN_VALUES
        Serial.printf("%04X,", result[y]);
    #endif
      if (result[y] || y == DRONE_DETECTION_LEVEL) {
        // check if we shuld alarm the dron
        if (filtered_result[y] == 1 && y <= DRONE_DETECTION_LEVEL) {
          drone_detected = true;
          if(drone_detected_freqancy_start == 0) {
            drone_detected_freqancy_start = freq; 
          }
          drone_detected_freqancy_end = freq;
          led_flag = true;
          tone(BUZZZER_PIN, 205, 10);
          display.setPixel(x, 1);
          display.setPixel(x, 2);
          display.setPixel(x, 3);
        }
          if(filtered_result[y] == 1 || y == DRONE_DETECTION_LEVEL){
            display.setPixel(x, y);
          }
      }
    }
    #ifdef PRINT_SCAN_VALUES
      Serial.println();
    #endif
    if (first_run || ANIMATED_RELOAD){
      display.display();
    }
    // wait a little bit before the next scan, otherwise the SX1262 hangs
    heltec_delay(1);
  }
  #ifdef PRINT_SCAN_VALUES
    Serial.println();
  #endif
  display.display();
  #ifdef PRINT_PROFILE_TIME
    scan_time = millis() - start;
    Serial.printf("Scan took %lld ms\n", scan_time);
  #endif
}

