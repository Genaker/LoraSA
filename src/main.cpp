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
#define FREQ_BEGIN 850
// TODO: if % RANGE_PER_PAGE  1= 0
#define FREQ_END 950

// MHZ per page
// to put everething into one page set RANGE_PER_PAGE = FREQ_END - 800
unsigned int RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN

// TODO: Ignore power lines
#define UP_FILTER = 5;
#define LOW_FILTER = 3;

// numbers of the spectrum screen lines = width of screen
// resolution of the scan limited by 128 pixel screan
#define STEPS 128
// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
#define SAMPLES 200 //(scan time = 1294)

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / STEPS)

unsigned int single_step = SINGLE_STEP;

unsigned int range = (int)(FREQ_END - FREQ_BEGIN);
unsigned int fr_begin = FREQ_BEGIN;
unsigned int fr_end = FREQ_BEGIN;

unsigned int iterations = RANGE / RANGE_PER_PAGE;

unsigned int range_freqancy = FREQ_END - FREQ_BEGIN;
unsigned int median_freqancy = FREQ_BEGIN + range_freqancy / 2;

// Measurement bandwidth. Allowed bandwidth values (in kHz) are:
// 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
// 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0
#define BANDWIDTH 93.8 // 467.0

// (optional) major and minor tickmarks at x MHz
#define MAJOR_TICKS 10
#define MINOR_TICKS 5

// Turns the 'PRG' button into the power button, long press is off
#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include <Arduino.h>
#include <heltec_unofficial.h>
#include <images.h>

// This file contaiminins binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

// Prints the scan measurement bins from the SX1262 in hex
// #define PRINT_SCAN_VALUES
#define PRINT_PROFILE_TIME
// Change spectrum plot values at once or by line
#define ANIMATED_RELOAD true

#define MAJOR_TICK_LENGTH 2
#define MINOR_TICK_LENGTH 1
// WEIGHT of the x-asix line
#define X_AXIS_WEIGHT 1
// Height of the ploter area
#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE
//
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH)
#define STATUS_TEXT_TOP (64 - 10)

// Detection level from the 33 levels. Higher number is more sensative
#define DEFAULT_DRONE_DETECTION_LEVEL 21

#define BUZZZER_PIN 41
// REB trigger PIN
#define REB_PIN 42

#define WATERFALL_START 37

#define DISABLE_PLOT_CHART false

// Array to store the scan results
uint16_t result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];
uint16_t filtered_result[RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE];

// Waterfall array
bool waterfall[10][STEPS][10];
// global variable
unsigned short int scan_var = 0;
// initialized flag
bool initialized = false;
// Used as a Led Light and Buzzer/sount trigger
bool led_flag = false;
bool first_run = false;
// drone tetection flag
bool drone_detected = false;
bool detected = false;
unsigned int drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL;
unsigned int drone_detected_freqancy_start = 0;
unsigned int drone_detected_freqancy_end = 0;
unsigned int detection_count = 0;
bool single_page_scan = false;

unsigned int start_scan_text = (128 / 2) - 3;

unsigned int scan_time = 0;
unsigned int scan_start_time = 0;

uint64_t start = 0;

unsigned int x, y, i, w = 0;

float freq = 0;

void clearStatus()
{
  // clear status line
  display.setColor(BLACK);
  display.fillRect(0, STATUS_TEXT_TOP + 2, 128, 13);
  display.setColor(WHITE);
}

void clearPloter()
{
  // clear the scan plot rectangle
  display.setColor(BLACK);
  display.fillRect(0, 0, STEPS, HEIGHT);
  display.setColor(WHITE);
}

/**
 * @brief Draws ticks on the display at regular whole intervals.
 *
 * @param every The interval between ticks in MHz.
 * @param length The length of each tick in pixels.
 */
void drawTicks(float every, int length)
{
  int first_tick = 0;
  //+ (every - (fr_begin - (int)(fr_begin / every) * every));
  /*if (first_tick < fr_begin)
  {
    first_tick += every;
  }*/
  bool correction = false;
  int pixels_per_step = STEPS / (RANGE_PER_PAGE / every);
  if (STEPS / RANGE_PER_PAGE != 0)
  {
    correction = true;
  }
  int correction_number = STEPS - (int)(pixels_per_step * (RANGE_PER_PAGE / every));
  int tick = 0;
  int tick_minor = 0;
  int median = (RANGE_PER_PAGE / every) / 2;
  // TODO: (RANGE_PER_PAGE / every) * 2 has twice extra steps we need to figureout correct logic or minor ticks is not showing to the end
  for (int t = 0; t <= (RANGE_PER_PAGE / every) * 2; t++)
  {
    // fix if pixels per step is not int and we have shift
    if (correction && t % 2 != 0 && correction_number > 1)
    {
      // pixels_per_step++;
      correction_number--;
    }

    tick += pixels_per_step;
    tick_minor = tick / 2;

    if (tick < 128 - 3)
    {
      display.drawLine(tick, HEIGHT + X_AXIS_WEIGHT, tick, HEIGHT + X_AXIS_WEIGHT + length);
      // Central tick
      if (tick > (128 / 2) - 3 && tick < (128 / 2) + 3)
      {
        display.drawLine(tick + 1, HEIGHT + X_AXIS_WEIGHT, tick + 1, HEIGHT + X_AXIS_WEIGHT + length);
      }
    }

#ifdef MINOR_TICKS
    // Fix two ticks togather
    if (tick_minor + 1 != tick && tick_minor - 1 != tick && tick_minor + 2 != tick && tick_minor - 2 != tick)
    {
      display.drawLine(tick_minor, HEIGHT + X_AXIS_WEIGHT, tick_minor, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
    }
    // Central tick
    if (tick_minor > (128 / 2) - 3 && tick_minor < (128 / 2) + 3)
    {
      display.drawLine(tick_minor + 1, HEIGHT + X_AXIS_WEIGHT, tick_minor + 1, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
    }
//}
#endif
  }

  // Draw Central Tick
  // display.drawLine(128/2, HEIGHT + X_AXIS_WEIGHT, 128/2, HEIGHT + X_AXIS_WEIGHT + 5);
}

/**
 * @brief Decorates the display: everything but the plot itself.
 */
void displayDecorate(int begin = 0, int end = 0, bool redraw = false)
{
  if (!initialized)
  {
    // begining and end ticks
    display.fillRect(0, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH + 1);
    display.fillRect(126, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH + 1);

    // drone detection level
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(drone_detection_level));
  }

  if (!initialized || redraw)
  {
    // Clear something
    display.setColor(BLACK);
    display.fillRect(0, SCALE_TEXT_TOP + 1, 128, 12);
    display.setColor(WHITE);

    // drone detection level
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(drone_detection_level));

    // frequency start
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, SCALE_TEXT_TOP, (begin == 0) ? String(FREQ_BEGIN) : String(begin));

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(128 / 2, SCALE_TEXT_TOP, (begin == 0) ? String(median_freqancy) : String(begin + ((end - begin) / 2)));

    // frequency end
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, SCALE_TEXT_TOP, (end == 0) ? String(FREQ_END) : String(end));
  }

  if (led_flag == true && detection_count > 5)
  {
    digitalWrite(LED, HIGH);
    tone(BUZZZER_PIN, 104, 250);
    digitalWrite(REB_PIN, HIGH);
    led_flag = false;
  }

  else if (!redraw)
  {
    digitalWrite(LED, LOW);
  }
  // Status text block
  if (!drone_detected)
  {
    // "Scanning"
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    // clear status line
    clearStatus();
    if (scan_var == 0)
    {
      display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scan.  ");
    }
    else if (scan_var == 1)
    {
      display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scan.. ");
    }
    else if (scan_var == 2)
    {
      display.drawString(start_scan_text, STATUS_TEXT_TOP, "Scan...");
    }
    scan_var++;
    if (scan_var == 3)
    {
      scan_var = 0;
    }
  }

  if (drone_detected)
  {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    // clear status line
    clearStatus();

    display.drawString(start_scan_text, STATUS_TEXT_TOP, String(drone_detected_freqancy_start) + ">RF<" + String(drone_detected_freqancy_end));
  }

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, STATUS_TEXT_TOP, String(FREQ_BEGIN));

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, STATUS_TEXT_TOP, String(FREQ_END));

  if (!initialized)
  {
    // X-axis
    display.fillRect(0, HEIGHT, STEPS, X_AXIS_WEIGHT);
// ticks
#ifdef MAJOR_TICKS
    drawTicks(MAJOR_TICKS, MAJOR_TICK_LENGTH);
#endif
  }
  initialized = true;
}

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(BUZZZER_PIN, OUTPUT);
  pinMode(REB_PIN, OUTPUT);
  heltec_setup();
  display.clear();
  // draw the UCOG welcome logo
  display.drawXbm(0, 2, 128, 64, epd_bitmap_ucog);
  display.display();

  delay(2000);

  // initialize SX1262 FSK modem at the initial frequency
  both.println("Init radio");
  RADIOLIB_OR_HALT(radio.beginFSK(FREQ_BEGIN));
  // upload a patch to the SX1262 to enable spectral scan
  // NOTE: this patch is uploaded into volatile memory,
  // and must be re-uploaded on every power up
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
  both.printf("V battery: %.2fV (%d%%)\n", vbat, heltec_battery_percent(vbat));
  delay(300);

  display.clear();

  float resolution = RANGE / STEPS;
  if (RANGE_PER_PAGE == range)
  {
    single_page_scan = true;
  }
  else
  {
    single_page_scan = false;
  }

  // Adjust range if it is not even to RANGE_PER_PAGE
  if (!single_page_scan && range % RANGE_PER_PAGE != 0)
  {
    // range = range + range % RANGE_PER_PAGE;
  }

  if (single_page_scan)
  {
    both.println("Single Page Screen MODE");
    both.println("Multi Screen View Press P - button");
    both.println("Single Screen Resolution: " + String(resolution) + "Mhz/tick");
    both.println("Curent Resolution: " + String((float)RANGE_PER_PAGE / STEPS) + "Mhz/tick");

    for (int i = 0; i < 500; i++)
    {
      button.update();
      delay(10);
      both.print(".");
      if (button.pressed())
      {
        RANGE_PER_PAGE = 50;
        single_page_scan = false;
        tone(BUZZZER_PIN, 205, 100);
        delay(50);
        tone(BUZZZER_PIN, 205, 100);
        break;
      }
    }
  }
  else
  {
    both.println("Multi Page Screen MODE");
    both.println("Single screen View Press P - button");
    both.println("Single screen Resolution: " + String(resolution) + "Mhz/tick");
    both.println("Curent Resolution: " + String((float)RANGE_PER_PAGE / STEPS) + "Mhz/tick");

    for (int i = 0; i < 500; i++)
    {
      button.update();
      delay(10);
      both.print(".");
      if (button.pressed())
      {
        RANGE_PER_PAGE = range;
        single_page_scan = true;
        tone(BUZZZER_PIN, 205, 100);

        break;
      }
    }
  }
  display.clear();

  // waterfall start line y-asix
  w = WATERFALL_START;
}

void loop()
{
  displayDecorate();
  drone_detected = false;
  detection_count = 0;
  drone_detected_freqancy_start = 0;
#ifdef PRINT_PROFILE_TIME
  start = millis();
#endif

  if (!ANIMATED_RELOAD || !single_page_scan)
  {
    // clear the scan plot rectangle
    clearPloter();
  }

  // do the scan
  range = FREQ_END - FREQ_BEGIN;

  if (RANGE_PER_PAGE > range)
  {
    RANGE_PER_PAGE = range;
  }

  fr_begin = FREQ_BEGIN;
  fr_end = fr_begin;
  // 50 is a single screen range
  // TODO:Make 50 as a variable with the option show full range
  iterations = range / RANGE_PER_PAGE;

  single_step = RANGE_PER_PAGE / 128;
  if (range % RANGE_PER_PAGE != 0)
  {
    // add more scan
    //++;
  }

  if (RANGE_PER_PAGE == range)
  {
    single_page_scan = true;
  }
  else
  {
    single_page_scan = false;
  }

  // Iterateing by small ranges by 50 Mhz each pixel is 0.4 Mhz
  for (i = 0; i < iterations; i++)
  {
    range = RANGE_PER_PAGE;
    drone_detected_freqancy_start == 0;

    fr_begin = (i == 0) ? fr_begin : fr_begin += range;
    fr_end = fr_begin + RANGE_PER_PAGE;

    if (!ANIMATED_RELOAD || !single_page_scan)
    {
      // clear the scan plot rectangle
      clearPloter();
    }

    if (single_page_scan == false)
    {
      displayDecorate(fr_begin, fr_end, true);
    }

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    // horizontal x assix loop
    for (x = 0; x < STEPS; x++)
    {
      scan_start_time = millis();
      if (ANIMATED_RELOAD)
      {
        // Draw animated cursor on reload process
        display.setColor(BLACK);
        display.drawVerticalLine(x, 0, HEIGHT);
        display.drawVerticalLine(x + 1, 0, HEIGHT);
        display.setColor(WHITE);
      }
      waterfall[i][x][w] = false;
      freq = fr_begin + (range * ((float)x / STEPS));
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
      while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE)
      {
        heltec_delay(1);
      }
      // read the results Array to which the results will be saved
      radio.spectralScanGetResult(result);
      detected = false;
      // Filter Elements without neabors
      for (y = 1; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
      {
        if (result[y] && (result[y + 1] > 0 || result[y - 1] > 0))
        {
          filtered_result[y] = 1;
        }
        else
        {
          filtered_result[y] = 0;
        }
      }

      for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
      {
#ifdef PRINT_SCAN_VALUES
        Serial.printf("%04X,", result[y]);
#endif
        if (result[y] || y == drone_detection_level)
        {
          // check if we shuld alarm the dron
          if (filtered_result[y] == 1 && y <= drone_detection_level)
          {
            drone_detected = true;
            if (single_page_scan)
            {
              // Drone detection true for waterfall
              waterfall[i][x][w] = true;
              display.setColor(WHITE);
              display.setPixel(x, w);
            }

            if (drone_detected_freqancy_start == 0)
            {
              drone_detected_freqancy_start = freq;
            }
            drone_detected_freqancy_end = freq;
            led_flag = true;
            // if level to sensetive doing beep every 10th freaqancy and shorter
            if (drone_detection_level < 25)
            {
              if (detection_count == 1)
                tone(BUZZZER_PIN, 205, 10);
              if (detection_count % 5 == 0)
                tone(BUZZZER_PIN, 205, 10);
            }
            else
            {
              if (detection_count % 20 == 0)
                tone(BUZZZER_PIN, 205, 10);
            }
            display.setPixel(x, 1);
            display.setPixel(x, 2);
            display.setPixel(x, 3);
            display.setPixel(x, 4);
          }
          else if (filtered_result[y] == 1 && y > drone_detection_level && single_page_scan && waterfall[i][x][w] != true)
          {
            // if drone not found dark pixel on waterfall
            // TODO: make somethin like scrolling up if possible
            waterfall[i][x][w] = false;
            display.setColor(BLACK);
            display.setPixel(x, w);
            display.setColor(WHITE);
          }

          if (filtered_result[y] == 1)
          {
            // Set signal level pixel
            display.setPixel(x, y);
            detected = true;
          }

          // Draw detection Level line
          if (y == drone_detection_level && x % 2 == 0)
          {
            display.setPixel(x, y);
          }
        }
        if (detected)
        {
          detection_count++;
        }
#ifdef PRINT_PROFILE_TIME
        scan_time = millis() - scan_start_time;
        // Huge performance issue if enable
        // Serial.printf("Single Scan took %lld ms\n", scan_time);
#endif
      }
      detected = false;
#ifdef PRINT_SCAN_VALUES
      Serial.println();
#endif
      if (first_run || ANIMATED_RELOAD)
      {
        display.display();
      }

      // Detection level button short press
      if (button.pressedFor(100))
      {
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        // erase old value
        display.setColor(BLACK);
        display.fillRect(128 - 13, 0, 13, 13);
        display.setColor(WHITE);
        drone_detection_level++;
        // print new value
        display.drawString(128, 0, String(drone_detection_level));
        tone(BUZZZER_PIN, 104, 150);
        if (drone_detection_level > 30)
        {
          drone_detection_level = 1;
        }
      }
      // wait a little bit before the next scan, otherwise the SX1262 hangs
      heltec_delay(1);
    }
    w++;
    if (w > STATUS_TEXT_TOP + 1)
    {
      w = WATERFALL_START;
    }

    // Draw waterfall position cursor
    if (single_page_scan)
    {
      display.setColor(BLACK);
      display.drawHorizontalLine(0, w, STEPS);
      display.setColor(WHITE);
    }
    display.display();
  }

#ifdef PRINT_SCAN_VALUES
  Serial.println();
#endif
  // display.display();
#ifdef PRINT_PROFILE_TIME
  scan_time = millis() - start;
  Serial.printf("Scan took %lld ms\n", scan_time);
#endif
}
