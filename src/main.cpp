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
// TODO: if % RANGE_PER_PAGE  != 0
#define FREQ_END 950

// true for RSSI and flase for default spectralScan method
#define RSSI_METHOD false

// Feature to scan diapazones. Other frequency settings will be ignored.
int SCAN_RANGES[] = {};
// int SCAN_RANGES[] = {850890, 920950};

// MHZ per page
// to put everething into one page set RANGE_PER_PAGE = FREQ_END - 800
unsigned int RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN
// To Enable Multi Screen scan
//  unsigned int RANGE_PER_PAGE = 50;
//  Default Range on Menu Button Switch
#define DEFAULT_RANGE_PER_PAGE 50

// TODO: Ignore power lines
#define UP_FILTER 5
#define LOW_FILTER 3
#define FILTER_SPECTRUM_RESULTS true

// The number of the spectrum screen lines = width of screen
// Resolution of the scan is limited by 128-pixel screen
#define STEPS 128
// Number of samples for each frequency scan. Fewer samples = better temporal resolution.
// if more than 100 it can freez
#define SAMPLES 100 //(scan time = 1294)
// number of samples for RSSI method
#define SAMPLES_RSSI 21//

#define RANGE (int)(FREQ_END - FREQ_BEGIN)

#define SINGLE_STEP (float)(RANGE / STEPS)

unsigned int single_step = SINGLE_STEP;

unsigned int range = (int)(FREQ_END - FREQ_BEGIN);
unsigned int fr_begin = FREQ_BEGIN;
unsigned int fr_end = FREQ_BEGIN;

unsigned int iterations = RANGE / RANGE_PER_PAGE;

unsigned int range_frequency = FREQ_END - FREQ_BEGIN;
unsigned int median_frequency = FREQ_BEGIN + range_frequency / 2;

// Measurement bandwidth. Allowed bandwidth values (in kHz) are:
// 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0, 46.9, 58.6,
// 78.2, 93.8, 117.3, 156.2, 187.2, 234.3, 312.0, 373.6 and 467.0
#define BANDWIDTH 467.0 

// (optional) major and minor tickmarks at x MHz
#define MAJOR_TICKS 10
#define MINOR_TICKS 5

#define ONE_MILLISEC 1

// Turns the 'PRG' button into the power button, long press is off
#define HELTEC_POWER_BUTTON // must be before "#include <heltec_unofficial.h>"
#include <Arduino.h>
#include <heltec_unofficial.h>
#include <images.h>

// This file contains a binary patch for the SX1262
#include "modules/SX126x/patches/SX126x_patch_scan.h"

// Prints debug information and the scan measurement bins from the SX1262 in hex
//#define PRINT_DEBUG
// Change spectrum plot values at once or by line
#define ANIMATED_RELOAD true

#define MAJOR_TICK_LENGTH 2
#define MINOR_TICK_LENGTH 1
// WEIGHT of the x-asix line
#define X_AXIS_WEIGHT 1
// Height of the plotter area
#define HEIGHT RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE
//
#define SCALE_TEXT_TOP (HEIGHT + X_AXIS_WEIGHT + MAJOR_TICK_LENGTH)
#define STATUS_TEXT_TOP (64 - 10)

// Detection level from the 33 levels. The higher number is more sensitive
#define DEFAULT_DRONE_DETECTION_LEVEL 21

#define BUZZER_PIN 41
// REB trigger PIN
#define REB_PIN 42

#define SCREAN_HEIGHT 64

#define WATERFALL_ENABLED true
#define WATERFALL_START 37
#define OSD_ENABLED true

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
// Used as a Led Light and Buzzer/count trigger
bool led_flag = false;
bool first_run = false;
// drone detection flag
bool drone_detected = false;
bool detected = false;
unsigned int drone_detection_level = DEFAULT_DRONE_DETECTION_LEVEL;
unsigned int drone_detected_frequency_start = 0;
unsigned int drone_detected_frequency_end = 0;
unsigned int detection_count = 0;
bool single_page_scan = false;
bool SOUND_ON = true;

unsigned int start_scan_text = (128 / 2) - 3;

unsigned int scan_time = 0;
unsigned int scan_start_time = 0;

uint64_t start = 0;

unsigned int x, y, i, w = 0;
unsigned int ranges_count = 0;

float freq = 0;
int rssi = 0;
int state = 0;
int result_index = 0;

unsigned int button_pressed_counter = 0;

void clearStatus()
{
  // clear status line
  display.setColor(BLACK);
  display.fillRect(0, STATUS_TEXT_TOP + 2, 128, 13);
  display.setColor(WHITE);
}

void clearPlotter()
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

    if (tick <= 128 - 3)
    {
      display.drawLine(tick, HEIGHT + X_AXIS_WEIGHT, tick, HEIGHT + X_AXIS_WEIGHT + length);
      // Central tick
      if (tick > (128 / 2) - 3 && tick < (128 / 2) + 3)
      {
        display.drawLine(tick + 1, HEIGHT + X_AXIS_WEIGHT, tick + 1, HEIGHT + X_AXIS_WEIGHT + length);
      }
    }

#ifdef MINOR_TICKS
    // Fix two ticks together
    if (tick_minor + 1 != tick && tick_minor - 1 != tick && tick_minor + 2 != tick && tick_minor - 2 != tick)
    {
      display.drawLine(tick_minor, HEIGHT + X_AXIS_WEIGHT, tick_minor, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
    }
    // Central tick
    if (tick_minor > (128 / 2) - 3 && tick_minor < (128 / 2) + 3)
    {
      display.drawLine(tick_minor + 1, HEIGHT + X_AXIS_WEIGHT, tick_minor + 1, HEIGHT + X_AXIS_WEIGHT + MINOR_TICK_LENGTH);
    }
#endif
  }
}

/**
 * @brief Decorates the display: everything but the plot itself.
 */
void displayDecorate(int begin = 0, int end = 0, bool redraw = false)
{
  if (!initialized)
  {
    // Start and end ticks
    display.fillRect(0, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH + 1);
    display.fillRect(126, HEIGHT + X_AXIS_WEIGHT, 2, MAJOR_TICK_LENGTH + 1);

    // Drone detection level
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(drone_detection_level));
  }

  if (!initialized || redraw)
  {
    // Clear something
    display.setColor(BLACK);
    display.fillRect(0, SCALE_TEXT_TOP + 1, 128, 12);
    display.setColor(WHITE);

    // Drone detection level
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 0, String(drone_detection_level));

    // Frequency start
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, SCALE_TEXT_TOP, (begin == 0) ? String(FREQ_BEGIN) : String(begin));

    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(128 / 2, SCALE_TEXT_TOP, (begin == 0) ? String(median_frequency) : String(begin + ((end - begin) / 2)));

    // Frequency end
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, SCALE_TEXT_TOP, (end == 0) ? String(FREQ_END) : String(end));
  }

  if (led_flag == true && detection_count >= 5)
  {
    digitalWrite(LED, HIGH);
    if (SOUND_ON)
    {
      tone(BUZZER_PIN, 104, 100);
    }
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

    display.drawString(start_scan_text, STATUS_TEXT_TOP, String(drone_detected_frequency_start) + ">RF<" + String(drone_detected_frequency_end));
  }

  if (ranges_count == 0)
  {
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, STATUS_TEXT_TOP, String(FREQ_BEGIN));

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, STATUS_TEXT_TOP, String(FREQ_END));
  }
  else if (ranges_count > 0)
  {
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, STATUS_TEXT_TOP, String(SCAN_RANGES[i] / 1000) + "-" + String(SCAN_RANGES[i] % 1000));
    if (i + 1 < iterations)
    {
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.drawString(128, STATUS_TEXT_TOP, String(SCAN_RANGES[i + 1] / 1000) + "-" + String(SCAN_RANGES[i + 1] % 1000));
    }
  }

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
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(REB_PIN, OUTPUT);
  heltec_setup();
  display.clear();
  // draw the UCOG welcome logo
  display.drawXbm(0, 2, 128, 64, epd_bitmap_ucog);
  display.display();

  for (int i = 0; i < 200; i++)
  {
    button.update();
    delay(10);
    if (button.pressed())
    {
      SOUND_ON = false;
      tone(BUZZER_PIN, 205, 100);
      delay(50);
      tone(BUZZER_PIN, 205, 100);
      break;
    }
  }

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
  both.println("Starting scanning...");
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
        RANGE_PER_PAGE = DEFAULT_RANGE_PER_PAGE;
        single_page_scan = false;
        tone(BUZZER_PIN, 205, 100);
        delay(50);
        tone(BUZZER_PIN, 205, 100);
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
        tone(BUZZER_PIN, 205, 100);

        break;
      }
    }
  }
  display.clear();

  // waterfall start line y-axis
  w = WATERFALL_START;
}

void loop()
{
  displayDecorate();
  drone_detected = false;
  detection_count = 0;
  drone_detected_frequency_start = 0;
#ifdef PRINT_PROFILE_TIME
  start = millis();
#endif

  if (!ANIMATED_RELOAD || !single_page_scan)
  {
    // clear the scan plot rectangle
    clearPlotter();
  }

  // do the scan
  range = FREQ_END - FREQ_BEGIN;

  if (RANGE_PER_PAGE > range)
  {
    RANGE_PER_PAGE = range;
  }

  fr_begin = FREQ_BEGIN;
  fr_end = fr_begin;
  // 50 is a single-screen range
  // TODO: Make 50 a variable with the option to show the full range
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

  ranges_count = 0;

  for (int range : SCAN_RANGES)
  {
    ranges_count++;
  }

  if (ranges_count > 0)
  {
    iterations = ranges_count;
    single_page_scan = false;
  }

  // Iterating by small ranges by 50 Mhz each pixel is 0.4 Mhz
  for (i = 0; i < iterations; i++)
  {
    range = RANGE_PER_PAGE;

    if (ranges_count == 0)
    {
      fr_begin = (i == 0) ? fr_begin : fr_begin += range;
      fr_end = fr_begin + RANGE_PER_PAGE;
    }
    else
    {
      fr_begin = SCAN_RANGES[i] / 1000;
      fr_end = SCAN_RANGES[i] % 1000;
      range = fr_end - fr_begin;
    }

    if (!ANIMATED_RELOAD || !single_page_scan)
    {
      // clear the scan plot rectangle
      clearPlotter();
    }

    if (single_page_scan == false)
    {
      displayDecorate(fr_begin, fr_end, true);
    }

    drone_detected_frequency_start = 0;

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    // horizontal x axis loop
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
      // TODO: RSSI METHOD
      // Gets RSSI (Recorded Signal Strength Indicator)
      // Restart continuous receive mode on the new frequency
      // state = radio.startReceive();
      // if (state == RADIOLIB_ERR_NONE) {
      // Serial.println(F("Started continuous receive mode"));
      //} else {
      // Serial.print(F("Failed to start receive mode, error code: "));
      // Serial.println(state);
      //}
      // rssi = radio.getRSSI(false);
      // Serial.println(String(rssi) + "db");
      // delay(25);
      // This code will iterate over the specified frequencies, changing the frequency every
      // second and printing the RSSI value for each frequency to the serial monitor. Adjust the frequencies array
      // to include the specific frequencies you're interested in monitoring.
      // A short delay after changing the frequency
      // ensures the module has time to stabilize and get an accurate RSSI reading.
#ifdef PRINT_DEBUG
      Serial.println();
      Serial.print("step-");
      Serial.print(x);
      Serial.print(" Frequency:");
      Serial.print(freq);
      Serial.println();
#endif
      // SpectralScan Method
      if (!RSSI_METHOD)
      {
        // start spectral scan third parameter is a sleep interval
        radio.spectralScanStart(SAMPLES, 1);
        // wait for spectral scan to finish
        while (radio.spectralScanGetStatus() != RADIOLIB_ERR_NONE)
        {
          Serial.print("radio.spectralScanGetStatus ERROR: ");
          Serial.println(radio.spectralScanGetStatus());
          heltec_delay(ONE_MILLISEC);
        }
        // read the results Array to which the results will be saved
        radio.spectralScanGetResult(result);
      }

      // Spectrum analyzer using getRSSI
      if (RSSI_METHOD)
      {
        state = radio.startReceive(0);
        if (state == RADIOLIB_ERR_NONE)
        {
#ifdef PRINT_DEBUG
          Serial.println(F("Started continuous receive mode"));
#endif
        }
        else
        {
          Serial.print(F("Failed to start receive mode, error code: "));
          Serial.println(state);
        }

        for (int r = 1; r < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; r++)
        {
          result[r] = 0;
        }
        result_index = 0;
        // N of samples
        for (int r = 1; r < SAMPLES_RSSI; r++)
        {
          rssi = radio.getRSSI(false);
          // delay(ONE_MILLISEC);
          // ToDO: check if 4 is correct value for 33 power bins
          result_index = (abs(rssi) / 4);

          // Debug Information
#ifdef PRINT_DEBUG
          Serial.print("Frequency: ");
          Serial.println(freq);
          Serial.println(rssi);
          Serial.println(result_index);
#endif
        // Saving max value only rss is negative so smaller is bigger
        if (result[result_index] > rssi)
        {
          result[result_index] = rssi;
        }
      }
    }
    detected = false;
#ifdef FILTER_SPECTRUM_RESULTS
    // Filter Elements without neighbors
    for (y = 1; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
    {
      // if RSSI method actual value is -xxx dB
      if (result[y] && (result[y + 1] != 0 || result[y - 1] != 0))
      {
        // Filling the empty pixel between signals int the level < 27 (noise level)
        /* if (y < 27 && result[y + 1] == 0 && result[y + 2] > 0)
         {
           result[y + 1] = 1;
           filtered_result[y + 1] = 1;
         }*/
        filtered_result[y] = 1;
      }
      else
      {
        filtered_result[y] = 0;
      }
    }
#endif
    for (y = 0; y < RADIOLIB_SX126X_SPECTRAL_SCAN_RES_SIZE; y++)
    {
#ifdef PRINT_DEBUG
      Serial.printf("%04X,", result[y]);
#endif
      if (result[y] || y == drone_detection_level)
      {
        // check if we should alarm about a drone presence
        if (filtered_result[y] == 1 && y <= drone_detection_level)
        {
          drone_detected = true;
#ifdef WATERFALL_ENABLED
          if (single_page_scan)
          {
            // Drone detection true for waterfall
            waterfall[i][x][w] = true;
            display.setColor(WHITE);
            display.setPixel(x, w);
          }
#endif
          if (drone_detected_frequency_start == 0)
          {
            drone_detected_frequency_start = freq;
          }
          drone_detected_frequency_end = freq;
          led_flag = true;
          // If level is set to sensitive, start beeping every 10th frequency and shorter
          if (drone_detection_level <= 25)
          {
            if (detection_count == 1 && SOUND_ON)
              tone(BUZZER_PIN, 205, 10);
            if (detection_count % 5 == 0 && SOUND_ON)
              tone(BUZZER_PIN, 205, 10);
          }
          else
          {
            if (detection_count % 20 == 0 && SOUND_ON)
              tone(BUZZER_PIN, 205, 10);
          }
          display.setPixel(x, 1);
          display.setPixel(x, 2);
          display.setPixel(x, 3);
          display.setPixel(x, 4);
        }
#ifdef WATERFALL_ENABLED
        if (filtered_result[y] == 1 && y > drone_detection_level && single_page_scan && waterfall[i][x][w] != true)
        {
          // If drone not found set dark pixel on the waterfall
          // TODO: make something like scrolling up if possible
          waterfall[i][x][w] = false;
          display.setColor(BLACK);
          display.setPixel(x, w);
          display.setColor(WHITE);
        }
#endif
        if (filtered_result[y] == 1)
        {
          // Set signal level pixel
          display.setPixel(x, y);
          detected = true;
        }

        // Draw detection Level line evere 2 pixel
        if (y == drone_detection_level && x % 2 == 0)
        {
          display.setPixel(x, y);
        }
      }

#ifdef PRINT_PROFILE_TIME
      scan_time = millis() - scan_start_time;
      // Huge performance issue if enable
      // Serial.printf("Single Scan took %lld ms\n", scan_time);
#endif
    }
    if (detected)
    {
      detection_count++;
    }
    detected = false;
#ifdef PRINT_DEBUG
    Serial.println("....");
#endif
    if (first_run || ANIMATED_RELOAD)
    {
      display.display();
    }

    // Detection level button short press
    if (button.pressedFor(100))
    {
      button.update();
      button_pressed_counter = 0;
      // if long press stop
      while (button.pressedNow())
      {
        delay(10);
        // Print Curent frequency
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(128 / 2, 0, String(freq));
        display.display();
        button_pressed_counter++;
        if (button_pressed_counter > 150)
        {
          digitalWrite(LED, HIGH);
          delay(150);
          digitalWrite(LED, LOW);
        }
      }
      if (button_pressed_counter > 150)
      {
        // Remove Curent Freqancy Text
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setColor(BLACK);
        display.drawString(128 / 2, 0, String(freq));
        display.setColor(WHITE);
        display.display();
        break;
      }
      if (button_pressed_counter > 50 && button_pressed_counter < 150)
      {
        // Visually confirm it's off so user releases button
        display.displayOff();
        // Deep sleep (has wait for release so we don't wake up immediately)
        heltec_deep_sleep();
        break;
      }
      button.update();

      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      // erase old value
      display.setColor(BLACK);
      display.fillRect(128 - 13, 0, 13, 13);
      display.setColor(WHITE);
      drone_detection_level++;
      // print new value
      display.drawString(128, 0, String(drone_detection_level));

      tone(BUZZER_PIN, 104, 150);

      if (drone_detection_level > 30)
      {
        drone_detection_level = 1;
      }
    }
    // wait a little bit before the next scan, otherwise the SX1262 hangs
    // Add more logic before insead of long delay...
    // heltec_delay(1);
    // Loop is needed if heltec_delay(1) not used
    heltec_loop();
  }
  w++;
  if (w > STATUS_TEXT_TOP + 1)
  {
    w = WATERFALL_START;
  }
#ifdef WATERFALL_ENABLED
  // Draw waterfall position cursor
  if (single_page_scan)
  {
    display.setColor(BLACK);
    display.drawHorizontalLine(0, w, STEPS);
    display.setColor(WHITE);
  }
#endif
  display.display();
}

#ifdef PRINT_DEBUG
Serial.println("----");
#endif
// display.display();
#ifdef PRINT_PROFILE_TIME
scan_time = millis() - start;
Serial.printf("Scan took %lld ms\n", scan_time);
#endif
}
