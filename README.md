# Lora SA(Spectrum Analyzer)

# Supportted boards: 
- Heltec Lora V3 128 x 64 OLED
- Heltec Wireless Stick V3 64 x 32 (Not tested)
- Heltec Wireless Stick Lite V3 No Display (Not Tested)
- Heltec Vision Master E290 - e-Ink 296 x 128 (No OSD)
- Heltec Vision MAster T190 - color TFT 320X170 (No OSD)
- LilyGo Radio Lora T3S3 V.2 SX1262
- LilyGo Radio Lora T3S3 V.2 SX1280
- LilyGo Radio Lora T3_V1.6.1 SX1276 (Not Tested)

## RF Spectrum Analyzer using Lora Radio

<img src="https://github.com/user-attachments/assets/4caeb467-1964-4184-ab20-ba68b97144aa" alt="LORA hardware" width="200"/>

Based on RadioLib SX126x Spectrum Scan.

Perform a spectrum power scan using SX126x.
The output is in the form of scan lines; each line has 33 power bins.
The first power bin corresponds to -11 dBm, the second to -15 dBm, and so on.
The higher number of samples in a bin corresponds to more power received
at that level.

```text
N in Bin / dBm
(0)1 -11
2 -15
3 -19
4 -23
5 -27
6 -31
7 -35
8 -39
9 -43
10 -47
11 -51
12 -55
13 -59
14 -63
15 -67
16 -71
17 -75
18 -79
19 -83
20 -87
21 -91
22 -95
23 -99
24 -103
25 -107
26 -111
27 -115
28 -119
29 -123
30 -127
31 -131
32 -135
33 -139
```

Example:

```text
step-13 Frequency:816.25
Power Bins: 0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0400,  0000,0000,0000,0000,0000,0000,0006,001B,000E,0005,0006,0002,0000
```

The spectrum analyzer performs power measurements in the configured bandwidth.

The X-axis represents frequency in MHz, and the Y-axis displays actually received power.
In the example above, the frequency span goes from 850 MHz to 950 MHz (that is a 100MHz range), and
the visual amplitude goes from -11 dBm to -110(-139) according to the datasheet (High sensitivity: down to -148dBm) dBm.

To show the results in a plot, run the Python script
RadioLib/extras/SX126x_Spectrum_Scan/SpectrumScan.py

## Features

### Multiple Ranges Scan

Disabled By Default

```c
// Feature to scan diapazones. Other frequency settings will be ignored.
int SCAN_DIAPAZONES[] = {};
//int SCAN_DIAPAZONES[] = {850890, 920950};
```

To Enable Add/ uncomment an array of the frequencies

```c
int SCAN_DIAPAZONES[] = {850890, 920950};
```

where 850890 stands for 950-890Mhz range
920950 - 920-890Mhz
Other settings will be ignored if **Multiple Ranges Scan** is enabled.

### Waterfall

Waterfall showed only on One Page Scan
to disable - uncomment this line

```c
#define WATERFALL_ENABLED true
```

Waterfall shows the last **N** = SCREEN_HEIGHT (64) - WATERFALL_START(37) - 8 (part of the ROW_STATUS_TEXT)  = **19** signal detection that excited set signal level

### RSSI Method of scan

By default, we are using the spectralScan method of the RadioLib Library: <https://jgromes.github.io/RadioLib/class_s_x126x.html#a8a3ad4e12df862ab18b326d9dba26d66>
This method works only with Sx1262 modules.
We implemented a scan using the **getRSSI** method, which has more flexibility and supports sx1276 and other modules.
Using this method, we also receive the signal's **dB** values, not just the O-33 number.
To enable this method, set the value of the **RSSI_METHOD** to true.

### Multi Screen Scan

Single screen scan for now has **RANGE / 128** resolution.
Multi-page scan can be adjusted to how many MHz per page you wanna scan

```c
// frequency range in MHz to scan
#define FREQ_BEGIN 850
// TODO: if % RANGE_PER_PAGE  != 0
#define FREQ_END 950

// Feature to scan diapazones. Other frequency settings will be ignored.
int SCAN_DIAPAZONES[] = {};
// int SCAN_DIAPAZONES[] = {850890, 920950};

// MHZ per page
//To put everything into one page set RANGE_PER_PAGE = FREQ_END - 800
unsigned int RANGE_PER_PAGE = FREQ_END - FREQ_BEGIN; // FREQ_END - FREQ_BEGIN
//To Enable Multi Screen scan
// unsigned int RANGE_PER_PAGE = 50;
// Default Range on Menu Button Switch 
#define DEFAULT_RANGE_PER_PAGE = 50;
```

To enable Multi-page by default set **RANGE_PER_PAGE** less than  **FREQ_END - FREQ_BEGIN**;
Switch to multi-page during regular One Screen application run. Restart the ESP32 on screen after the logo press the P button.

### Mute Audio Notifications

Restart ESP32, and on the logo display, press the P button.

### Pause Execution

Press P for more than 2 seconds. Execution will pause, and the scan's current Mhz position will be shown on the display.
If less, ESP32 will turn off. Fast pressing(less than 0.5 second) P button changes the notification level

## VSCode Platform.IO development env installation

1. Install VSCode
2. install Platform.IO extension
   ![image](https://github.com/user-attachments/assets/00547068-6153-4c78-9f12-54981b013b06)
3. Connect ESP32 to USB. Install USB CP2101 drivers for Windows or other OS
   https://docs.heltec.org/general/establish_serial_connection.html#for-windows
   https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads

   ## NOTE: MACOS Heltec USB driver 
   https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads <br/>
   I used legacy driver
   
5. Clone this Git Repo or download zip of the sources
   ![image](https://github.com/user-attachments/assets/971b6592-3b71-414c-971c-2ecd20f0f0b7)

    ```bash
    git clone https://github.com/Genaker/LoraSA.git
    ```

   NOTE: in you case name will be Just LoraSA. I have LoraSA2 because I already have LoraSA folder

6. Open the Project with the VS code Platform.IO
   ![image](https://github.com/user-attachments/assets/5066836b-32ac-4a24-ac03-b5f739a6a658)
   ![image](https://github.com/user-attachments/assets/da14488d-7a4a-410e-b754-59598578016d)

7. Select Proper Environment
   ![image](https://github.com/user-attachments/assets/a9c6557b-a387-4457-b59b-b3d7242d2826)

   ---
   
   >**Important note:** make sure your ESP32 Expressif catalog is up to date before selecting environment, otherwise you'll get error `Error: Unknown board ID 'heltec_wifi_lora_32_V3'` when trying to select environment.
   >
   >Open a PlatformIO CLI: https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-core-cli
   >
   >Run: `pio pkg update -g -p espressif32`
   
9. Select ESP32 USB Device to program
   ![image](https://github.com/user-attachments/assets/af76c4b1-7122-45e1-b26b-08b59e03ca3b)
Note: It is theoretically possible to program via WiFi and BTH.
10. Program your ESP32
    ![image](https://github.com/user-attachments/assets/9e67afd8-0522-4a96-82dc-8e1cdb32add5)

11. Wait until you are done with the compilation and upload.
    Usually takes 1 minute. The first run is slower. It needs to compile all libraries.
    ![image](https://github.com/user-attachments/assets/6796eb5d-6e3f-45bc-b88c-251499f1ad47)
You will have the UCOG SA logo and spectrum analyzing scanning screen when done.
![image](https://github.com/user-attachments/assets/f86ab32a-cab1-461e-ade9-7021136a0af7)

## Hardware

Heltec ESP32 Lora V3:
<https://www.amazon.com/Heltec-Development-863-870MHz-ESP32-S3FN8-902-928MHz/dp/B0D1H1FN9Y/>
<https://heltec.org/project/wifi-lora-32-v3/>
<https://www.aliexpress.us/item/3256807037422978.html>

Or Heltec Wireless Stick. The same hardware but without or with a smaller display 
https://heltec.org/project/wireless-stick-v3/
https://heltec.org/project/wireless-stick-lite-v2/

Heltec Vision Master E290 - With large e-ink display 293x128:

https://heltec.org/project/vision-master-e290/
https://www.aliexpress.us/item/3256807048047234.html
**NOTE: to upload a new code, you need to press BOOT + RESET button**

Battery with Wire JT connector :
<https://www.amazon.com/EEMB-2000mAh-Battery-Rechargeable-Connector/dp/B08214DJLJ>

## 3D printed case

![image](https://github.com/user-attachments/assets/52ae4e90-5f25-4d72-888e-7586bef9df69)
<https://www.printables.com/model/118750-heltec-lora-32-case-for-meshtastic>
<https://www.thingiverse.com/thing:3125854>
<https://thangs.com/designer/Snake0017/3d-model/Heltec%20LoRa%2032%20Desktop%20%26%20Vehicle%20Enclosure-40844>
or buy :
<https://www.amazon.com/DIYmalls-ESP32-OLED-WiFi-Type-C/dp/B0BR3MQ9BG>

<https://www.thingiverse.com/thing:6522462>

## Heltec ESP32 Lora v3 Pin Map

![image](https://github.com/user-attachments/assets/a1e00b51-5566-4ff5-98fe-67eaeb5bc81f)
We are using pin 41 as a Buzzer trigger. Connect buzzer + leg with pin 41 and - leg with the ground (GND). You can change the buzzer pin in the code.


## Analog FPV OSD (ON SCREEN DISPLAY)
To Enable OSD, Uncomment these lines </br>
```
//  #define OSD_ENABLED true
```
**OSD sidebar enabled/disable**
comment or uncomment  this line
```
#define OSD_SIDE_BAR true
```

Or you can set this and other variables as a build parameter: 
```   
build_flags = 
	-DOSD_ENABLED
```

## DFRobot OSD Wiring 
**Heltec V3 -> DFRobot OSD** <br />
GND -> GND <br />
3V3 -> 3V3 <br />
26 -> SCK <br />
34 ->MOSI <br />
33 ->MISO <br />
47 -> D3 <br />

More photos you can see there:  <br />
https://github.com/Genaker/LoraSA/issues/11

![image](https://github.com/user-attachments/assets/3ba8230d-21de-449d-881b-bdc5f5b4907d)

# Camera to DF robot Wiring
**Camera -> DFRobotOSD** <br />
Video out -> In <br />
GND -> GND <br />
3v3 -> 3V3 Heltec or some 3v on FPV <br />

# DFRobot to Drone or VTX(video transmitter)  
**DF Robot -> VTX or** <br />
Video Out - Video IN <br />

```
// SPI pins
#define OSD_CS 47
#define OSD_MISO 33
#define OSD_MOSI 34
#define OSD_SCK 26
```

## Joystick Wiring 
https://www.aliexpress.us/item/2251832815289133.html 
https://www.amazon.com/dp/B00P7QBGD2

**Loystic -> Heltec V3**  <br />
SW -> 46 <br />
VRX -> 19 <br />
VRY -> X has not been implemented yet  <br />
+5v -> 5V <br />
GND -> GND <br />

## Buzzer/Beeper Wiring
TMB12A03 - in my case. Low voltage is better.  <br />
**Buzzer -> Heltec V3**  <br />
(+) -> 41 <br />
GND (another) -> GND  <br />

## Select Board to build
Select Visual Code environment:
![image](https://github.com/user-attachments/assets/3765615b-3a80-4270-bc74-8f6eae2b8458)

Edit **paltformio.io** uncommenting/selecting your sources
```
[platformio]
; for env:vision-master-e190
; src_dir = tft_src
; for env:vision-master-e290
; src_dir = eink_src
; for env:heltec_wifi_lora_32_V3
; src_dir = src ;;Default
```
for LilyGo use env:heltec_wifi_lora_32_V3

# WiFi and Bluetooth BT Scanning
Works only with OSD enabled <br/>
Uncomment this lines 
```
//  #define OSD_ENABLED true
//  #define WIFI_SCANNING_ENABLED true
//  #define BT_SCANNING_ENABLED true
```
