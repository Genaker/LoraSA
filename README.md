# Lora SA(Spectrum Analyzer)

## Index

- [Supported boards](#supported-boards)
- [RF Spectrum Analyzer using Lora Radio](#rf-spectrum-analyzer-using-lora-radio)
- [Features](#features)
  - [Multiple Ranges Scan](#multiple-ranges-scan)
  - [Waterfall](#waterfall)
  - [RSSI Method of scan](#rssi-method-of-scan)
  - [Multi Screen Scan](#multi-screen-scan)
  - [Mute Audio Notifications](#mute-audio-notifications)
  - [Pause Execution](#pause-execution)
- [VSCode Platform.IO development env installation](#vscode-platformio-development-env-installation)
- [Hardware](#hardware)
- [3D printed case](#3d-printed-case)
- [Heltec ESP32 Lora v3 Pin Map](#heltec-esp32-lora-v3-pin-map)
- [Analog FPV OSD (ON SCREEN DISPLAY) using DFRobot OSD](#analog-fpv-osd-on-screen-display-using-dfrobot-osd)
- [Joystick Wiring](#joystick-wiring)
- [Buzzer/Beeper Wiring](#buzzerbeeper-wiring)
- [Select Board to build](#select-board-to-build)
- [WiFi and Bluetooth BT Scanning](#wifi-and-bluetooth-bt-scanning)
- [Communication via Serial USB](#communication-via-serial-usb)
- [Web app](#web-app)
- [Send Scan Data via Lora](#send-scan-data-via-lora)
- [Seek on Jam / FPV OSD using Flight controller](#seek-on-jam-fpv-osd-using-flight-controller)
- [Platformio targets](#platformio-targets)

## Supported boards: 

### Heltec Automation:

   - Heltec Lora V3 128 x 64 OLED   
   - Heltec Wireless Stick V3 64 x 32 (Not tested)
   - Heltec Wireless Stick Lite V3 No Display (Not Tested)
   - Heltec Vision Master E290 - e-Ink 296 x 128 (No OSD)
   - Heltec Vision Master T190 - color TFT 320X170 (No OSD)

### LilyGo:

   - LilyGo Radio Lora T3S3 V.2 SX1262
   - LilyGo Radio Lora T3S3 V.2 SX1280
   - LilyGo Radio Lora T3S3 V.2 LR1121
   - LilyGo Radio Lora T3_V1.6.1 SX1276 (Not Tested)

## RF Spectrum Analyzer using Lora Radio

<img src="https://github.com/user-attachments/assets/4caeb467-1964-4184-ab20-ba68b97144aa" alt="LORA hardware" width="200"/>

Based on RadioLib SX126x Spectrum Scan.

Perform a spectrum power scan using SX126x.
The output is in the form of scan lines; each line has 33 power bins.
The first power bin corresponds to -11 dBm, the second to -15 dBm, and so on.
The higher number of samples in a bin corresponds to more power received
at that level.

<details>
<summary>Click to expand to show BIN <> dBm mapping</summary>

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
</details>

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

Waterfall shows the last **N** = SCREEN_HEIGHT (64) - WATERFALL_START(37) - 8 (part of the ROW_STATUS_TEXT)  = **19** signal detection that exceeded set signal level

### RSSI Method of scan

By default, we are using the spectralScan method of the RadioLib Library: <https://jgromes.github.io/RadioLib/class_s_x126x.html#a8a3ad4e12df862ab18b326d9dba26d66>
This method works only with Sx1262 modules.
We implemented a scan using the **getRSSI** method, which has more flexibility and supports sx1276 and other modules.
Using this method, we also receive the signal's **dB** values, not just the O-33 number.
To enable this method, set the value of the **RSSI_METHOD** to true.

### Multi Screen Scan

Single screen scan for now has **RANGE / 128** resolution.
Multi-page scan can be adjusted to how many MHz per page you want to scan

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

   NOTE: in your case name will be just LoraSA. I have LoraSA2 because I already have LoraSA folder

6. Open the Project with the VS code Platform.IO
   ![image](https://github.com/user-attachments/assets/5066836b-32ac-4a24-ac03-b5f739a6a658)
   ![image](https://github.com/user-attachments/assets/da14488d-7a4a-410e-b754-59598578016d)

7. Select Proper Environment
   ![image](https://github.com/user-attachments/assets/a9c6557b-a387-4457-b59b-b3d7242d2826)

   ---
   
   >**Important note:** If using a Heltec V3 board, make sure your ESP32 Expressif catalog is up to date before selecting environment, otherwise might get a build time error such as: `Error: Unknown board ID 'heltec_wifi_lora_32_V3'` when trying to select the environment.
   >
   >Open a PlatformIO CLI: https://docs.platformio.org/en/latest/integration/ide/vscode.html#platformio-core-cli
   >
   >Run: `pio pkg update -g -p espressif32`
   
8. Select ESP32 USB Device to program
   ![image](https://github.com/user-attachments/assets/af76c4b1-7122-45e1-b26b-08b59e03ca3b)
Note: It is theoretically possible to program via WiFi and BTH.
9. Program your ESP32
    ![image](https://github.com/user-attachments/assets/9e67afd8-0522-4a96-82dc-8e1cdb32add5)

10. Wait until you are done with the compilation and upload.
    Usually takes 1 minute. The first run is slower. It needs to compile all libraries.
    ![image](https://github.com/user-attachments/assets/6796eb5d-6e3f-45bc-b88c-251499f1ad47)
You will have the UCOG SA logo and spectrum analyzing scanning screen when done.
![image](https://github.com/user-attachments/assets/f86ab32a-cab1-461e-ade9-7021136a0af7)

## Hardware

- Heltec ESP32 Lora V3:
   - [Amazon](https://www.amazon.com/Heltec-Development-863-870MHz-ESP32-S3FN8-902-928MHz/dp/B0D1H1FN9Y/)
   - [Heltec](https://heltec.org/project/wifi-lora-32-v3/)
   - [AliExpress](https://www.aliexpress.us/item/3256807037422978.html)

- Heltec Wireless Stick. The same hardware but without or with a smaller display 
   - [Heltec: Wireless Stick V3](https://heltec.org/project/wireless-stick-v3/)
   - [Heltec: Wireless Stick Lite V3](https://heltec.org/project/wireless-stick-lite-v2/)

- Heltec Vision Master E290 - With large e-ink display 293x128:
   - [Heltec](https://heltec.org/project/vision-master-e290/)
   - [AliExpress](https://www.aliexpress.us/item/3256807048047234.html)

- Heltec Vision Master T190 - With large color TFT 320x170 display:
   - [Heltec](https://heltec.org/project/vision-master-t190/)
   - [AliExpress](https://www.aliexpress.us/item/1005007760785910.html)

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


## Analog FPV OSD (ON SCREEN DISPLAY) using DFRobot OSD:

To Enable OSD, Uncomment these lines </br>

<details>
<summary>Click to expand to show how to set up  OSD for DFrobot board</summary>

```cpp
//  #define OSD_ENABLED true
```

**OSD sidebar enabled/disable**

comment or uncomment  this line
```cpp
#define OSD_SIDE_BAR true
```

Or you can set this and other variables as a build parameter: 

```   
build_flags = 
	-DOSD_ENABLED
```

#### DFRobot OSD Wiring 

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

#### Camera to DF robot Wiring
**Camera -> DFRobotOSD** <br />
Video out -> In <br />
GND -> GND <br />
3v3 -> 3V3 Heltec or some 3v on FPV <br />

#### DFRobot to Drone or VTX(video transmitter)  
**DF Robot -> VTX or** <br />
Video Out - Video IN <br />

```
// SPI pins
#define OSD_CS 47
#define OSD_MISO 33
#define OSD_MOSI 34
#define OSD_SCK 26
```

</details>

## Joystick Wiring 
https://www.aliexpress.us/item/2251832815289133.html 
https://www.amazon.com/dp/B00P7QBGD2

**Joystick -> Heltec V3**  <br />
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

Edit **platformio.ini** uncommenting/selecting your sources

Heltec boards: no need to change anything, but for T190 and E290 you need to change the src_dir to tft_src or eink_src respectively.

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

## WiFi and Bluetooth BT Scanning

Works only with OSD enabled <br/>
Uncomment this lines 
```
//  #define OSD_ENABLED true
//  #define WIFI_SCANNING_ENABLED true
//  #define BT_SCANNING_ENABLED true
```

## Communication via Serial USB:

You can receive RAW frequency data from the ESP32 by sending the serial command **Scan -1 -1**. Other parameters are also available. 
Also, you can use ASCII_SA.py to read data from serial and represent as an ASCII character chart on your device. </br>
Install **pyserial**:
```
pip3 install pyserial
```

Run:
```
python3 ASCII_SA.py    
```
or with parameters to set input 
```
python3 ASCII_SA.py --resolution 1 --threshold -110 --db-per-hash 5 --no-color --no-debug
```
Output:

<img width="861" alt="image" src="https://github.com/user-attachments/assets/131842e6-2216-4da1-adb3-accb0fb80427">

## Web app 

We also have a web app interface to output ESP Lora data via USB/Serial </br>
Go to the website (https://lora-sa.pages.dev/), click connect and select a device: </br>
Known issue: doesn't work on mobile ;(</br>
https://lora-sa.pages.dev/

| ![image](https://github.com/user-attachments/assets/71b7d2aa-bb70-4c55-899b-0edfbeb73ed3) | <img width="1625" alt="image" src="https://github.com/user-attachments/assets/71acf551-f256-47ab-b95e-116b2fefcdaa"> |
|---|---|

## Send Scan Data via Lora 

Enable feature:
```
# /lib/config/config.h
listen_on_usb(String("readline")), rx_lora(NULL), tx_lora(NULL),
// Enable Lora Send:
// rx_lora(configureLora("freq:920")),tx_lora(configureLora("freq:916"))
is_host(false)
```
is_host(false) - for host receiver or can be set via Serial command ***SET is_host true*** <br>
The Host automatically outputs data to serial that can be read by our Python Serail app<br>
Example of the serial out:

<details>
<summary>Click to expand to show Lora Scan Data</summary>

```
SCAN_RESULT 258 [ (850000, -106), (850389, -106), (850778, -110), (851167, -110), (851556, -110), (851945, -110), (852334, -110), (852723, -110), (853112, -106), (853501, -106), (853890, -106), (854279, -83), (854668, -83), (855057, -110), (855446, -110), (855835, -110), (856224, -110), (856613, -110), (857002, -110), (857391, -110), (857780, -110), (858169, -110), (858558, -110), (858947, -110), (859336, -110), (859725, -110), (860114, -110), (860503, -110), (860892, -110), (861281, -110), (861670, -110), (862059, -110), (862448, -110), (862837, -110), (863226, -110), (863615, -106), (864004, -106), (864393, -110), (864782, -110), (865171, -110), (865560, -110), (865949, -110), (866338, -110), (866727, -110), (867116, -110), (867505, -110), (867894, -110), (868283, -110), (868672, -101), (869061, -101), (869450, -102), (869839, -102), (870228, -101), (870617, -101), (871006, -101), (871395, -98), (871784, -98), (872173, -89), (872562, -89), (872951, -92), (873340, -92), (873729, -85), (874118, -85), (874507, -90), (874896, -90), (875285, -90), (875674, -85), (876063, -85), (876452, -93), (876841, -93), (877230, -90), (877619, -90), (878008, -101), (878397, -101), (878786, -110), (879175, -110), (879564, -110), (879953, -90), (880342, -90), (880731, -93), (881120, -93), (881509, -96), (881898, -96), (882287, -91), (882676, -91), (883065, -91), (883454, -86), (883843, -86), (884232, -93), (884621, -93), (885010, -91), (885399, -91), (885788, -89), (886177, -89), (886566, -85), (886955, -85), (887344, -85), (887733, -87), (888122, -87), (888511, -93), (888900, -93), (889289, -110), (889678, -110), (890067, -110), (890456, -110), (890845, -110), (891234, -110), (891623, -110), (892012, -110), (892401, -110), (892790, -110), (893179, -110), (893568, -110), (893957, -110), (894346, -110), (894735, -110), (895124, -106), (895513, -106), (895902, -106), (896291, -110), (896680, -110), (897069, -110), (897458, -110), (897847, -110), (898236, -110), (898625, -110), (899014, -110), (899403, -110), (899792, -110), (900181, -110), (900570, -110), (900959, -110), (901348, -110), (901737, -110), (902126, -110), (902515, -110), (902904, -110), (903293, -110), (903682, -110), (904071, -110), (904460, -110), (904849, -110), (905238, -110), (905627, -110), (906016, -110), (906405, -110), (906794, -110), (907183, -110), (907572, -110), (907961, -110), (908350, -110), (908739, -110), (909128, -110), (909517, -110), (909906, -110), (910295, -110), (910684, -110), (911073, -110), (911462, -110), (911851, -110), (912240, -110), (912629, -110), (913018, -110), (913407, -110), (913796, -110), (914185, -110), (914574, -110), (914963, -110), (915352, -110), (915741, -110), (916130, -110), (916519, -110), (916908, -110), (917297, -110), (917686, -110), (918075, -110), (918464, -110), (918853, -110), (919242, -110), (919631, -110), (920020, -110), (920409, -110), (920798, -110), (921187, -110), (921576, -110), (921965, -110), (922354, -110), (922743, -110), (923132, -110), (923521, -110), (923910, -110), (924299, -110), (924688, -110), (925077, -110), (925466, -110), (925855, -110), (926244, -110), (926633, -110), (927022, -110), (927411, -110), (927800, -110), (928189, -110), (928578, -110), (928967, -110), (929356, -110), (929745, -110), (930134, -110), (930523, -110), (930912, -110), (931301, -110), (931690, -110), (932079, -110), (932468, -110), (932857, -110), (933246, -110), (933635, -110), (934024, -110), (934413, -110), (934802, -110), (935191, -110), (935580, -103), (935969, -103), (936358, -110), (936747, -110), (937136, -110), (937525, -110), (937914, -110), (938303, -110), (938692, -110), (939081, -110), (939470, -110), (939860, -110), (940250, -110), (940640, -110), (941030, -110), (941420, -110), (941810, -110), (942200, -110), (942590, -110), (942980, -110), (943370, -110), (943760, -110), (944150, -110), (944540, -110), (944930, -110), (945320, -110), (945710, -110), (946100, -110), (946490, -110), (946880, -110), (947270, -110), (947660, -110), (948050, -110), (948440, -110), (948830, -110), (949220, -110), (949610, -110), (950000, -110) ]
```
</details>


## Seek on Jam / FPV OSD using Flight controller:

Needs MILBETA 1.47 or later.

Use target `seek-on-jam-sx1280` or `seek-on-jam-lr1121`. Make sure the pins are correct.

I am using LilyGo T3S3 **V1.2** board for seek on jam, this board has a connector for GND, 3.3V, RX and TX for UART SERIAL. No need to solder to the board itself.

![imagen](https://github.com/user-attachments/assets/d238a87f-3768-4293-a90c-eb3e4b530130)

Only GND, 3.3V and TX are needed. 

Use connector and solder to RPI or similar, I'm using a RPI Zero 2. The targets in platformio.ini are set to use `-DTX_PIN=43` which corresponds to the built-in connector on the T3S3 V1.2 board. RPI Zero2 needs to use the proxy.

## Platformio targets:

### 1. `heltec_wifi_lora_32_V3`
- **Chip Used**: Heltec WiFi LoRa 32 V3
- **Frequency**: 150 - 850 MHz

### 2. `heltec_wifi_lora_32_V3-OSD`
- **Chip Used**: Heltec WiFi LoRa 32 V3
- **Frequency**: Not specified
- **Unique Features**: Includes OSD (On-Screen Display) specific flags `-DOSD_ENABLED` and `-DOSD_SIDE_BAR`.

### 3. `heltec_wifi_lora_32_V3_433`
- **Chip Used**: Heltec WiFi LoRa 32 V3
- **Frequency**: 130 MHz to 180 MHz
- **Unique Features**: Frequency range specified with `-DFREQ_BEGIN=130` and `-DFREQ_END=180`. 

### 4. `lilygo-T3S3-v1-2-sx1262`
- **Chip Used**: LilyGO T3 S3 V1.2 with SX1262
- **Frequency**: 150 - 850 MHz
- **Unique Features**: Uses SX1262 chip with specific flags like `-DT3_S3_V1_2_SX1262`.

### 5. `lilygo-T3S3-v1-2-lr1121`
- **Chip Used**: LilyGO T3 S3 V1.2 with LR1121
- **Frequency**: 2400 MHz to 2500 MHz

### 6. `lilygo-T3S3-v1-2-sx1280`
- **Chip Used**: LilyGO T3 S3 V1.2 with SX1280
- **Frequency**: 2400 MHz to 2500 MHz
- **Unique Features**: Includes `-DSERIAL_OUT` for serial output.

### 7. `lilygo-T3-v1-6-xs1276`
- **Chip Used**: LilyGO T3 V1.6 with SX1276
- **Frequency**: 150 - 850 MHz
- **Unique Features**: `-DARDUINO_USB_CDC_ON_BOOT=0` to avoid serial issues.
- **Warning**: Not tested yet.

### 8. `vision-master-e290`
- **Chip Used**: Heltec WiFi LoRa 32 V3
- **Frequency**: 150 - 850 MHz

### 9. `vision-master-t190`
- **Chip Used**: Heltec WiFi LoRa 32 V3
- **Frequency**: 150 - 850 MHz
- **Unique Features**: Includes `-DROTATION=1` for display rotation. 1 == normal 3 == upside down

### 10. `seek-on-mavic-sx1280`
- **Chip Used**: LilyGO T3 S3 V1.2 with SX1280
- **Frequency**: 2410 MHz to 2452 MHz (Mavic 3 frequency ranges)
- **Unique Features**: 
   - `-DLOG_DATA_JSON=1` for JSON logging
   - `-DDISABLE_SDCARD` to disable SD card and make boot faster
   - `-DSERIAL_OUT` to enable serial output
   - `-DSEEK_ON_X` to enable seek on jam
   - `-DBANDWIDTH=4.8` to set the bandwidth to 4.8 kHz

### 11. `seek-on-mavic-lr1121`
- **Chip Used**: LilyGO T3 S3 V1.2 with LR1121
- **Frequency**: 2410 MHz to 2452 MHz (Mavic 3 frequency ranges)
- **Unique Features**:
   - `-DLOG_DATA_JSON=1` for JSON logging
   - `-DDISABLE_SDCARD` to disable SD card and make boot faster
   - `-DSERIAL_OUT` to enable serial output
   - `-DSEEK_ON_X` to enable seek on jam
   - `-DBANDWIDTH=4.8` to set the bandwidth to 4.8 kHz
