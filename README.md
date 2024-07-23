# Lora SA(Spectrum Analyzer)
RF Spectrum Analyzer using Lora Radio

![IMG_5267](https://github.com/user-attachments/assets/4caeb467-1964-4184-ab20-ba68b97144aa)

RadioLib SX126x Spectrum Scan 

Perform a spectrum power scan using SX126x.
The output is in the form of scan lines; each line has 33 power bins.
The first power bin corresponds to -11 dBm, the second to -15 dBm, and so on.
The higher number of samples in a bin corresponds to more power received
at that level.

N in Bin / dBm
1	-11
2	-15
3	-19
4	-23
5	-27
6	-31
7	-35
8	-39
9	-43
10	-47
11	-51
12	-55
13	-59
14	-63
15	-67
16	-71
17	-75
18	-79
19	-83
20	-87
21	-91
22	-95
23	-99
24	-103
25	-107
26	-111
27	-115
28	-119
29	-123
30	-127
31	-131
32	-135
33	-139

Example:
step-13 Frequancy:816.25
Power Bins: 0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,  0000,0000,0000,0000,0000,0000,0000,0000,0000,0400,  0000,0000,0000,0000,0000,0000,0006,001B,000E,0005,   0006,0002,0000,

The spectrum analyzer performs power measurements in the configured bandwidth.

The X axis represents frequency in MHZ and Y axis display actual received power.
In this example above, the frequency span goes from 850 MHz to 950 MHz (that is a 100MHz range), and
the visual amplitude goes from -11 dBm to -110(-139) according to the datasheet(High sensitivity: down to -148dBm) dBm.

To show the results in a plot, run the Python script
RadioLib/extras/SX126x_Spectrum_Scan/SpectrumScan.py

# VSCode Platform.IO development env installation
1. Install VSCode
2. install Platfor.IO extension
   ![image](https://github.com/user-attachments/assets/00547068-6153-4c78-9f12-54981b013b06)
3. Connect ESP32 to USB. Install USB drivers for Windows
4. Clone this Git Repo or download zip of the sources
   ![image](https://github.com/user-attachments/assets/971b6592-3b71-414c-971c-2ecd20f0f0b7)
   ```
   git clone https://github.com/Genaker/LoraSA.git
   ```
NOTE: in you case name will be Just LoraSA. I have LoraSA2 because I already have LoraSA folder
6. Open the Project with the VS code Platform.IO
   ![image](https://github.com/user-attachments/assets/5066836b-32ac-4a24-ac03-b5f739a6a658)
   ![image](https://github.com/user-attachments/assets/da14488d-7a4a-410e-b754-59598578016d)

7. Select Proper Environment
   ![image](https://github.com/user-attachments/assets/a9c6557b-a387-4457-b59b-b3d7242d2826)
8. Select ESP32 USB Device to program
![image](https://github.com/user-attachments/assets/af76c4b1-7122-45e1-b26b-08b59e03ca3b)
Note: It is theoretically possible to program via WiFi and BTH.
9. Programm your ESP32
    ![image](https://github.com/user-attachments/assets/9e67afd8-0522-4a96-82dc-8e1cdb32add5)

10. Wait until you are done with the compilation and upload.
    Usually takes 1 minute. The first run is slower. It needs to compile all libraries. 
    ![image](https://github.com/user-attachments/assets/6796eb5d-6e3f-45bc-b88c-251499f1ad47)
You will have the UCOG SA logo and spectrum analyzing scanning screen when done.
![image](https://github.com/user-attachments/assets/f86ab32a-cab1-461e-ade9-7021136a0af7)

 
# Hardware
Heltec ESP32 Lora V3:
https://www.amazon.com/Heltec-Development-863-870MHz-ESP32-S3FN8-902-928MHz/dp/B0D1H1FN9Y/ref=asc_df_B0D1H1FN9Y/?tag=hyprod-20&linkCode=df0&hvadid=692875362841&hvpos=&hvnetw=g&hvrand=5065402673437807142&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9031318&hvtargid=pla-2281435177418&psc=1&mcid=00370fce72f93747a79b8b147a2aa8f1&hvocijid=5065402673437807142-B0D1H1FN9Y-&hvexpln=73&gad_source=1
Battery with Wire JT connector : 
https://www.amazon.com/EEMB-2000mAh-Battery-Rechargeable-Connector/dp/B08214DJLJ?psc=1&pd_rd_w=mU7Pe&content-id=amzn1.sym.3c1bdb31-a20d-42d0-a8cf-ddadf63cd1a8&pf_rd_p=3c1bdb31-a20d-42d0-a8cf-ddadf63cd1a8&pf_rd_r=55CKD999WT1GWQ396J0K&pd_rd_wg=3ydoN&pd_rd_r=715cbd78-a843-4972-a5d5-e26e2716b910&ref_=sspa_dk_detail_2

# 3D printed case
https://www.thingiverse.com/thing:3125854
or buy : 
https://www.amazon.com/DIYmalls-ESP32-OLED-WiFi-Type-C/dp/B0BR3MQ9BG
