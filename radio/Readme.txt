1. Wiring:
Connect the SBUS output from your receiver to the appropriate UART (Universal Asynchronous Receiver-Transmitter) port on your flight controller. This is usually labeled as RX or SBUS on the flight controller.
2. Betaflight Configuration:
Connect to Betaflight Configurator: Use the Betaflight Configurator software to connect to your flight controller via USB.
Ports Tab: In the Configurator, navigate to the "Ports" tab. Enable the UART port where your SBUS receiver is connected. Set the port to "Serial RX".
Configuration Tab: Go to the "Configuration" tab. Under "Receiver", select "Serial-based receiver" and then choose "SBUS" from the protocol dropdown menu.
Save and Reboot: After making these changes, save the configuration and reboot the flight controller.
3. Testing:
After configuration, test the setup by moving the sticks on your transmitter and observing the response in the Betaflight Configurator's "Receiver" tab. The channels should move according to your stick inputs.


Steps to Connect to Betaflight via Browser
1. Download Betaflight Configurator:
Visit the Betaflight Configurator releases page on GitHub. 
https://github.com/betaflight/betaflight-configurator/releases
Download the appropriate version for your operating system (Windows, macOS, or Linux).

go to Port -> select port -> check Serial RX

Go to 

# SBUS tested

ESP32 39 -> SBUS(R2) Any of them
GND -> G
Article explains : https://speedybee.zendesk.com/hc/en-us/articles/19968381088795-How-to-set-up-your-SBUS-receiver-in-Betaflight-configurator-on-SpeedyBee-F405MINI-flight-controller


