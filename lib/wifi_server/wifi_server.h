#pragma once

#include <Arduino.h>

// Parameter name constants
extern const String SSID;
extern const String PASS;
extern const String IP;
extern const String GATEWAY;
extern const String FSTART;
extern const String FEND;

// WiFi config variables
extern String ssid, pass, ip, gateway, fstart, fend, smpls;

void writeParameterToFile(String value, String file);
void writeParameterToParameterFile(String param, String value);
String readParameterFromParameterFile(String param);

#ifdef WEB_SERVER
void serverStart();
#endif
