#include "wifi_server.h"
#include "file_io.h"
#include <LittleFS.h>

// Parameter name constants
const String SSID = "ssid";
const String PASS = "pass";
const String IP = "ip";
const String GATEWAY = "gateway";
const String FSTART = "fstart";
const String FEND = "fend";

// WiFi config variables
String ssid = "LoraSA", pass = "1234567890", ip = "192.168.1.100",
       gateway = "192.168.1.1", fstart = "", fend = "", smpls = "";

#ifdef WEB_SERVER
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

IPAddress localIP;
IPAddress localGateway;
IPAddress subnet(255, 255, 0, 0);

unsigned long previousMillis = 0;
const long interval = 10000;

bool initWiFi()
{
    Serial.println("SSID:" + ssid);
    Serial.println("PSWD:" + pass);
    Serial.println("IP:" + ip);
    Serial.println("SUB:" + subnet);
    Serial.println("GATAWAY:" + gateway);
    if (ssid == "" || ip == "")
    {
        Serial.println("Undefined SSID or IP address.");
        return false;
    }

    WiFi.mode(WIFI_STA);
    localIP.fromString(ip.c_str());
    localGateway.fromString(gateway.c_str());

    if (!WiFi.config(localIP, localGateway, subnet))
    {
        Serial.println("STA Failed to configure");
        return false;
    }
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.println("Connecting to WiFi...");

    unsigned long currentMillis = millis();
    previousMillis = currentMillis;

    while (WiFi.status() != WL_CONNECTED)
    {
        currentMillis = millis();
        if (currentMillis - previousMillis >= interval)
        {
            Serial.println("Failed to connect.");
            return false;
        }
    }

    Serial.println(WiFi.localIP());
    return true;
}

static void serverServer()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/index.html", "text/html"); });

    server.serveStatic("/", LittleFS, "/");

    server.on("/", HTTP_POST,
              [](AsyncWebServerRequest *request)
              {
                  int params = request->params();
                  for (int i = 0; i < params; i++)
                  {
                      Serial.println("Parameter " + String(i) + ": " +
                                     request->getParam(i)->value());
                  }
                  Serial.println(request->params());

                  String p;
                  if (request->hasParam(IP, true))
                  {
                      p = request->getParam(IP, true)->value();
                      writeParameterToParameterFile(IP, p);
                  }

                  if (request->hasParam(SSID, true))
                  {
                      p = request->getParam(SSID, true)->value();
                      writeParameterToParameterFile(SSID, p);
                  }

                  if (request->hasParam(PASS, true))
                  {
                      p = request->getParam(PASS, true)->value();
                      writeParameterToParameterFile(PASS, p);
                  }

                  if (request->hasParam(GATEWAY, true))
                  {
                      p = request->getParam(GATEWAY, true)->value();
                      writeParameterToParameterFile(GATEWAY, p);
                  }

                  if (request->hasParam(FSTART, true))
                  {
                      p = request->getParam(FSTART, true)->value();
                      writeParameterToParameterFile(FSTART, p);
                  }

                  if (request->hasParam(FEND, true))
                  {
                      p = request->getParam(FEND, true)->value();
                      writeParameterToParameterFile(FEND, p);
                  }

                  if (request->hasParam("samples", true))
                  {
                      p = request->getParam("samples", true)->value();
                      writeParameterToParameterFile("samples", p);
                  }

                  request->send(200, "text/plain",
                                "Done. ESP will restart, connect to your router and "
                                "go to IP address: " +
                                    ip);
                  delay(3000);
                  ESP.restart();
              });

    server.begin();
}

void serverStart()
{
    if (initWiFi())
    {
        Serial.println("Setting Secure WIFI (Access Point)");
        serverServer();
    }
    else
    {
        Serial.println("Setting AP (Access Point)");
        WiFi.softAP("LoraSA", NULL);

        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);

        serverServer();
    }
}
#endif

void writeParameterToFile(String value, String file)
{
    writeFile(LittleFS, file.c_str(), value.c_str());
}

void writeParameterToParameterFile(String param, String value)
{
    String file = String("/" + param + ".txt");
    writeParameterToFile(value, file.c_str());
}

String readParameterFromParameterFile(String param)
{
    String file = String("/" + param + ".txt");
    return readFile(LittleFS, file.c_str());
}
