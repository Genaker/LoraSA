#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <WiFi.h>

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const String SSID = "ssid";
const String PASS = "pass";
const String IP = "ip";
const String GATEWAY = "gateway";
const String FSTART = "fstart";
const String FEND = "fend";

// File paths to save input values permanently
// const char *ssidPath = "/ssid.txt";

// Variables to save values from HTML form
String ssid = "LoraSA", pass = "1234567890", ip = "192.168.1.100",
       gateway = "192.168.1.1", fstart = "", fend = "", smpls = "";

IPAddress localIP;
// Set your Gateway IP address
IPAddress localGateway;
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

// Initialize WiFi
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

void writeParameterToFile(String value, String file)
{
    // Write file to save value
    writeFile(LittleFS, file.c_str(), value.c_str());
}

void writeParameterToParameterFile(String param, String value)
{
    String file = String("/" + param + ".txt");
    // Write file to save value
    writeParameterToFile(value, file.c_str());
}

String readParameterFromParameterFile(String param)
{
    String file = String("/" + param + ".txt");
    return readFile(LittleFS, file.c_str());
}

void serverServer()
{
    // Route for root / web page
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

                  if (request->hasParam(IP, true))
                  {
                      p = request->getParam(IP, true)->value();
                      writeParameterToParameterFile(IP, p);
                  }

                  if (request->hasParam(IP, true))
                  {
                      p = request->getParam(IP, true)->value();
                      writeParameterToParameterFile(IP, p);
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

    /* // Route to set GPIO state to HIGH
     server.on("/on", HTTP_GET,
               [](AsyncWebServerRequest *request)
               {
                   digitalWrite(ledPin, HIGH);
                   request->send(LittleFS, "/index.html", "text/html", false,
                                 processor);
               });

     // Route to set GPIO state to LOW
     server.on("/off", HTTP_GET,
               [](AsyncWebServerRequest *request)
               {
                   digitalWrite(ledPin, LOW);
                   request->send(LittleFS, "/index.html", "text/html", false,
                                 processor);
               });*/
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
        // Connect to Wi-Fi network with default SSID and password
        Serial.println("Setting AP (Access Point)");
        // NULL sets an open Access Point
        WiFi.softAP("LoraSA", NULL);

        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);

        serverServer();
    }
}
